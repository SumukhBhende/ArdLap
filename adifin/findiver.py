import cv2
import serial
import time
import re
import numpy as np
from collections import deque
from ultralytics import YOLO

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
CAM_INDEX = 1
MODEL_PATH = './adifin/best.pt'

# INVERSION SETTINGS
INVERT_X, INVERT_Y = False, True  

# VELOCITY & TRACKING SETTINGS
MAX_SPEED_LIMIT = 600  
JOG_DIST = 1000        
KP_X, KP_Y = 0.16, 0.16   
KD_X, KD_Y = 0.06, 0.06   
MAX_ACCEL = 50            # Internal Feedrate Accel Limiter

# ROBUSTNESS SETTINGS
DEADBAND = 12             
DETECTION_TIMEOUT = 6     
SMOOTHING_WINDOW = 3      
STARTUP_SETTLE_FRAMES = 5 # ❌ Added: Settling phase to prevent initial jerk

# PHYSICAL LIMITS
PHYSICAL_W, PHYSICAL_S = 1.70, -2.60 
PHYSICAL_A, PHYSICAL_D = 1.80, -1.80 
BRAKE_FACTOR = 0.06 

class DiverTrackerFinal:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            time.sleep(2); self.ser.write(b"\r\n\r\n"); time.sleep(1)
            self.ser.write(b"$X\n"); self.ser.write(b"G92 X0 Y0\n") 
            self.model = YOLO(MODEL_PATH)
            
            # State Variables
            self.v_x, self.v_y = 0, 0
            self.last_dir = (0, 0)
            self.last_f = 0
            self.tracking_enabled, self.current_moving = True, False
            
            # Error Memory
            self.prev_error_x, self.prev_error_y = 0, 0
            self.missed_frames = 0
            self.settle_counter = 0 # Track startup settling
            self.coord_buffer = deque(maxlen=SMOOTHING_WINDOW)
            print("System Online. Mode: Production Baseline.")
        except Exception as e:
            print(f"Init Error: {e}"); exit()

    def get_real_pos(self):
        self.ser.write(b"?")
        status = self.ser.readline().decode('utf-8', errors='ignore')
        match = re.search(r'MPos:([-.\d]+),([-.\d]+),([-.\d]+)', status)
        if match:
            ma, mb = float(match.group(1)), float(match.group(2))
            return (ma + mb) / 2, (mb - ma) / 2
        return None

    def update_motion(self, ex, ey):
        if abs(ex) < DEADBAND: ex = 0
        if abs(ey) < DEADBAND: ey = 0

        if ex == 0 and ey == 0:
            if self.current_moving:
                self.ser.write(b'\x85'); self.current_moving = False; self.last_f = 0
            return

        # PD Calculation
        vx = (ex * KP_X) + ((ex - self.prev_error_x) * KD_X)
        vy = (ey * KP_Y) + ((ey - self.prev_error_y) * KD_Y)
        self.prev_error_x, self.prev_error_y = ex, ey

        # Magnitude and Accel Limiting
        target_f = min(MAX_SPEED_LIMIT, (vx**2 + vy**2)**0.5)
        
        # 🔧 Fix B: Don't send jog if feed is extremely low (prevents buzzing)
        if target_f < 10:
            if self.current_moving:
                self.ser.write(b'\x85'); self.current_moving = False
            return

        if abs(target_f - self.last_f) > MAX_ACCEL:
            target_f = self.last_f + (MAX_ACCEL if target_f > self.last_f else -MAX_ACCEL)
        self.last_f = target_f

        # 🔧 Fix A: Handle zero-velocity direction safely
        dir_x = 0 if vx == 0 else (1 if vx > 0 else -1)
        dir_y = 0 if vy == 0 else (1 if vy > 0 else -1)
        
        dx = 0 if dir_x == 0 else dir_x * JOG_DIST
        dy = 0 if dir_y == 0 else dir_y * JOG_DIST
        
        if INVERT_X: dx *= -1
        if INVERT_Y: dy *= -1
        ma, mb = dx - dy, dx + dy
        
        # De-duplication Logic
        if (dir_x, dir_y) != self.last_dir:
            self.ser.write(b'\x85')
            self.last_dir = (dir_x, dir_y)

        self.ser.write(f"$J=G91 G21 X{ma:.3f} Y{mb:.3f} F{int(target_f)}\n".encode())
        self.v_x, self.v_y, self.current_moving = vx, vy, True

    def run(self):
        cap = cv2.VideoCapture(CAM_INDEX)
        while True:
            ret, frame = cap.read()
            if not ret: break
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2

            if self.tracking_enabled:
                results = self.model(frame, conf=0.5, verbose=False)
                target = next(( (b.xywh[0][0].cpu().numpy(), b.xywh[0][1].cpu().numpy()) 
                                for r in results if r.boxes for b in r.boxes ), None)

                if target:
                    # ❌ Settlement check to prevent initial "jump"
                    if self.settle_counter < STARTUP_SETTLE_FRAMES:
                        self.settle_counter += 1
                        target = None # Ignore until settled
                    else:
                        self.missed_frames = 0
                        self.coord_buffer.append(target)
                        avg_tx, avg_ty = np.mean(self.coord_buffer, axis=0)
                        self.update_motion(cx - avg_tx, cy - avg_ty)
                        cv2.circle(frame, (int(avg_tx), int(avg_ty)), 5, (0, 255, 0), -1)
                else:
                    self.settle_counter = 0 # Reset on loss
                    self.missed_frames += 1
                    if self.missed_frames > DETECTION_TIMEOUT:
                        self.prev_error_x = self.prev_error_y = 0
                        if self.current_moving:
                            self.ser.write(b'\x85'); self.v_x = self.v_y = self.last_f = 0
                            self.ser.write(b"G90 G21 X0 Y0 F800\n"); self.current_moving = False

            # HUD
            pos = self.get_real_pos()
            curr_x, curr_y = pos if pos else (0,0)
            cv2.rectangle(frame, (5,5), (280, 75), (0,0,0), -1)
            cv2.putText(frame, "TRACKING" if self.tracking_enabled else "PAUSED", (15, 35), 0, 0.8, (0,255,0) if self.tracking_enabled else (0,0,255), 2)
            cv2.putText(frame, f"X:{curr_x:.2f} Y:{curr_y:.2f}", (15, 65), 0, 0.5, (255,255,255), 1)

            cv2.imshow("Production Diver Tracker", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('x'):
                self.ser.write(b'\x85'); self.tracking_enabled = not self.tracking_enabled; self.current_moving = False
            elif key == ord('c'):
                self.tracking_enabled = False; self.ser.write(b'\x85'); time.sleep(0.1); self.ser.write(b"G90 G21 X0 Y0 F800\n"); self.current_moving = False
            elif key == ord('q'):
                self.ser.write(b'\x85'); time.sleep(0.1); self.ser.write(b"G90 G21 X0 Y0 F800\n"); time.sleep(1.5); break

        cap.release(); self.ser.close(); cv2.destroyAllWindows()

if __name__ == "__main__":
    DiverTrackerFinal().run()