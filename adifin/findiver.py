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
TARGET_CLASS = 'diver'  # <--- Strictly track this class only

# INVERSION SETTINGS
INVERT_X, INVERT_Y = False, True  

# VELOCITY & TRACKING SETTINGS
MAX_SPEED_LIMIT = 600  
JOG_DIST = 1000        
KP_X, KD_X = 0.16, 0.06   
KP_Y, KD_Y = 0.12, 0.04   

# ROBUSTNESS SETTINGS
DEADBAND_X = 6 #12           
DEADBAND_Y = 1.0 #2.0          
DETECTION_TIMEOUT = 6     
SMOOTHING_WINDOW = 3 #3 , integer only     
STARTUP_SETTLE_FRAMES = 5 

# PHYSICAL LIMITS
PHYSICAL_W, PHYSICAL_S = 1.70, -2.60 
PHYSICAL_A, PHYSICAL_D = 1.80, -1.80 
BRAKE_FACTOR = 0.06 

class DiverOnlyTracker:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            time.sleep(2); self.ser.write(b"\r\n\r\n"); time.sleep(1)
            self.ser.write(b"$X\n"); self.ser.write(b"G92 X0 Y0\n") 
            
            print("Loading YOLO Model...")
            self.model = YOLO(MODEL_PATH)
            
            # Map class name to ID
            self.target_class_id = None
            for key, value in self.model.names.items():
                if value == TARGET_CLASS:
                    self.target_class_id = key
                    break
            
            if self.target_class_id is None:
                print(f"Warning: Class '{TARGET_CLASS}' not found in model. Tracking first available.")

            self.v_x, self.v_y = 0, 0
            self.last_dir = (0, 0)
            self.tracking_enabled, self.current_moving = True, False
            self.target_size = None  
            self.prev_error_x, self.prev_error_y = 0, 0
            self.missed_frames = 0
            self.settle_counter = 0 
            self.coord_buffer = deque(maxlen=SMOOTHING_WINDOW)
            print(f"System Online. Filtering for class: {TARGET_CLASS}")
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
        if abs(ex) < DEADBAND_X: ex = 0
        if abs(ey) < DEADBAND_Y: ey = 0

        if ex == 0 and ey == 0:
            if self.current_moving:
                self.ser.write(b'\x85'); self.current_moving = False
            return

        vx = (ex * KP_X) + ((ex - self.prev_error_x) * KD_X)
        vy = (ey * KP_Y) + ((ey - self.prev_error_y) * KD_Y)
        self.prev_error_x, self.prev_error_y = ex, ey

        target_f = min(MAX_SPEED_LIMIT, (vx**2 + vy**2)**0.5)
        if target_f < 10:
            if self.current_moving: self.ser.write(b'\x85'); self.current_moving = False
            return

        dir_x, dir_y = (0 if vx == 0 else (1 if vx > 0 else -1)), (0 if vy == 0 else (1 if vy > 0 else -1))
        dx, dy = dir_x * JOG_DIST, dir_y * JOG_DIST
        if INVERT_X: dx *= -1
        if INVERT_Y: dy *= -1
        ma, mb = dx - dy, dx + dy
        
        if (dir_x, dir_y) != self.last_dir:
            self.ser.write(b'\x85')
            self.last_dir = (dir_x, dir_y)

        self.ser.write(f"$J=G91 G21 X{ma:.3f} Y{mb:.3f} F{int(target_f)}\n".encode())
        self.current_moving = True

    def run(self):
        cap = cv2.VideoCapture(CAM_INDEX)
        while True:
            ret, frame = cap.read()
            if not ret: break
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2

            if self.tracking_enabled:
                results = self.model(frame, conf=0.5, verbose=False)
                diver_box = None

                # --- FILTERING LOGIC ---
                for r in results:
                    for box in r.boxes:
                        class_id = int(box.cls[0])
                        # Check if this box belongs to the 'diver' class
                        if self.target_class_id is not None:
                            if class_id == self.target_class_id:
                                diver_box = box.xywh[0].cpu().numpy()
                                break
                        else:
                            # Fallback if class name matching failed
                            diver_box = box.xywh[0].cpu().numpy()
                            break
                    if diver_box is not None: break

                if diver_box is not None:
                    tx, ty, tw, th = diver_box
                    if self.settle_counter < STARTUP_SETTLE_FRAMES:
                        self.settle_counter += 1
                    else:
                        current_size = (tw + th) / 2.0
                        if self.target_size is None:
                            self.target_size = current_size

                        self.missed_frames = 0
                        self.coord_buffer.append((tx, ty, current_size))
                        avg_tx, avg_ty, avg_size = np.mean(self.coord_buffer, axis=0)
                        
                        self.update_motion(cx - avg_tx, self.target_size - avg_size)
                        
                        # Visual Feedback
                        cv2.rectangle(frame, (int(tx-tw/2), int(ty-th/2)), (int(tx+tw/2), int(ty+th/2)), (0, 255, 0), 2)
                        cv2.putText(frame, "DIVER", (int(tx-tw/2), int(ty-th/2)-10), 0, 0.6, (0, 255, 0), 2)
                else:
                    self.settle_counter = 0 
                    self.missed_frames += 1
                    if self.missed_frames > DETECTION_TIMEOUT:
                        self.target_size = None 
                        self.prev_error_x = self.prev_error_y = 0
                        if self.current_moving:
                            self.ser.write(b'\x85'); self.ser.write(b"G90 G21 X0 Y0 F800\n"); self.current_moving = False

            # HUD & Crosshair
            cv2.line(frame, (cx-20, cy), (cx+20, cy), (100,100,100), 1)
            cv2.line(frame, (cx, cy-20), (cx, cy+20), (100,100,100), 1)
            pos = self.get_real_pos()
            curr_x, curr_y = pos if pos else (0,0)
            cv2.rectangle(frame, (5,5), (320, 75), (0,0,0), -1)
            cv2.putText(frame, f"LOCK: {TARGET_CLASS.upper()}", (15, 35), 0, 0.7, (0,255,0), 2)
            cv2.putText(frame, f"X:{curr_x:.2f} Y:{curr_y:.2f}", (15, 65), 0, 0.4, (255,255,255), 1)

            cv2.imshow("Diver-Specific Visual Servo", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('x'):
                self.ser.write(b'\x85'); self.tracking_enabled = not self.tracking_enabled; self.current_moving = False
            elif key == ord('c'):
                self.tracking_enabled = False; self.ser.write(b'\x85'); time.sleep(0.1); self.ser.write(b"G90 G21 X0 Y0 F800\n"); self.current_moving = False
            elif key == ord('q'):
                self.ser.write(b'\x85'); time.sleep(0.1); self.ser.write(b"G90 G21 X0 Y0 F800\n"); time.sleep(1.5); break

        cap.release(); self.ser.close(); cv2.destroyAllWindows()

if __name__ == "__main__":
    DiverOnlyTracker().run()