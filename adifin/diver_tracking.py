import cv2
import serial
import time
import re
import numpy as np
from ultralytics import YOLO

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
CAM_INDEX = 1
MODEL_PATH = 'best.pt'

# INVERSION SETTINGS
INVERT_X = False 
INVERT_Y = True  

# VELOCITY & TRACKING SETTINGS
MAX_SPEED_LIMIT = 500  
JOG_DIST = 1000        
KP_X = 0.15 
KP_Y = 0.15 

# PHYSICAL LIMITS (Calibrated)
PHYSICAL_W = 1.70 
PHYSICAL_S = -2.60 
PHYSICAL_A = 1.80  
PHYSICAL_D = -1.80 
BRAKE_FACTOR = 0.06 

class AutonomousSafeTracker:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            time.sleep(2) 
            self.ser.write(b"\r\n\r\n") 
            time.sleep(1)
            self.ser.write(b"$X\n") 
            self.ser.write(b"G92 X0 Y0\n") 
            
            print("Loading YOLO Model...")
            self.model = YOLO(MODEL_PATH)
            
            self.v_x, self.v_y = 0, 0
            self.current_moving = False 
            self.tracking_enabled = True 
            
            print("System Ready.")
            print("X: Toggle Tracking/Pause | C: Return to Center | Q: Home & Quit")
        except Exception as e:
            print(f"Init Error: {e}")
            exit()

    def get_real_pos(self):
        self.ser.write(b"?")
        status = self.ser.readline().decode('utf-8', errors='ignore')
        match = re.search(r'MPos:([-.\d]+),([-.\d]+),([-.\d]+)', status)
        if match:
            ma, mb = float(match.group(1)), float(match.group(2))
            return (ma + mb) / 2, (mb - ma) / 2
        return None

    def stop_and_home(self):
        self.ser.write(b'\x85') 
        self.v_x, self.v_y = 0, 0
        time.sleep(0.1)
        self.ser.write(b"G90 G21 X0 Y0 F800\n") 

    def update_tracking_motion(self, vx, vy):
        if not self.tracking_enabled:
            return

        total_f = (vx**2 + vy**2)**0.5
        if total_f > MAX_SPEED_LIMIT:
            ratio = MAX_SPEED_LIMIT / total_f
            vx, vy = vx * ratio, vy * ratio
            total_f = MAX_SPEED_LIMIT

        if total_f < 5:
            if self.current_moving:
                self.ser.write(b'\x85')
                self.current_moving = False
            return

        self.v_x, self.v_y = vx, vy
        dx = JOG_DIST if vx > 0 else -JOG_DIST
        dy = JOG_DIST if vy > 0 else -JOG_DIST
        if INVERT_X: dx *= -1
        if INVERT_Y: dy *= -1
        ma, mb = dx - dy, dx + dy
        
        self.ser.write(b'\x85')
        jog_cmd = f"$J=G91 G21 X{ma:.3f} Y{mb:.3f} F{int(total_f)}\n"
        self.ser.write(jog_cmd.encode())
        self.current_moving = True

    def run(self):
        cap = cv2.VideoCapture(CAM_INDEX)
        
        while True:
            ret, frame = cap.read()
            if not ret: break
            
            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2

            # 1. Detection Phase
            if self.tracking_enabled:
                results = self.model(frame, conf=0.5, verbose=False)
                target_found = False

                for r in results:
                    if r.boxes:
                        target_found = True
                        box = r.boxes[0].xywh[0].cpu().numpy()
                        tx, ty = box[0], box[1]
                        self.update_tracking_motion((center_x - tx) * KP_X, (center_y - ty) * KP_Y)
                        cv2.rectangle(frame, (int(tx-box[2]/2), int(ty-box[3]/2)), 
                                     (int(tx+box[2]/2), int(ty+box[3]/2)), (0, 255, 0), 2)
                        break

                if not target_found and self.current_moving:
                    self.stop_and_home()
                    self.current_moving = False

            # 2. Status & Position HUD
            pos = self.get_real_pos()
            curr_x, curr_y = pos if pos else (0.0, 0.0)
            
            # --- VISUAL HUD ELEMENTS ---
            if self.tracking_enabled:
                status_txt = "TRACKING"
                color = (0, 255, 0) # Green
            else:
                status_txt = "TRACKING PAUSED"
                color = (0, 0, 255) # Red

            # Draw status background for better readability
            cv2.rectangle(frame, (5, 5), (280, 75), (0, 0, 0), -1)
            cv2.putText(frame, status_txt, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(frame, f"POS: X{curr_x:.2f} Y{curr_y:.2f}", (15, 65), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 3. Boundary Safety (Always running in background)
            if pos:
                total_v = (self.v_x**2 + self.v_y**2)**0.5
                skid = (total_v / 60.0) * BRAKE_FACTOR
                if (curr_y >= PHYSICAL_W - skid or curr_y <= PHYSICAL_S + skid or 
                    curr_x >= PHYSICAL_A - skid or curr_x <= PHYSICAL_D + skid):
                    if self.current_moving:
                        self.stop_and_home()
                        self.current_moving = False

            cv2.imshow("Diver Tracking System", frame)
            
            # --- KEYBOARD LOGIC ---
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('x'):
                if self.tracking_enabled:
                    self.ser.write(b'\x85') 
                    self.tracking_enabled = False
                    self.current_moving = False
                else:
                    self.tracking_enabled = True

            elif key == ord('c'):
                self.tracking_enabled = False 
                self.stop_and_home()
                self.current_moving = False

            elif key == ord('q'):
                self.ser.write(b'\x85')
                self.tracking_enabled = False
                self.current_moving = False
                time.sleep(0.1)
                self.ser.write(b"G90 G21 X0 Y0 F800\n") 
                time.sleep(1.5) 
                break

        cap.release()
        self.ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    AutonomousSafeTracker().run()