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
MAX_SPEED_LIMIT = 500  # Max Feedrate (mm/min)
JOG_DIST = 1000        # Virtual distance for continuous jog
# KP (Gain): Higher values make the machine more "aggressive" to catch the diver
KP_X = 0.15 
KP_Y = 0.15 

# PHYSICAL LIMITS (mm)
PHYSICAL_W = 1.70 
PHYSICAL_S = -2.60 
PHYSICAL_A = 1.80  
PHYSICAL_D = -1.80 
BRAKE_FACTOR = 0.06 

class AutonomousTracker:
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
            self.is_tracking = False
            print("System Ready. Tracking Active.")
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
        print("Safety/Target Lost: Resetting to Center.")

    def update_motion(self, vx, vy):
        """Sends new jog command only if velocity changes significantly"""
        # 1. Apply Speed Cap
        total_f = (vx**2 + vy**2)**0.5
        if total_f > MAX_SPEED_LIMIT:
            ratio = MAX_SPEED_LIMIT / total_f
            vx, vy = vx * ratio, vy * ratio
            total_f = MAX_SPEED_LIMIT

        # 2. Only update if speed is > 5 (prevents jitter at rest)
        if total_f < 5:
            if self.is_tracking:
                self.ser.write(b'\x85')
                self.is_tracking = False
            return

        self.v_x, self.v_y = vx, vy
        
        # CoreXY Kinematics
        dx = JOG_DIST if vx > 0 else -JOG_DIST
        dy = JOG_DIST if vy > 0 else -JOG_DIST
        
        if INVERT_X: dx *= -1
        if INVERT_Y: dy *= -1
        
        ma, mb = dx - dy, dx + dy
        
        # Immediate interrupt and update
        self.ser.write(b'\x85')
        jog_cmd = f"$J=G91 G21 X{ma:.3f} Y{mb:.3f} F{int(total_f)}\n"
        self.ser.write(jog_cmd.encode())
        self.is_tracking = True

    def run(self):
        cap = cv2.VideoCapture(CAM_INDEX)
        
        while True:
            ret, frame = cap.read()
            if not ret: break
            
            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2

            # 1. YOLO Detection
            results = self.model(frame, conf=0.5, verbose=False)
            target_found = False

            for r in results:
                if r.boxes:
                    target_found = True
                    # Get center of the first detected box
                    box = r.boxes[0].xywh[0].cpu().numpy()
                    tx, ty = box[0], box[1]
                    
                    # 2. Calculate Error (Distance from center)
                    err_x = center_x - tx
                    err_y = center_y - ty
                    
                    # 3. Translate Error to Velocity
                    target_vx = err_x * KP_X
                    target_vy = err_y * KP_Y
                    
                    self.update_motion(target_vx, target_vy)

                    # Visuals
                    cv2.rectangle(frame, (int(tx-box[2]/2), int(ty-box[3]/2)), 
                                 (int(tx+box[2]/2), int(ty+box[3]/2)), (0, 255, 0), 2)
                    break

            # 4. If target lost, stop machine
            if not target_found and self.is_tracking:
                self.stop_and_home()
                self.is_tracking = False

            # 5. Boundary Check
            pos = self.get_real_pos()
            if pos:
                curr_x, curr_y = pos
                total_v = (self.v_x**2 + self.v_y**2)**0.5
                skid = (total_v / 60.0) * BRAKE_FACTOR
                
                if (curr_y >= PHYSICAL_W - skid or curr_y <= PHYSICAL_S + skid or 
                    curr_x >= PHYSICAL_A - skid or curr_x <= PHYSICAL_D + skid):
                    if self.is_tracking:
                        self.stop_and_home()
                        self.is_tracking = False

            cv2.imshow("Diver Autonomous Tracking", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            if key == ord('c'): self.stop_and_home()

        cap.release()
        self.ser.close()

if __name__ == "__main__":
    AutonomousTracker().run()