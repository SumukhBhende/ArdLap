import cv2
import serial
import time
import re

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
CAM_INDEX = 1

# INVERSION SETTINGS
INVERT_X = False 
INVERT_Y = True  

JOG_SPEED = 100  # You can change this (e.g., 10, 30, 70)
JOG_DIST = 500  

# PHYSICAL LIMITS (Measured at your slowest speed, e.g., F10)
PHYSICAL_W = 1.70 #1.90 
PHYSICAL_S = -2.60 #-2.80
PHYSICAL_A = 1.80  #1.80
PHYSICAL_D = -1.80 #-1.90 

# LATENCY COMPENSATION
# This compensates for the "skid" at higher speeds.
# 0.06 means the script stops 0.06 seconds 'early' to account for lag.
BRAKE_FACTOR = 0.06 

class SafeController:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            time.sleep(2) 
            self.ser.write(b"\r\n\r\n") 
            time.sleep(1)
            self.ser.write(b"$X\n") # Unlock
            self.ser.write(b"G92 X0 Y0\n") # Zero center
            print(f"Safe Mode Active. Jog Speed: {JOG_SPEED}")
        except Exception as e:
            print(f"Connection Error: {e}")
            exit()

    def get_real_pos(self):
        self.ser.write(b"?")
        status = self.ser.readline().decode('utf-8', errors='ignore')
        match = re.search(r'MPos:([-.\d]+),([-.\d]+),([-.\d]+)', status)
        if match:
            ma, mb = float(match.group(1)), float(match.group(2))
            return (ma + mb) / 2, (mb - ma) / 2
        return None

    def stop_and_home(self, manual=False):
        self.ser.write(b'\x85') # Jog Cancel
        time.sleep(0.1)
        self.ser.write(b"G90 G21 X0 Y0 F800\n") 
        if not manual:
            print("Boundary Safety Triggered.")

    def run(self):
        cap = cv2.VideoCapture(CAM_INDEX)
        current_moving = False

        while True:
            ret, frame = cap.read()
            if not ret: break

            pos = self.get_real_pos()
            if pos:
                curr_x, curr_y = pos
                
                # --- DYNAMIC LIMIT CALCULATION ---
                # This subtracts a 'safety buffer' that grows with speed.
                # Speed (mm/min) / 60 = mm per second.
                skid_margin = (JOG_SPEED / 60.0) * BRAKE_FACTOR
                
                limit_w = PHYSICAL_W - skid_margin
                limit_s = PHYSICAL_S + skid_margin
                limit_a = PHYSICAL_A - skid_margin
                limit_d = PHYSICAL_D + skid_margin

                # Check against Dynamic Limits
                if (curr_y >= limit_w or curr_y <= limit_s or 
                    curr_x >= limit_a or curr_x <= limit_d):
                    if current_moving:
                        print(f"Braking! Margin: {skid_margin:.3f}mm | Pos: X{curr_x:.2f} Y{curr_y:.2f}")
                        self.stop_and_home()
                        current_moving = False

                cv2.putText(frame, f"POS: X{curr_x:.2f} Y{curr_y:.2f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Feedback Control", frame)
            key = cv2.waitKey(1) & 0xFF

            # Movement Logic
            if key in [ord('w'), ord('s'), ord('a'), ord('d')] and not current_moving:
                dx, dy = 0, 0
                if key == ord('w'): dy = JOG_DIST
                elif key == ord('s'): dy = -JOG_DIST
                elif key == ord('a'): dx = JOG_DIST
                elif key == ord('d'): dx = -JOG_DIST
                
                if INVERT_X: dx *= -1
                if INVERT_Y: dy *= -1

                ma, mb = dx - dy, dx + dy
                self.ser.write(f"$J=G91 G21 X{ma:.3f} Y{mb:.3f} F{JOG_SPEED}\n".encode())
                current_moving = True
            
            # --- YOUR CRITICAL KEYBOARD LOGIC ---
            elif key == ord('c'):
                self.stop_and_home(manual=True)
                current_moving = False
            
            elif key == ord('x') or key == ord('q'):
                self.ser.write(b'\x85')
                current_moving = False
                if key == ord('q'):
                    self.ser.write(b'\x85')
                    current_moving = False
                    break

        cap.release()
        self.ser.close()

if __name__ == "__main__":
    SafeController().run()