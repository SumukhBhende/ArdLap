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

JOG_DIST = 1000 
SPEED_STEP = 10 
MAX_SPEED_LIMIT = 500  # <--- SPEED LIMIT CAP

# PHYSICAL LIMITS
PHYSICAL_W = 1.70 
PHYSICAL_S = -2.60 
PHYSICAL_A = 1.80  
PHYSICAL_D = -1.80 

BRAKE_FACTOR = 0.06 

class VariableSafeController:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            time.sleep(2) 
            self.ser.write(b"\r\n\r\n") 
            time.sleep(1)
            self.ser.write(b"$X\n") 
            self.ser.write(b"G92 X0 Y0\n") 
            
            self.v_x = 0
            self.v_y = 0
            
            print(f"Speed Limit set to: {MAX_SPEED_LIMIT}")
            print("W/S: +/- Y | A/D: +/- X | X: Stop")
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
        self.ser.write(b'\x85') 
        self.v_x = 0
        self.v_y = 0
        time.sleep(0.1)
        self.ser.write(b"G90 G21 X0 Y0 F800\n") 

    def update_motion(self):
        """Forces an update by canceling current jog and sending new one"""
        # 1. Stop current movement so GRBL accepts the new speed
        self.ser.write(b'\x85')
        
        # 2. Check if we should actually move
        if self.v_x == 0 and self.v_y == 0:
            return

        # CoreXY Kinematics
        dx = JOG_DIST if self.v_x > 0 else (-JOG_DIST if self.v_x < 0 else 0)
        dy = JOG_DIST if self.v_y > 0 else (-JOG_DIST if self.v_y < 0 else 0)

        if INVERT_X: dx *= -1
        if INVERT_Y: dy *= -1

        ma, mb = dx - dy, dx + dy
        
        # Calculate Magnitude and apply Speed Limit
        total_f = int((self.v_x**2 + self.v_y**2)**0.5)
        if total_f > MAX_SPEED_LIMIT:
            # Scale down v_x and v_y proportionally if they exceed limit
            ratio = MAX_SPEED_LIMIT / total_f
            self.v_x = int(self.v_x * ratio)
            self.v_y = int(self.v_y * ratio)
            total_f = MAX_SPEED_LIMIT
        
        jog_cmd = f"$J=G91 G21 X{ma:.3f} Y{mb:.3f} F{total_f}\n"
        self.ser.write(jog_cmd.encode())
        print(f"Current Velocity: X={self.v_x} Y={self.v_y} F={total_f}")

    def run(self):
        cap = cv2.VideoCapture(CAM_INDEX)

        while True:
            ret, frame = cap.read()
            if not ret: break

            pos = self.get_real_pos()
            if pos:
                curr_x, curr_y = pos
                current_total_v = (self.v_x**2 + self.v_y**2)**0.5
                skid_margin = (current_total_v / 60.0) * BRAKE_FACTOR
                
                # Boundary Safety
                if (curr_y >= PHYSICAL_W - skid_margin or curr_y <= PHYSICAL_S + skid_margin or 
                    curr_x >= PHYSICAL_A - skid_margin or curr_x <= PHYSICAL_D + skid_margin):
                    if self.v_x != 0 or self.v_y != 0:
                        self.stop_and_home()

                # HUD
                cv2.putText(frame, f"V_X: {self.v_x} V_Y: {self.v_y}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame, f"POS: {curr_x:.2f}, {curr_y:.2f}", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow("Variable Speed Safe Mode", frame)
            key = cv2.waitKey(1) & 0xFF

            # Input Mapping
            if key == ord('w'):
                self.v_y += SPEED_STEP
                self.update_motion()
            elif key == ord('s'):
                self.v_y -= SPEED_STEP
                self.update_motion()
            elif key == ord('a'):
                self.v_x += SPEED_STEP
                self.update_motion()
            elif key == ord('d'):
                self.v_x -= SPEED_STEP
                self.update_motion()
            elif key == ord('c'):
                self.stop_and_home(manual=True)
            elif key == ord('x'):
                self.ser.write(b'\x85')
                self.v_x, self.v_y = 0, 0
            elif key == ord('q'):
                self.ser.write(b'\x85')
                break

        cap.release()
        self.ser.close()

if __name__ == "__main__":
    VariableSafeController().run()