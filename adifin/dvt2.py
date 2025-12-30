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

JOG_SPEED = 10  
JOG_DIST = 300  


# YOUR MEASURED LIMITS JogSpeed 70 for BOUNDARIES
OLIMIT_W = 4.11  # Max +Y
OLIMIT_S = -3.76 # Max -Y
OLIMIT_A = 4.22  # Max +X
OLIMIT_D = -4.48 # Max -X

# YOUR CALIBRATED LIMITS at JogSpeed 10 for BOUNDARIES
LIMIT_W = (OLIMIT_W/10)*JOG_SPEED  # Max +Y
LIMIT_S = (OLIMIT_S/10)*JOG_SPEED  # Max -Y
LIMIT_A = (OLIMIT_A/10)*JOG_SPEED  # Max +X
LIMIT_D = (OLIMIT_D/10)*JOG_SPEED  # Max -X

class SafeController:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            time.sleep(2) 
            self.ser.write(b"\r\n\r\n") 
            time.sleep(1)
            self.ser.write(b"$X\n") # Unlock
            self.ser.write(b"G92 X0 Y0\n") # Zero center
            print("GRBL 1.1h Safe Mode Active. W-A-S-D to move, X to stop.")
        except Exception as e:
            print(f"Connection Error: {e}")
            exit()

    def get_real_pos(self):
        """Polls GRBL for actual Machine Position (MPos)"""
        self.ser.write(b"?")
        status = self.ser.readline().decode('utf-8', errors='ignore')
        # Regex to find MPos:X,Y,Z
        match = re.search(r'MPos:([-.\d]+),([-.\d]+),([-.\d]+)', status)
        if match:
            # CoreXY Transformation back to Cartesian for boundary check
            # cartesian_x = (motor_a + motor_b) / 2
            # cartesian_y = (motor_b - motor_a) / 2
            ma, mb = float(match.group(1)), float(match.group(2))
            return (ma + mb) / 2, (mb - ma) / 2
        return None

    def stop_and_home(self):
        """Stops all motion immediately and returns home"""
        self.ser.write(b'\x85') # Real-time Jog Cancel
        time.sleep(0.1)
        self.ser.write(b"G90 G21 X0 Y0 F800\n") # Absolute return
        print("Safety Triggered: Returning to Center.")

    def run(self):
        cap = cv2.VideoCapture(CAM_INDEX)
        current_moving = False

        while True:
            ret, frame = cap.read()
            if not ret: break

            # 1. POLL REAL POSITION
            pos = self.get_real_pos()
            if pos:
                curr_x, curr_y = pos
                
                # 2. BOUNDARY CHECK (Hard Stop)
                if (curr_y >= LIMIT_W or curr_y <= LIMIT_S or 
                    curr_x >= LIMIT_A or curr_x <= LIMIT_D):
                    if current_moving:
                        print("current x: ", curr_x, " current y: ", curr_y)
                        self.stop_and_home()
                        current_moving = False

                # HUD
                cv2.putText(frame, f"ACTUAL POS: X{curr_x:.2f} Y{curr_y:.2f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Safe Feedback Control", frame)
            key = cv2.waitKey(1) & 0xFF

            # 3. HANDLE INPUTS
            if key in [ord('w'), ord('s'), ord('a'), ord('d')] and not current_moving:
                dx, dy = 0, 0
                if key == ord('w'): dy = JOG_DIST
                elif key == ord('s'): dy = -JOG_DIST
                elif key == ord('a'): dx = JOG_DIST
                elif key == ord('d'): dx = -JOG_DIST
                
                if INVERT_X: dx *= -1
                if INVERT_Y: dy *= -1

                # CoreXY: MotorA = X - Y | MotorB = X + Y
                ma, mb = dx - dy, dx + dy
                self.ser.write(f"$J=G91 G21 X{ma:.3f} Y{mb:.3f} F{JOG_SPEED}\n".encode())
                current_moving = True

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