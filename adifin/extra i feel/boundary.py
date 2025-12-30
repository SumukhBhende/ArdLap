import cv2
import serial
import time
import math

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
CAM_INDEX = 1

# INVERSION SETTINGS
INVERT_X = False 
INVERT_Y = True  

# CALIBRATION SETTINGS
JOG_SPEED = 10   # Keeping your requested speed
JOG_DIST = 300   # Large distance for "infinite" travel

def run_boundary_calibration():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) 
        ser.write(b"\r\n\r\n") 
        time.sleep(1)
        ser.write(b"$X\n") # Unlock Alarm
        # Set current position as (0,0)
        ser.write(b"G92 X0 Y0\n") 
        print("Calibration Mode Active. Machine centered at (0,0).")
        print("W-A-S-D to start, X to stop at boundary.")
    except Exception as e:
        print(f"Connection Error: {e}")
        return

    cap = cv2.VideoCapture(CAM_INDEX)
    current_moving_key = None
    start_time = 0
    
    # Store boundaries
    boundaries = {"W": 0.0, "S": 0.0, "A": 0.0, "D": 0.0}

    while True:
        ret, frame = cap.read()
        if not ret: break

        status_text = f"LAST BOUNDARY: {list(boundaries.items())[-1] if current_moving_key else 'NONE'}"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("Boundary Calibration", frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        # 1. START MOVEMENT
        if key in [ord('w'), ord('s'), ord('a'), ord('d')] and current_moving_key is None:
            ser.write(b'\x85') # Safety clear
            
            dx, dy = 0, 0
            if key == ord('w'): dy = JOG_DIST
            elif key == ord('s'): dy = -JOG_DIST
            elif key == ord('a'): dx = JOG_DIST
            elif key == ord('d'): dx = -JOG_DIST
            
            if INVERT_X: dx *= -1
            if INVERT_Y: dy *= -1

            ma = dx - dy
            mb = dx + dy
            
            jog_cmd = f"$J=G91 G21 X{ma:.3f} Y{mb:.3f} F{JOG_SPEED}\n"
            ser.write(jog_cmd.encode())
            
            current_moving_key = chr(key).upper()
            start_time = time.time()
            print(f"Calibration started for direction: {current_moving_key}")

        # 2. STOP AND CALCULATE DISTANCE
        elif key == ord('x') and current_moving_key is not None:
            ser.write(b'\x85') # Stop machine
            end_time = time.time()
            
            # Distance = (Speed mm/min / 60) * seconds
            travelled = (JOG_SPEED / 60.0) * (end_time - start_time)
            boundaries[current_moving_key] = round(travelled, 2)
            
            print(f"--- BOUNDARY REACHED ---")
            print(f"Direction {current_moving_key}: {boundaries[current_moving_key]} mm")
            
            # 3. AUTO-RETURN TO CENTER
            print("Returning to center...")
            time.sleep(0.5)
            # Standard G1 Absolute return
            ser.write(b"G90 G21 X0 Y0 F1000\n") 
            
            # Reset state for next direction
            current_moving_key = None

        elif key == ord('q'):
            ser.write(b'\x85') # Stop machine
            print("Returning to center...")
            time.sleep(0.5)
            # Standard G1 Absolute return
            ser.write(b"G90 G21 X0 Y0 F1000\n")
            break

    print("\nFINAL CALIBRATED BOUNDARIES:")
    for direction, dist in boundaries.items():
        print(f"Direction {direction}: {dist} mm")
    
    cap.release()
    ser.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_boundary_calibration()