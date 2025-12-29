import cv2
import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
CAM_INDEX = 1

# INVERSION SETTINGS
INVERT_X = False  # Set to True if A/D keys move the wrong way
INVERT_Y = True   # Set to True if W/S keys move the wrong way

# MOVEMENT SETTINGS
STEP_SIZE = 0.2   # mm per press (Increased for visibility; adjust as needed)
FEED_RATE = 1000  # mm/min (Increased for smoother manual response)

def run_manual_control():
    # 1. Connect to GRBL 0.9j
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)  # Wait for boot
        ser.write(b"\r\n\r\n") 
        time.sleep(1)
        ser.write(b"$X\n")     # Unlock Alarm
        ser.write(b"G21\n")    # Millimeters
        ser.write(b"G91\n")    # Relative Positioning
        print("Connected to GRBL 0.9j. Home position set at current location.")
        print("Controls: W-A-S-D or Arrows. Q to return home and quit.")
    except Exception as e:
        print(f"Connection Error: {e}")
        return

    # Tracking Variables
    total_x = 0.0
    total_y = 0.0

    # 2. Open Camera
    cap = cv2.VideoCapture(CAM_INDEX)
    
    while True:
        ret, frame = cap.read()
        if not ret: break

        # Visual Interface
        h, w, _ = frame.shape
        cv2.line(frame, (w//2, 0), (w//2, h), (100, 100, 100), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (100, 100, 100), 1)
        cv2.putText(frame, f"X: {total_x:.2f} Y: {total_y:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("GRBL Manual Control", frame)
        
        # 3. Handle Key Presses
        key = cv2.waitKey(1) & 0xFF
        dx, dy = 0.0, 0.0
        
        if key == ord('w') or key == 82:   # Up
            dy = STEP_SIZE
        elif key == ord('s') or key == 84: # Down
            dy = -STEP_SIZE
        elif key == ord('a') or key == 81: # Left
            dx = STEP_SIZE
        elif key == ord('d') or key == 83: # Right
            dx = -STEP_SIZE
        elif key == ord('q'):
            break

        # 4. Process Movement
        if dx != 0 or dy != 0:
            # Apply Axis Inversion
            if INVERT_X: dx *= -1
            if INVERT_Y: dy *= -1

            # Update position tracking
            total_x += dx
            total_y += dy

            # CoreXY Transformation: MotorA = X - Y | MotorB = X + Y
            ma = dx - dy
            mb = dx + dy
            
            gcode = f"G1 X{ma:.3f} Y{mb:.3f} F{FEED_RATE}\n"
            ser.write(gcode.encode())

    # 5. Return Home Sequence
    print(f"Returning to Home... Moving X:{-total_x:.2f} Y:{-total_y:.2f}")
    
    # Calculate return deltas
    rev_dx = -total_x
    rev_dy = -total_y
    
    # CoreXY Transformation for the return trip
    rev_ma = rev_dx - rev_dy
    rev_mb = rev_dx + rev_dy
    
    # Send the homing command
    home_gcode = f"G1 X{rev_ma:.3f} Y{rev_mb:.3f} F{FEED_RATE}\n"
    ser.write(home_gcode.encode())
    
    # Wait briefly for the command to be processed before closing serial
    time.sleep(abs(max(rev_ma, rev_mb)) / (FEED_RATE / 60) + 1)

    # Cleanup
    print("Closing...")
    ser.write(b"!") # Final safety stop
    cap.release()
    ser.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_manual_control()