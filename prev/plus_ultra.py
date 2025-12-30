import cv2
import serial
import time
import collections
import numpy as np

# ===================== CONFIGURATION =====================

# --- HARDWARE ---
CAMERA_INDEX = 1         # Try 1 if 0 fails (e.g., USB cam)
SERIAL_PORT = "COM3"     # <--- CHANGE THIS to your Arduino Port
BAUDRATE = 115200

# --- TUNING (THE "FEEL" OF THE ROBOT) ---
# 1. Lateral (Left/Right) Settings
DEAD_ZONE_X = 20         # Pixels. Error < 20px? Do nothing.
KP_X = 0.1               # Speed factor: 100px error * 0.1 = 10mm move
INVERT_X = False         # Flip if machine moves Left when it should go Right

# 2. Depth (Forward/Backward) Settings
DEAD_ZONE_DEPTH = 15     # Pixels (Width difference).
KP_DEPTH = 0.2           # Depth usually needs a bit more "kick" than X
INVERT_Y = False         # Flip if machine moves Back when it should go Front

# 3. Safety & Smoothing
MAX_STEP_MM = 10.0       # Max distance allowed in one loop cycle (Safety Cap)
SMOOTHING_BUFFER = 5     # Average last 5 frames to stop jitter

# =========================================================

# --- GLOBAL SERIAL SETUP ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)            # Wait for Arduino reset
    ser.write(b"\r\n\r\n")   # Wake up GRBL
    time.sleep(1)
    ser.flushInput()
    
    print("✅ GRBL Connected")
    
    # 1. SET HOME: Tell GRBL that current position is 0,0
    print("📍 Setting Current Position as HOME (0,0)")
    ser.write(b"G92 X0 Y0\n")
    time.sleep(0.5)

    # 2. SETUP MODES: Millimeters (G21) and Relative Movement (G91)
    ser.write(b"G21 G91\n") 
    ser.write(b"G1 F1500\n") # Set Feedrate (Speed)

except Exception as e:
    print(f"❌ Serial Error: {e}")
    ser = None

# --- SMOOTHING BUFFERS ---
history_x = collections.deque(maxlen=SMOOTHING_BUFFER)
history_width = collections.deque(maxlen=SMOOTHING_BUFFER)

def send_mixed_gcode(axis, val_mm):
    """
    Translates logic to Mixed Kinematics (G91 Relative Mode).
    """
    if ser is None: return

    # Safety Clamp
    val_mm = max(min(val_mm, MAX_STEP_MM), -MAX_STEP_MM)
    
    cmd = ""
    d = abs(val_mm)
    d_str = f"{d:.2f}"

    # --- LATERAL MOVEMENT (X AXIS) ---
    if axis == 'X':
        # Logic: POSITIVE = RIGHT, NEGATIVE = LEFT
        move_right = (val_mm > 0)
        if INVERT_X: move_right = not move_right

        if move_right:
            # Your 'posx' logic: Both motors positive
            cmd = f"X{d_str} Y{d_str}"
        else:
            # Your 'negx' logic: Both motors negative
            cmd = f"X-{d_str} Y-{d_str}"

    # --- DEPTH MOVEMENT (Y AXIS) ---
    elif axis == 'Y':
        # Logic: POSITIVE = FORWARD (Approach), NEGATIVE = BACK (Retreat)
        move_forward = (val_mm > 0)
        if INVERT_Y: move_forward = not move_forward

        if move_forward:
            # Your 'negy' logic: X+ Y-
            cmd = f"X{d_str} Y-{d_str}"
        else:
            # Your 'posy' logic: X- Y+
            cmd = f"X-{d_str} Y{d_str}"

    if cmd:
        # Send Relative Move (G1)
        full_cmd = f"G1 {cmd}\n"
        ser.write(full_cmd.encode())

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("❌ Camera error")
        return

    # Low res for speed
    cap.set(3, 640)
    cap.set(4, 480)

    ret, frame = cap.read()
    if not ret: return

    print("\n--- INSTRUCTIONS ---")
    print("1. Select object to lock onto.")
    print("2. The INITIAL size is the 'Target Distance'.")
    print("3. Press 'q' to Quit and Return to Home.")

    roi = cv2.selectROI("Target Lock", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Target Lock")

    if roi[2] == 0 or roi[3] == 0: 
        print("❌ No selection.")
        return

    # --- INIT TRACKER ---
    try:
        tracker = cv2.TrackerCSRT_create()
    except:
        tracker = cv2.legacy.TrackerCSRT_create()
        
    tracker.init(frame, roi)

    # Set Targets
    TARGET_WIDTH = roi[2]
    frame_h, frame_w = frame.shape[:2]
    TARGET_CENTER_X = frame_w // 2

    print(f"✅ LOCKED. Maintaining Width: {TARGET_WIDTH}px")

    try:
        # --- MAIN LOOP ---
        while True:
            ret, frame = cap.read()
            if not ret: break

            ok, bbox = tracker.update(frame)

            if ok:
                (x, y, w, h) = [int(v) for v in bbox]
                current_center_x = x + w // 2
                current_width = w

                # 1. Smoothing
                history_x.append(current_center_x)
                history_width.append(current_width)
                avg_x = sum(history_x) / len(history_x)
                avg_width = sum(history_width) / len(history_width)

                # 2. Calculate Errors
                error_x = avg_x - TARGET_CENTER_X
                error_width = TARGET_WIDTH - avg_width 

                status_x = "Stable"
                status_y = "Stable"

                # 3. Control Logic
                # -> X Control
                if abs(error_x) > DEAD_ZONE_X:
                    move_mm = error_x * KP_X
                    send_mixed_gcode('X', move_mm)
                    status_x = f"Moving X: {move_mm:.1f}mm"
                
                # -> Depth Control
                # Only adjust depth if X is reasonably stable to avoid confusion
                if abs(error_width) > DEAD_ZONE_DEPTH:
                    move_mm = error_width * KP_DEPTH
                    send_mixed_gcode('Y', move_mm)
                    status_y = f"Adjusting Depth: {move_mm:.1f}mm"

                # 4. Drawing
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.line(frame, (int(avg_x), 0), (int(avg_x), frame_h), (0, 255, 255), 1)
                cv2.line(frame, (TARGET_CENTER_X, 0), (TARGET_CENTER_X, frame_h), (255, 0, 0), 1)

                cv2.putText(frame, f"Dist Err: {error_width:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Lat Err: {error_x:.1f}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, status_x, (10, frame_h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(frame, status_y, (10, frame_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            else:
                cv2.putText(frame, "TRACKING LOST", (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

            cv2.imshow("Follow-Me Robot", frame)
            
            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("🛑 Stop signal received.")
                break

    except KeyboardInterrupt:
        print("⚠️ User Interrupt.")

    finally:
        # --- RETURN TO HOME LOGIC ---
        cap.release()
        cv2.destroyAllWindows()
        
        if ser and ser.is_open:
            print("🏠 Returning to Home Position (0,0)...")
            try:
                # 1. Switch to Absolute Mode (G90)
                # 2. Rapid Move (G0) to X0 Y0
                ser.write(b"G90\n")
                ser.write(b"G0 X0 Y0\n")
                
                # Wait a few seconds for the machine to physically travel back
                time.sleep(5) 
            except Exception as e:
                print(f"Error returning home: {e}")
            
            ser.close()
            print("✅ Connection Closed.")

if __name__ == "__main__":
    main()