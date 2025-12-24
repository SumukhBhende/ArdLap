import cv2
import serial
import time

# ===================== CONFIGURATION =====================
CAMERA_INDEX = 0      # Try 1 if 0 doesn't work
SERIAL_PORT = "COM3"  # Change to your port
BAUDRATE = 115200

# Tuning
DEAD_ZONE = 20        # Pixels (Increased to stop jitter)
PIXEL_TO_MM = 0.05    # Conversion ratio (tune this for distance)
MOVE_DELAY = 0.05     # Delay between commands to let motors react

# =========================================================

# 1. Setup Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2) # Allow Arduino to reset
    ser.write(b"\r\n\r\n") # Wake up GRBL
    time.sleep(1)
    ser.flushInput()
    print("✅ GRBL Connected")
    
    # Set to Relative Mode (G91) and Millimeters (G21)
    ser.write(b"G21 G91\n") 
    # Set speed (F value)
    ser.write(b"G1 F2000\n") 
except Exception as e:
    print(f"❌ Serial Error: {e}")
    ser = None

# 2. Your Custom Kinematics Functions (G91 Relative)
def move_machine(direction, distance_mm):
    if ser is None: return
    
    cmd = ""
    d = f"{distance_mm:.2f}"
    
    # YOUR specific logic for mixed axis movement
    if direction == "RIGHT":  # posx
        cmd = f"X{d} Y{d}"
    elif direction == "LEFT": # negx
        cmd = f"X-{d} Y-{d}"
    elif direction == "UP":   # posy (Camera Up / Machine Back)
        cmd = f"X-{d} Y{d}"
    elif direction == "DOWN": # negy (Camera Down / Machine Front)
        cmd = f"X{d} Y-{d}"
        
    if cmd:
        # Send G1 (Linear Move) + the relative coordinates
        full_cmd = f"G1 {cmd}\n"
        # print(f"Sending: {full_cmd.strip()}") 
        ser.write(full_cmd.encode())

# ===================== MAIN LOOP =====================
def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("❌ Camera not found")
        return

    ret, frame = cap.read()
    if not ret: return

    print("\n--- INSTRUCTIONS ---")
    print("1. Click and drag a box around the object (Top-Left to Bottom-Right).")
    print("2. Press ENTER or SPACE to start tracking.")
    
    # FIX: fromCenter must be False for standard dragging
    roi = cv2.selectROI("Tracker Setup", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Tracker Setup")

    if roi[2] == 0 or roi[3] == 0:
        print("❌ No selection made.")
        return

    # Initialize Tracker
    try:
        tracker = cv2.TrackerCSRT_create()
    except AttributeError:
        tracker = cv2.legacy.TrackerCSRT_create()
        
    tracker.init(frame, roi)
    print("✅ Tracking Started... Press 'q' to Quit.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # Update Tracker
        success, bbox = tracker.update(frame)

        if success:
            x, y, w, h = [int(v) for v in bbox]
            
            # Calculate Centers
            obj_cx = x + w // 2
            obj_cy = y + h // 2
            
            h_frame, w_frame = frame.shape[:2]
            frame_cx = w_frame // 2
            frame_cy = h_frame // 2
            
            error_x = obj_cx - frame_cx
            error_y = obj_cy - frame_cy

            # --- MOTION CONTROL ---
            # We move ONE axis at a time to prevent math conflict in G91
            
            # X-AXIS CORRECTION
            if abs(error_x) > DEAD_ZONE:
                dist = abs(error_x) * PIXEL_TO_MM
                if error_x > 0:
                    move_machine("RIGHT", dist)
                else:
                    move_machine("LEFT", dist)
                    
            # Y-AXIS CORRECTION
            elif abs(error_y) > DEAD_ZONE:
                dist = abs(error_y) * PIXEL_TO_MM
                if error_y > 0:
                    # Object is "lower" in pixel coordinates (positive Y), 
                    # so camera must move DOWN to catch it.
                    move_machine("DOWN", dist)
                else:
                    move_machine("UP", dist)

            # Draw UI
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.line(frame, (frame_cx, frame_cy), (obj_cx, obj_cy), (0, 255, 255), 2)
            cv2.putText(frame, f"Err X:{error_x} Y:{error_y}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "LOST TRACKING", (10, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Show Crosshair
        h_frame, w_frame = frame.shape[:2]
        cv2.circle(frame, (w_frame // 2, h_frame // 2), 5, (255, 0, 0), -1)
        
        cv2.imshow("CNC Tracker", frame)
        
        # Small delay to prevent flooding the serial buffer
        if success:
            time.sleep(MOVE_DELAY)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser: ser.close()

if __name__ == "__main__":
    main()