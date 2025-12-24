import cv2
import serial
import time
import collections

# ===================== CONFIGURATION =====================

# --- HARDWARE ---
CAMERA_INDEX = 1         
SERIAL_PORT = "COM3"     
BAUDRATE = 115200

# --- SAFETY LIMITS ---
MAX_TRAVEL_LIMIT = 2.9   # Machine stays within +/- 2.9 units

# --- TUNING (ADJUSTED FOR STABILITY) ---

# 1. Lateral (Left/Right)
DEAD_ZONE_X = 30         # Increased: Ignore small shakes
KP_X = 0.05              # Reduced: Move slower/smoother
INVERT_X = True          # <--- CHANGED: Fixes your Left/Right issue

# 2. Depth (Forward/Backward)
DEAD_ZONE_DEPTH = 20     # Increased: Needs to be very wrong before moving
KP_DEPTH = 0.05          # Reduced: Prevents the "Runaway" overshoot
INVERT_Y = False         # Keep False first. If it moves WRONG way, change to True.

# 3. Safety
MAX_STEP_MM = 0.2        # <--- REDUCED: Max step is tiny now to stop crashing
SMOOTHING_BUFFER = 10    # <--- INCREASED: Average last 10 frames for stability

# =========================================================

# --- STATE VARIABLES ---
current_machine_x = 0.0
current_machine_y = 0.0

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    ser.write(b"\r\n\r\n")
    time.sleep(1)
    ser.flushInput()
    print("✅ GRBL Connected")
    ser.write(b"G92 X0 Y0\n") # Set Home
    ser.write(b"G21 G91\n")   # mm and Relative
    ser.write(b"G1 F1000\n")  # Slower Feedrate for safety

except Exception as e:
    print(f"❌ Serial Error: {e}")
    ser = None

# --- SMOOTHING ---
history_x = collections.deque(maxlen=SMOOTHING_BUFFER)
history_width = collections.deque(maxlen=SMOOTHING_BUFFER)

def move_machine_safely(axis, move_val):
    global current_machine_x, current_machine_y, ser
    if ser is None: return

    # 1. Clamp Speed
    move_val = max(min(move_val, MAX_STEP_MM), -MAX_STEP_MM)
    
    # 2. Check Soft Limits
    if axis == 'X':
        predicted_pos = current_machine_x + move_val
        if abs(predicted_pos) > MAX_TRAVEL_LIMIT:
            print(f"⚠️ WALL HIT X: {predicted_pos:.2f}")
            return
        
        current_machine_x += move_val
        
        # INVERT_X HANDLER
        # If Logic says "Move Right" (move_val > 0)
        # We send the command that YOU verified goes Right (X+ Y+)
        # If INVERT_X is True, we flip the sign of move_val before deciding
        
        effective_move = move_val
        if INVERT_X: effective_move = -move_val

        # CoreXY Logic:
        # If effective > 0 (Right): X+ Y+
        # If effective < 0 (Left):  X- Y-
        cmd = f"X{effective_move:.3f} Y{effective_move:.3f}"

    elif axis == 'Y':
        predicted_pos = current_machine_y + move_val
        if abs(predicted_pos) > MAX_TRAVEL_LIMIT:
            print(f"⚠️ WALL HIT Y: {predicted_pos:.2f}")
            return

        current_machine_y += move_val
        
        # INVERT_Y HANDLER
        effective_move = move_val
        if INVERT_Y: effective_move = -move_val

        # Y Logic (Depth):
        # effective > 0 (Move Forward/Down): X+ Y-
        # effective < 0 (Move Back/Up):      X- Y+
        
        if effective_move > 0:
            cmd = f"X{effective_move:.3f} Y-{effective_move:.3f}"
        else:
            d = abs(effective_move)
            cmd = f"X-{d:.3f} Y{d:.3f}"

    full_cmd = f"G1 {cmd}\n"
    ser.write(full_cmd.encode())

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened(): return

    # Standard res
    cap.set(3, 640)
    cap.set(4, 480)

    ret, frame = cap.read()
    if not ret: return

    print("\n--- READY ---")
    roi = cv2.selectROI("Select Object", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Select Object")
    if roi[2] == 0: return

    try:
        tracker = cv2.TrackerCSRT_create()
    except:
        tracker = cv2.legacy.TrackerCSRT_create()
    tracker.init(frame, roi)

    TARGET_WIDTH = roi[2]
    TARGET_CENTER_X = frame.shape[1] // 2
    
    print(f"✅ LOCKED. Target Width: {TARGET_WIDTH}")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            ok, bbox = tracker.update(frame)
            status_text = "Idle"

            if ok:
                (x, y, w, h) = [int(v) for v in bbox]
                cx = x + w // 2
                
                history_x.append(cx)
                history_width.append(w)
                avg_x = sum(history_x) / len(history_x)
                avg_w = sum(history_width) / len(history_width)

                # --- ERROR CALCULATION ---
                # X Error: Target(320) - Current(200) = +120. Machine must move LEFT to catch up.
                # (My previous code did Current - Target, so signs were flipped. 
                # I fixed it here to be standard: Error = Setpoint - ProcessVar)
                
                # Logic: We want the CAMERA to follow the object.
                # If Object is at 100 (Left), Camera is at 320.
                # Camera needs to move LEFT.
                # Error = 100 - 320 = -220. 
                # Negative Error -> Move Negative (Left).
                # My 'move_machine' function maps Neg -> Left.
                # So: Error = Current - Center.
                
                err_x = avg_x - TARGET_CENTER_X
                err_depth = TARGET_WIDTH - avg_w 

                # CONTROL X
                if abs(err_x) > DEAD_ZONE_X:
                    # err_x is pixels (e.g. 50). KP is 0.05. Move = 2.5 units? Too big.
                    # scaled by 0.1 again inside just to be sure.
                    move_req = err_x * KP_X * 0.1 
                    move_machine_safely('X', move_req)
                    status_text = f"X Correction"

                # CONTROL Y (Depth)
                elif abs(err_depth) > DEAD_ZONE_DEPTH:
                    # If err_depth is Positive (Target > Current), Object is Small (Far).
                    # We need to move Forward (+).
                    move_req = err_depth * KP_DEPTH * 0.1
                    move_machine_safely('Y', move_req)
                    
                    if move_req > 0: status_text = "Approaching..."
                    else: status_text = "Backing up..."

                # Visuals
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
            cv2.putText(frame, f"Machine: ({current_machine_x:.2f}, {current_machine_y:.2f})", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, status_text, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            cv2.imshow("Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        print("\n🛑 Returning Home...")
        cap.release()
        cv2.destroyAllWindows()
        if ser:
            ser.write(b"G90\n")
            ser.write(b"G0 X0 Y0\n")
            time.sleep(3)
            ser.close()

if __name__ == "__main__":
    main()