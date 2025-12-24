import cv2
import serial
import time
import collections

# ===================== CONFIGURATION =====================

# --- HARDWARE ---
CAMERA_INDEX = 0         
SERIAL_PORT = "COM3"     # <--- CHECK YOUR PORT
BAUDRATE = 115200

# --- SAFETY LIMITS (CRITICAL) ---
# The machine can only move 3.0 units from center.
# We set it to 2.9 to be safe.
MAX_TRAVEL_LIMIT = 2.9   

# --- TUNING ---
DEAD_ZONE_X = 20         # Pixels
KP_X = 0.1               # Gain for X

DEAD_ZONE_DEPTH = 15     # Pixels (Width difference)
KP_DEPTH = 0.2           # Gain for Y

MAX_STEP_MM = 0.5        # Limit single step size (smoothness)
SMOOTHING_BUFFER = 5     

# =========================================================

# --- STATE VARIABLES ---
# We track where the machine IS, so we don't hit the wall.
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
    
    # 1. SET ZERO: Tell machine "Here is 0,0"
    ser.write(b"G92 X0 Y0\n")
    time.sleep(0.1)
    
    # 2. MODES: mm (G21) and Relative (G91)
    ser.write(b"G21 G91\n") 
    ser.write(b"G1 F1500\n") # Speed

except Exception as e:
    print(f"❌ Serial Error: {e}")
    ser = None

# --- SMOOTHING ---
history_x = collections.deque(maxlen=SMOOTHING_BUFFER)
history_width = collections.deque(maxlen=SMOOTHING_BUFFER)

def move_machine_safely(axis, move_val):
    """
    Checks limits BEFORE sending G-code.
    axis: 'X' or 'Y'
    move_val: requested distance in units (e.g., +0.5 or -0.2)
    """
    global current_machine_x, current_machine_y, ser

    if ser is None: return

    # 1. Clamp the step size (Don't jump too fast)
    move_val = max(min(move_val, MAX_STEP_MM), -MAX_STEP_MM)
    
    # 2. Check Soft Limits (The Wall)
    if axis == 'X':
        predicted_pos = current_machine_x + move_val
        if abs(predicted_pos) > MAX_TRAVEL_LIMIT:
            print(f"⚠️ LIMIT HIT X: {predicted_pos:.2f} (Ignored)")
            return # CANCEL MOVE
        
        # Valid move? Update tracking
        current_machine_x += move_val
        
        # CoreXY Mixing for X (Right/Left)
        # RIGHT (+): X+ Y+
        # LEFT (-):  X- Y-
        cmd = f"X{move_val:.3f} Y{move_val:.3f}"

    elif axis == 'Y':
        predicted_pos = current_machine_y + move_val
        if abs(predicted_pos) > MAX_TRAVEL_LIMIT:
            print(f"⚠️ LIMIT HIT Y: {predicted_pos:.2f} (Ignored)")
            return # CANCEL MOVE

        current_machine_y += move_val

        # CoreXY Mixing for Y (Front/Back)
        # Assuming Positive Move = Forward (Front) = negy (X+ Y-)
        # Assuming Negative Move = Backward (Back) = posy (X- Y+)
        
        # Note: Based on your test "X-3 Y3 moves +Y (Back/Up)"
        # And "X3 Y-3 moves -Y (Front/Down)"
        
        if move_val > 0:
            # Moving "Forward" (closer to camera?) -> usually -Y in machine terms if 0,0 is center
            # Let's trust your test: To move -Y (Front), use X+ Y-
            # Wait, we need to define what "Positive move_val" means.
            # In tracking logic: Positive Error = Object Too Small = Move Closer.
            # If "Closer" means Front, and Front is X3 Y-3:
             cmd = f"X{move_val:.3f} Y-{move_val:.3f}"
        else:
            # Moving "Back" (Away). Your test: +Y is X-3 Y3
            # We need to invert the value because 'move_val' is negative here
            abs_val = abs(move_val)
            cmd = f"X-{abs_val:.3f} Y{abs_val:.3f}"

    # 3. Send Command
    full_cmd = f"G1 {cmd}\n"
    ser.write(full_cmd.encode())


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened(): return

    cap.set(3, 640)
    cap.set(4, 480)

    ret, frame = cap.read()
    if not ret: return

    print("\n--- INSTRUCTIONS ---")
    print(f"1. Machine Soft Limits set to +/- {MAX_TRAVEL_LIMIT} units.")
    print("2. Select object to LOCK TARGET.")
    
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

    print(f"✅ Target Width: {TARGET_WIDTH}. Area bounded to +/- {MAX_TRAVEL_LIMIT}.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            ok, bbox = tracker.update(frame)
            status_text = "Idle"

            if ok:
                (x, y, w, h) = [int(v) for v in bbox]
                cx = x + w // 2
                
                # Smoothing
                history_x.append(cx)
                history_width.append(w)
                avg_x = sum(history_x) / len(history_x)
                avg_w = sum(history_width) / len(history_width)

                # Errors
                err_x = avg_x - TARGET_CENTER_X
                # If Target=100, Curr=80 (Far away) -> Error = +20 -> Move Closer
                err_depth = TARGET_WIDTH - avg_w 

                # CONTROL X
                if abs(err_x) > DEAD_ZONE_X:
                    # Convert pixels to machine units
                    move_req = err_x * KP_X * 0.1 # Scaling factor (Pixels are big, mm is small)
                    # Note: You might need to tune this 0.1 depending on if "3 units" is 3mm or 3cm
                    move_machine_safely('X', move_req)
                    status_text = f"X Err {err_x:.0f}"

                # CONTROL Y (Depth)
                elif abs(err_depth) > DEAD_ZONE_DEPTH:
                    move_req = err_depth * KP_DEPTH * 0.1
                    move_machine_safely('Y', move_req)
                    status_text = f"Z Err {err_depth:.0f}"

                # Visuals
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
            # Info overlay
            cv2.putText(frame, f"Pos: ({current_machine_x:.2f}, {current_machine_y:.2f})", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, status_text, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Safe Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        # RETURN TO HOME
        print("\n🛑 STOPPING. Returning to Home (0,0)...")
        cap.release()
        cv2.destroyAllWindows()
        if ser:
            ser.write(b"G90\n")     # Absolute Mode
            ser.write(b"G0 X0 Y0\n") # Go Home
            time.sleep(3)
            ser.close()
            print("✅ Done.")

if __name__ == "__main__":
    main()