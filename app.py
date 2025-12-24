import cv2
import serial
import time
import collections

# ===================== CONFIGURATION =====================

# --- HARDWARE ---
CAMERA_INDEX = 1        
SERIAL_PORT = "COM3"     # <--- CHECK THIS
BAUDRATE = 115200

# --- SAFETY LIMITS ---
MAX_TRAVEL_LIMIT = 2.9   # Machine stays within +/- 2.9 units
MAX_SINGLE_STEP = 0.3    # Max combined step size (Safety Speed Limit)

# --- TUNING ---
# 1. Lateral (Left/Right)
DEAD_ZONE_X = 20         
KP_X = 0.08              # Gain for Side-to-Side
INVERT_X = True          # You confirmed this is needed

# 2. Depth (Front/Back)
DEAD_ZONE_DEPTH = 15     
KP_DEPTH = 0.15          # Depth usually needs higher gain to be responsive
INVERT_Y = False         # Standard: Object Big = Move Back

# 3. Smoothing
SMOOTHING_BUFFER = 8     # Average last 8 frames

# =========================================================

# --- STATE VARIABLES ---
cur_machine_x = 0.0
cur_machine_y = 0.0

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    ser.write(b"\r\n\r\n")
    time.sleep(1)
    ser.flushInput()
    print("✅ GRBL Connected")
    ser.write(b"G92 X0 Y0\n") 
    ser.write(b"G21 G91\n") 
    ser.write(b"G1 F1500\n") 

except Exception as e:
    print(f"❌ Serial Error: {e}")
    ser = None

# --- SMOOTHING ---
history_x = collections.deque(maxlen=SMOOTHING_BUFFER)
history_width = collections.deque(maxlen=SMOOTHING_BUFFER)

def send_simultaneous_move(move_right_val, move_front_val):
    """
    Combines Left/Right and Front/Back into a single smooth CoreXY motion.
    """
    global cur_machine_x, cur_machine_y, ser
    if ser is None: return

    # 1. Calculate the Motor Moves based on your Kinematics
    # Your Data:
    # Right (+): X+ Y+
    # Front (+): X+ Y-
    
    # Formula:
    # Gcode_X = Right + Front
    # Gcode_Y = Right - Front
    
    motor_x = move_right_val + move_front_val
    motor_y = move_right_val - move_front_val
    
    # 2. Safety Clamping (Prevent violent jumps)
    # We clamp the vector magnitude approximately by clamping components
    motor_x = max(min(motor_x, MAX_SINGLE_STEP), -MAX_SINGLE_STEP)
    motor_y = max(min(motor_y, MAX_SINGLE_STEP), -MAX_SINGLE_STEP)

    # 3. Soft Limit Check (Predictive)
    pred_x = cur_machine_x + motor_x
    pred_y = cur_machine_y + motor_y
    
    if abs(pred_x) > MAX_TRAVEL_LIMIT or abs(pred_y) > MAX_TRAVEL_LIMIT:
        print("⚠️ WALL HIT PREVENTED")
        return # Skip this move
        
    # 4. Update Position & Send
    cur_machine_x += motor_x
    cur_machine_y += motor_y
    
    # Only move if the values are non-zero (avoids spamming serial)
    if abs(motor_x) > 0.001 or abs(motor_y) > 0.001:
        cmd = f"G1 X{motor_x:.3f} Y{motor_y:.3f}\n"
        ser.write(cmd.encode())
        return True
    return False

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened(): return

    cap.set(3, 640)
    cap.set(4, 480)

    ret, frame = cap.read()
    if not ret: return

    print("\n--- INSTRUCTIONS ---")
    print("Select Object. Initial Width = Target Distance.")
    
    roi = cv2.selectROI("Tracker", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Tracker")
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
            status = "Idle"

            if ok:
                (x, y, w, h) = [int(v) for v in bbox]
                cx = x + w // 2
                
                # Update Smoothing
                history_x.append(cx)
                history_width.append(w)
                avg_x = sum(history_x) / len(history_x)
                avg_w = sum(history_width) / len(history_width)

                # --- 1. CALCULATE RAW ERRORS ---
                # X Error: Target - Current 
                # (Standard Control: Setpoint - ProcessVariable)
                raw_err_x = TARGET_CENTER_X - avg_x  
                
                # Depth Error: Target - Current
                # Positive = Object too small (Far) -> Needs Approach
                raw_err_depth = TARGET_WIDTH - avg_w

                # --- 2. CALCULATE DESIRED MOVES (CARTESIAN) ---
                move_right_req = 0.0
                move_front_req = 0.0

                # X Logic
                if abs(raw_err_x) > DEAD_ZONE_X:
                    # If Camera is Right of Object, Object looks "Left" (x < Center).
                    # err_x = Center - x (Positive).
                    # If object is Left, we must move Left to center it.
                    # So Positive Error -> Negative Move (Left).
                    move_right_req = -1 * raw_err_x * KP_X * 0.1
                    
                    if INVERT_X: move_right_req *= -1

                # Depth Logic
                if abs(raw_err_depth) > DEAD_ZONE_DEPTH:
                    # Positive Error (Far) -> Move Front (+)
                    move_front_req = raw_err_depth * KP_DEPTH * 0.1
                    
                    if INVERT_Y: move_front_req *= -1

                # --- 3. EXECUTE SIMULTANEOUS MOVE ---
                did_move = send_simultaneous_move(move_right_req, move_front_req)
                
                if did_move:
                    status = f"Tracking... X:{move_right_req:.2f} Y:{move_front_req:.2f}"

                # Visuals
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
            # Info
            cv2.putText(frame, f"Machine: {cur_machine_x:.2f}, {cur_machine_y:.2f}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, status, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Simultaneous Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    finally:
        print("\n🛑 Homing...")
        cap.release()
        cv2.destroyAllWindows()
        if ser:
            ser.write(b"G90 G0 X0 Y0\n")
            time.sleep(3)
            ser.close()

if __name__ == "__main__":
    main()