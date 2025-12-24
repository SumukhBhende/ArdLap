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
MAX_TRAVEL_LIMIT = 2.9   # Machine units (normalized)
MAX_SINGLE_STEP = 0.2    
MIN_TRACKING_SIZE = 20   

# --- TUNING (PD CONTROLLER) ---
# NOTE: Gains are in "Normalized Motion Units per Pixel"
# 1. Lateral (Left/Right)
DEAD_ZONE_X = 30         
KP_X = 0.04              
KD_X = 0.03              
INVERT_X = True          # If True: Moving Left = Negative Machine Coordinate

# 2. Depth (Front/Back)
DEAD_ZONE_DEPTH = 5      
KP_DEPTH = 0.15          
KD_DEPTH = 0.10          
INVERT_Y = False         

# 3. Smoothing
SMOOTHING_BUFFER = 10    
LOOP_DELAY = 0.02        # Enforce ~50Hz max to keep derivative stable

# =========================================================

cur_machine_x = 0.0
cur_machine_y = 0.0
prev_error_x = 0.0
prev_error_depth = 0.0

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
    ser.write(b"G1 F1000\n") 

except Exception as e:
    print(f"❌ Serial Error: {e}")
    ser = None

history_x = collections.deque(maxlen=SMOOTHING_BUFFER)
history_size = collections.deque(maxlen=SMOOTHING_BUFFER)

def send_smart_move(req_x, req_y):
    """
    Checks limits independently and sends Mixed Kinematics G-Code.
    """
    global cur_machine_x, cur_machine_y, ser
    if ser is None: return "NO SERIAL"

    # 1. INDEPENDENT LIMIT CHECKS
    pred_x = cur_machine_x + req_x
    pred_y = cur_machine_y + req_y
    
    blocked_x = False
    blocked_y = False

    if abs(pred_x) > MAX_TRAVEL_LIMIT:
        req_x = 0.0
        blocked_x = True
        
    if abs(pred_y) > MAX_TRAVEL_LIMIT:
        req_y = 0.0
        blocked_y = True

    if blocked_x and blocked_y: return "BLOCKED"
        
    # 2. COREXY MIXING
    # Formula: Motor A = X + Y, Motor B = X - Y
    motor_a = req_x + req_y
    motor_b = req_x - req_y
    
    # Clamp Speed (Safety)
    motor_a = max(min(motor_a, MAX_SINGLE_STEP), -MAX_SINGLE_STEP)
    motor_b = max(min(motor_b, MAX_SINGLE_STEP), -MAX_SINGLE_STEP)
    
    # 3. UPDATE & SEND
    if not blocked_x: cur_machine_x += req_x
    if not blocked_y: cur_machine_y += req_y
    
    if abs(motor_a) > 0.001 or abs(motor_b) > 0.001:
        cmd = f"G1 X{motor_a:.3f} Y{motor_b:.3f}\n"
        
        # [DEBUG] Print the exact command being sent
        print(f"[GCODE] {cmd.strip()}") 
        
        ser.write(cmd.encode())
        
        if blocked_x: return "WALL-X (Sliding Y)"
        if blocked_y: return "WALL-Y (Sliding X)"
        return "MOVING"
        
    return "IDLE"

def main():
    global prev_error_x, prev_error_depth

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened(): return
    cap.set(3, 640)
    cap.set(4, 480)

    ret, frame = cap.read()
    if not ret: return

    print("\n--- INSTRUCTIONS ---")
    print("1. Select Object.")
    print("2. 'r' to RESET target size.")
    print("3. 'x' to RE-SELECT.")
    
    roi = cv2.selectROI("Tracker", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Tracker")
    if roi[2] == 0: return

    try:
        tracker = cv2.TrackerCSRT_create()
    except:
        tracker = cv2.legacy.TrackerCSRT_create()
    tracker.init(frame, roi)

    TARGET_SIZE = (roi[2] + roi[3]) / 2.0
    TARGET_CENTER_X = frame.shape[1] // 2
    
    print(f"✅ LOCKED. Target Size: {TARGET_SIZE:.1f}")

    try:
        while True:
            # Start loop timer
            start_time = time.time()
            
            ret, frame = cap.read()
            if not ret: break

            ok, bbox = tracker.update(frame)
            status = "Idle"
            warning = ""

            if ok:
                (x, y, w, h) = [int(v) for v in bbox]
                current_size = (w + h) / 2.0
                cx = x + w // 2

                if current_size < MIN_TRACKING_SIZE:
                    warning = "⚠️ OBJ TOO SMALL"
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                else:
                    # History Smoothing
                    history_x.append(cx)
                    history_size.append(current_size)
                    avg_x = sum(history_x) / len(history_x)
                    avg_sz = sum(history_size) / len(history_size)

                    # --- 1. CLEAN ERROR CALCULATION ---
                    # Raw Error
                    curr_err_x = TARGET_CENTER_X - avg_x  
                    curr_err_depth = TARGET_SIZE - avg_sz

                    # Apply Inversion Logic EARLY (Refactoring Point 4)
                    # Now "Positive Error" always means "Move Positive"
                    if INVERT_X: curr_err_x *= -1
                    if INVERT_Y: curr_err_depth *= -1

                    move_x = 0.0
                    move_y = 0.0

                    # --- 2. PD CONTROL ---
                    # "Normalized Motion Units" logic: Scaling * 0.1 is absorbed into gains.
                    
                    # X Axis
                    if abs(curr_err_x) > DEAD_ZONE_X:
                        # Assumption: Δt is approx constant due to camera FPS & sleep
                        deriv_x = curr_err_x - prev_error_x
                        output_x = (curr_err_x * KP_X) + (deriv_x * KD_X)
                        
                        # Apply standard direction mapping
                        # Standard Logic: Error is (Target - Current). 
                        # If Target > Current (Positive Error), we usually need Positive Move.
                        # But in Camera Frame: If Target(Center) > Object(Left), Error is (+).
                        # We need to move Camera LEFT (-) to center it. 
                        # So Logic is naturally inverted: Move = -1 * Output.
                        move_x = -1 * output_x * 0.1
                    else:
                        prev_error_x = curr_err_x 

                    # Y Axis
                    if abs(curr_err_depth) > DEAD_ZONE_DEPTH:
                        deriv_depth = curr_err_depth - prev_error_depth
                        output_depth = (curr_err_depth * KP_DEPTH) + (deriv_depth * KD_DEPTH)
                        move_y = output_depth * 0.1
                    else:
                        prev_error_depth = curr_err_depth

                    # Update Previous Errors
                    prev_error_x = curr_err_x
                    prev_error_depth = curr_err_depth

                    # --- 3. MOVE ---
                    status = send_smart_move(move_x, move_y)
                    
                    # Visuals
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    label = f"Sz:{avg_sz:.0f} Tgt:{TARGET_SIZE:.0f}"
                    cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            else:
                warning = "TRACKING FAIL"
                
            # Info Overlay
            cv2.putText(frame, f"Pos: {cur_machine_x:.2f}, {cur_machine_y:.2f}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, status, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if warning:
                cv2.putText(frame, warning, (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

            cv2.imshow("Final Clean Tracker", frame)
            
            # --- LOOP TIMING CONTROL ---
            # Ensures dt is approximately constant for valid Derivative calculation
            elapsed = time.time() - start_time
            if elapsed < LOOP_DELAY:
                time.sleep(LOOP_DELAY - elapsed)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): 
                break
            elif key == ord('r'): 
                if ok:
                    TARGET_SIZE = (w + h) / 2.0
                    print("🔄 Target Resized")
            elif key == ord('x'):
                roi = cv2.selectROI("Tracker", frame, showCrosshair=True, fromCenter=False)
                cv2.destroyWindow("Tracker")
                if roi[2] != 0:
                    tracker.init(frame, roi)
                    TARGET_SIZE = (roi[2] + roi[3]) / 2.0

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