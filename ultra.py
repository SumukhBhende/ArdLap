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
MAX_TRAVEL_LIMIT = 2.9   
MAX_SINGLE_STEP = 0.2    # Kept low for safety

# --- TUNING (PD CONTROLLER) ---
# 1. Lateral (Left/Right) - X AXIS
DEAD_ZONE_X = 30         
KP_X = 0.04              # Gentle P-gain
KD_X = 0.03              # Strong D-gain (Braking) to stop oscillation
INVERT_X = True          

# 2. Depth (Front/Back) - Y AXIS (HIGH SENSITIVITY)
# We track (Width + Height) / 2.
DEAD_ZONE_DEPTH = 5      # <--- VERY LOW: React to small size changes
KP_DEPTH = 0.15          # <--- HIGH: Strong reaction to size change
KD_DEPTH = 0.10          # <--- HIGH: Brake hard to prevent "Runaway"
INVERT_Y = False         

# 3. Smoothing
SMOOTHING_BUFFER = 10    # Heavy smoothing to handle the low deadzone

# =========================================================

# --- STATE VARIABLES ---
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

def send_simultaneous_move(move_right_val, move_front_val):
    global cur_machine_x, cur_machine_y, ser
    if ser is None: return

    # CoreXY Mixing
    motor_x = move_right_val + move_front_val
    motor_y = move_right_val - move_front_val
    
    # Clamp Speed
    motor_x = max(min(motor_x, MAX_SINGLE_STEP), -MAX_SINGLE_STEP)
    motor_y = max(min(motor_y, MAX_SINGLE_STEP), -MAX_SINGLE_STEP)

    # Soft Limit Check
    pred_x = cur_machine_x + motor_x
    pred_y = cur_machine_y + motor_y
    
    if abs(pred_x) > MAX_TRAVEL_LIMIT or abs(pred_y) > MAX_TRAVEL_LIMIT:
        print("⚠️ WALL HIT PREVENTED")
        return False
        
    cur_machine_x += motor_x
    cur_machine_y += motor_y
    
    if abs(motor_x) > 0.001 or abs(motor_y) > 0.001:
        cmd = f"G1 X{motor_x:.3f} Y{motor_y:.3f}\n"
        ser.write(cmd.encode())
        return True
    return False

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
    print("2. The INITIAL SIZE will be maintained.")
    print("3. Press 'r' anytime to reset target distance.")
    
    roi = cv2.selectROI("Tracker", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Tracker")
    if roi[2] == 0: return

    try:
        tracker = cv2.TrackerCSRT_create()
    except:
        tracker = cv2.legacy.TrackerCSRT_create()
    tracker.init(frame, roi)

    # TARGET: Average of Width and Height
    TARGET_SIZE = (roi[2] + roi[3]) / 2.0
    TARGET_CENTER_X = frame.shape[1] // 2
    
    print(f"✅ LOCKED. Target Size: {TARGET_SIZE:.1f}px")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            ok, bbox = tracker.update(frame)
            status = "Idle"

            if ok:
                (x, y, w, h) = [int(v) for v in bbox]
                cx = x + w // 2
                current_size = (w + h) / 2.0
                
                # --- SMOOTHING ---
                history_x.append(cx)
                history_size.append(current_size)
                
                avg_x = sum(history_x) / len(history_x)
                avg_sz = sum(history_size) / len(history_size)

                # --- 1. ERROR CALCULATION ---
                curr_err_x = TARGET_CENTER_X - avg_x  
                
                # Size Error: 
                # Target (100) - Current (80) = +20 (Object too small/far) -> Move Closer (+Front)
                # Target (100) - Current (120) = -20 (Object too big/close) -> Move Back (-Front)
                curr_err_depth = TARGET_SIZE - avg_sz

                move_right_req = 0.0
                move_front_req = 0.0

                # --- 2. PD CONTROL ---
                
                # X-AXIS
                if abs(curr_err_x) > DEAD_ZONE_X:
                    deriv_x = curr_err_x - prev_error_x
                    output_x = (curr_err_x * KP_X) + (deriv_x * KD_X)
                    move_right_req = -1 * output_x * 0.1
                    if INVERT_X: move_right_req *= -1
                else:
                    prev_error_x = curr_err_x 

                # DEPTH-AXIS (SENSITIVE)
                if abs(curr_err_depth) > DEAD_ZONE_DEPTH:
                    deriv_depth = curr_err_depth - prev_error_depth
                    # We boost the output because pixels change slowly for depth
                    output_depth = (curr_err_depth * KP_DEPTH) + (deriv_depth * KD_DEPTH)
                    
                    move_front_req = output_depth * 0.1
                    if INVERT_Y: move_front_req *= -1
                else:
                    prev_error_depth = curr_err_depth

                prev_error_x = curr_err_x
                prev_error_depth = curr_err_depth

                # --- 3. MOVE ---
                did_move = send_simultaneous_move(move_right_req, move_front_req)
                
                if did_move:
                    status = f"X:{move_right_req:.3f} Y:{move_front_req:.3f}"

                # --- VISUALIZATION ---
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Draw Size Data (Critical for Debugging)
                label = f"Size: {avg_sz:.1f} / Tgt: {TARGET_SIZE:.1f}"
                color = (0, 255, 0) if abs(curr_err_depth) < DEAD_ZONE_DEPTH else (0, 0, 255)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            else:
                cv2.putText(frame, "LOST TRACKING", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                
            cv2.putText(frame, f"Pos: {cur_machine_x:.2f}, {cur_machine_y:.2f}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, status, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Sensitive Depth Tracker", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('r'): # Reset Target
                TARGET_SIZE = (w + h) / 2.0
                print(f"🔄 Target Reset to {TARGET_SIZE}")

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