import cv2
import serial
import time
import collections

# ===================== CONFIGURATION =====================

# --- HARDWARE ---
CAMERA_INDEX = 1
SERIAL_PORT = "COM3"
BAUDRATE = 115200

# --- WORKSPACE ---
WORKSPACE_SIZE = 6.0            # mm
HALF_RANGE = WORKSPACE_SIZE / 2

# --- JOG CONTROL ---
JOG_INTERVAL = 0.04             # 25 Hz
JOG_DISTANCE = 500.0            # Placeholder distance (mm)

# --- VELOCITY LIMITS (mm/min) ---
MAX_VEL = 600
MIN_VEL = 30

# --- DEAD ZONES (pixels) ---
DEAD_ZONE_X = 25
DEAD_ZONE_DEPTH = 6

# --- VELOCITY PD GAINS ---
KP_X, KD_X = 1.8, 1.2
KP_D, KD_D = 3.5, 2.5

INVERT_X = True
INVERT_Y = False

# --- SMOOTHING ---
SMOOTHING = 8

# =========================================================

cur_x = 0.0
cur_y = 0.0
prev_err_x = 0.0
prev_err_d = 0.0
last_jog_time = 0
frame_count = 0

hx = collections.deque(maxlen=SMOOTHING)
hs = collections.deque(maxlen=SMOOTHING)

# ===================== GRBL =====================

def init_grbl():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.5)
    time.sleep(2)

    ser.write(b"\r\n\r\n")
    time.sleep(1)
    ser.flushInput()

    print("🏠 Homing...")
    ser.write(b"$H\n")
    time.sleep(6)

    center = WORKSPACE_SIZE / 2
    ser.write(f"G90 G0 X{center} Y{center}\n".encode())
    time.sleep(3)

    ser.write(b"G92 X0 Y0\n")    # Define center as (0,0)
    ser.write(b"G21\n")          # mm
    ser.write(b"G91\n")          # incremental
    ser.write(b"$X\n")           # unlock

    print("✅ Homed & Centered")
    return ser

# ===================== HELPERS =====================

def clamp(v):
    if abs(v) < MIN_VEL:
        return 0.0
    return max(min(v, MAX_VEL), -MAX_VEL)

def update_position_from_grbl(ser):
    global cur_x, cur_y

    ser.write(b"?")
    line = ser.readline().decode(errors="ignore")

    if "WPos:" in line:
        try:
            pos = line.split("WPos:")[1].split("|")[0]
            x, y, *_ = map(float, pos.split(","))
            cur_x, cur_y = x, y
        except:
            pass

def send_jog(ser, vx, vy):
    # --- STOP ---
    if abs(vx) < 1e-3 and abs(vy) < 1e-3:
        ser.write(b"\x85")
        return

    dir_x = 1 if vx > 0 else -1
    dir_y = 1 if vy > 0 else -1

    # --- SOFTWARE FALLBACK LIMITS ---
    if (cur_x >= HALF_RANGE and dir_x > 0) or (cur_x <= -HALF_RANGE and dir_x < 0):
        vx = 0
    if (cur_y >= HALF_RANGE and dir_y > 0) or (cur_y <= -HALF_RANGE and dir_y < 0):
        vy = 0

    feed = max(abs(vx), abs(vy))

    if feed < MIN_VEL:
        ser.write(b"\x85")
        return

    cmd = (
        f"$J=G91 "
        f"X{JOG_DISTANCE * (vx / feed):.3f} "
        f"Y{JOG_DISTANCE * (vy / feed):.3f} "
        f"F{feed:.1f}\n"
    )
    ser.write(cmd.encode())

# ===================== MAIN =====================

def main():
    global prev_err_x, prev_err_d, last_jog_time, frame_count

    ser = init_grbl()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(3, 640)
    cap.set(4, 480)

    ret, frame = cap.read()
    if not ret:
        return

    roi = cv2.selectROI("Tracker", frame, False)
    cv2.destroyWindow("Tracker")

    tracker = cv2.TrackerCSRT_create()
    tracker.init(frame, roi)

    TARGET_SIZE = (roi[2] + roi[3]) / 2
    CX = frame.shape[1] // 2
    CY = frame.shape[0] // 2

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1

            if frame_count % 8 == 0:
                update_position_from_grbl(ser)

            ok, bbox = tracker.update(frame)
            vx = vy = 0.0

            # --- VISUAL WORKSPACE OVERLAY ---
            margin_x = int(frame.shape[1] * 0.15)
            margin_y = int(frame.shape[0] * 0.15)
            cv2.rectangle(
                frame,
                (margin_x, margin_y),
                (frame.shape[1] - margin_x, frame.shape[0] - margin_y),
                (255, 0, 0), 2
            )

            if ok:
                x, y, w, h = map(int, bbox)
                cx = x + w // 2
                size = (w + h) / 2

                hx.append(cx)
                hs.append(size)

                avg_x = sum(hx) / len(hx)
                avg_s = sum(hs) / len(hs)

                err_x = CX - avg_x
                err_d = TARGET_SIZE - avg_s

                if abs(err_x) > DEAD_ZONE_X:
                    vx = KP_X * err_x + KD_X * (err_x - prev_err_x)
                    if INVERT_X:
                        vx *= -1

                if abs(err_d) > DEAD_ZONE_DEPTH:
                    vy = KP_D * err_d + KD_D * (err_d - prev_err_d)
                    if INVERT_Y:
                        vy *= -1

                prev_err_x = err_x
                prev_err_d = err_d

                vx = clamp(vx)
                vy = clamp(vy)

                now = time.time()
                if now - last_jog_time > JOG_INTERVAL:
                    send_jog(ser, vx, vy)
                    last_jog_time = now

                cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)

            else:
                ser.write(b"\x85")

            cv2.putText(
                frame,
                f"POS X:{cur_x:.2f} Y:{cur_y:.2f}  VX:{vx:.0f} VY:{vy:.0f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,255),
                2
            )

            cv2.imshow("Velocity-Based Visual Servo (GRBL)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        ser.write(b"\x85")
        cap.release()
        cv2.destroyAllWindows()
        ser.close()

if __name__ == "__main__":
    main()
