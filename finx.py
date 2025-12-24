import cv2
import serial
import time
import math

# ===================== CONFIG =====================

# Camera
CAMERA_INDEX = 0

# Control
DEAD_ZONE = 10
PIXEL_TO_MM = 0.05
MAX_STEP_MM = 0.5          # max step per cycle (smooth motion)
CONTROL_DELAY = 0.05

# CNC limits (ABSOLUTE, SAFE WORKSPACE)
X_MIN, X_MAX = -3.0, 3.0
Y_MIN, Y_MAX = -3.0, 3.0

# GRBL
SERIAL_PORT = "COM3"
BAUDRATE = 115200
FEEDRATE = 800

# Kinematic scaling (45-degree correction)
KIN_SCALE = 1 / math.sqrt(2)

# =================================================


# ---------- GRBL SERIAL ----------
ser = serial.Serial(SERIAL_PORT, BAUDRATE)
time.sleep(2)
ser.write(b"\r\n\r\n")
time.sleep(2)
ser.flushInput()

def send_gcode(cmd):
    """Send and PRINT gcode"""
    print(f"[GCODE] {cmd}")
    ser.write((cmd + "\n").encode())
    ser.flush()


# ---------- STATE ----------
# Track absolute CNC position
cur_x = 0.0
cur_y = 0.0


# ---------- HELPERS ----------
def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))


def move_camera(dx_mm, dy_mm):
    """
    Camera-space correction -> Absolute CNC motion (G90)
    Applies diagonal kinematic compensation and workspace limits
    """
    global cur_x, cur_y

    # Inverse kinematics (camera -> machine)
    mx = (dx_mm - dy_mm) * KIN_SCALE
    my = (dx_mm + dy_mm) * KIN_SCALE

    # Limit per-step motion
    mx = clamp(mx, -MAX_STEP_MM, MAX_STEP_MM)
    my = clamp(my, -MAX_STEP_MM, MAX_STEP_MM)

    # Update absolute position
    new_x = clamp(cur_x + mx, X_MIN, X_MAX)
    new_y = clamp(cur_y + my, Y_MIN, Y_MAX)

    # If no movement possible
    if abs(new_x - cur_x) < 1e-4 and abs(new_y - cur_y) < 1e-4:
        return None

    cur_x, cur_y = new_x, new_y

    return f"G90 G1 X{cur_x:.3f} Y{cur_y:.3f} F{FEEDRATE}"


# ---------- MAIN ----------
def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("❌ Camera not found")
        return

    ret, frame = cap.read()
    if not ret:
        print("❌ Frame read error")
        return

    print("\nSelect object to track and press ENTER")
    roi = cv2.selectROI("Select Object", frame, False, True)
    cv2.destroyWindow("Select Object")

    if roi[2] == 0 or roi[3] == 0:
        print("❌ No ROI selected")
        return

    # Tracker
    if hasattr(cv2, "TrackerCSRT_create"):
        tracker = cv2.TrackerCSRT_create()
    else:
        tracker = cv2.legacy.TrackerCSRT_create()

    tracker.init(frame, roi)
    print("✅ Tracking started (press 'q' to quit)")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_h, frame_w = frame.shape[:2]
        frame_cx = frame_w // 2
        frame_cy = frame_h // 2

        ok, bbox = tracker.update(frame)

        if ok:
            x, y, bw, bh = map(int, bbox)

            obj_cx = x + bw // 2
            obj_cy = y + bh // 2

            error_x = obj_cx - frame_cx
            error_y = obj_cy - frame_cy

            dx_mm = 0.0
            dy_mm = 0.0

            if abs(error_x) > DEAD_ZONE:
                dx_mm = error_x * PIXEL_TO_MM

            if abs(error_y) > DEAD_ZONE:
                dy_mm = error_y * PIXEL_TO_MM

            gcode = move_camera(dx_mm, dy_mm)
            if gcode:
                send_gcode(gcode)

            # ---- VISUALIZATION ----
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.circle(frame, (obj_cx, obj_cy), 5, (0, 0, 255), -1)
            cv2.line(
                frame,
                (frame_cx, frame_cy),
                (obj_cx, obj_cy),
                (0, 255, 255),
                2
            )

        # Frame center
        cv2.circle(frame, (frame_cx, frame_cy), 5, (255, 0, 0), -1)

        # Dead zone
        cv2.rectangle(
            frame,
            (frame_cx - DEAD_ZONE, frame_cy - DEAD_ZONE),
            (frame_cx + DEAD_ZONE, frame_cy + DEAD_ZONE),
            (255, 255, 0),
            1
        )

        cv2.imshow("Vision-Based CNC Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(CONTROL_DELAY)

    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("✅ Shutdown complete")


if __name__ == "__main__":
    main()
