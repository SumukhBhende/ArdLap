import cv2
import sys

# --- CONFIGURATION ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- CALIBRATION & CONTROL (SIMULATION) ---
# The "Dead Zone" in pixels. If the error is smaller than this, 
# we'll consider it "centered".
DEAD_ZONE = 10 

# This is the "magic number" that converts PIXEL error to a physical unit.
# You will need to calibrate this later.
# For now, it's just for simulated terminal output.
# Format: (cm_per_pixel)
PIXEL_TO_CM_RATIO_X = 0.05  # !! TUNE THIS LATER
PIXEL_TO_CM_RATIO_Y = 0.05  # !! TUNE THIS LATER

def main():
    # 1. Initialize Camera
    cap = cv2.VideoCapture(0) # 0 is default webcam. Try 1 for USB cam.
    if not cap.isOpened():
        print("Error: Cannot open camera.")
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print(f"Camera opened at {FRAME_WIDTH}x{FRAME_HEIGHT}")
    
    # Calculate the center of the camera frame
    frame_center_x = FRAME_WIDTH // 2
    frame_center_y = FRAME_HEIGHT // 2

    # 2. Initialize Tracker
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't read first frame.")
        cap.release()
        return

    # Let user select the object to track
    print("\n--- OBJECT SELECTION ---")
    print("A window will open. Click and drag a box around the object.")
    print("Press ENTER or SPACE to confirm.")
    print("Press C to cancel.")
    roi = cv2.selectROI("Select Object to Track", frame, fromCenter=False, showCrosshair=True)
    
    # Check if a valid ROI was selected
    if roi[2] == 0 or roi[3] == 0: # roi is (x, y, w, h)
        print("No object selected. Exiting.")
        cap.release()
        cv2.destroyAllWindows()
        return
        
    # Create and initialize the tracker
    # We use CSRT for good accuracy
    tracker = cv2.TrackerCSRT_create() 
    tracker.init(frame, roi)
    print("Object selected. Starting tracking...")

    cv2.destroyWindow("Select Object to Track")

    # 3. Main Tracking Loop
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Frame stream ended.")
            break

        # Update tracker
        ok, bbox = tracker.update(frame)

        # Clear terminal for clean output
        # print("\033[H\033[J", end="") # Use this for a cleaner terminal

        if ok:
            # Bounding box (x, y, w, h)
            (x, y, w, h) = [int(v) for v in bbox]
            
            # Calculate center of the bounding box
            obj_center_x = x + w // 2
            obj_center_y = y + h // 2
            
            # --- ERROR CALCULATION ---
            # X error: positive = object is RIGHT of center
            error_x = obj_center_x - frame_center_x
            
            # Y error: positive = object is BELOW center (since 0,0 is top-left)
            error_y = obj_center_y - frame_center_y

            # --- PRINT TERMINAL INSTRUCTIONS ---
            is_centered = True
            
            # X-Axis Instruction
            if abs(error_x) > DEAD_ZONE:
                is_centered = False
                move_x_cm = error_x * PIXEL_TO_CM_RATIO_X
                direction = "RIGHT" if error_x > 0 else "LEFT"
                print(f"Error X: {error_x: 4d}px  -> Go {direction} by {abs(move_x_cm):.2f} cm")
            
            # Y-Axis Instruction
            if abs(error_y) > DEAD_ZONE:
                is_centered = False
                move_y_cm = error_y * PIXEL_TO_CM_RATIO_Y 
                # Note: Positive Y error means object is *below* center, 
                # so machine must move "DOWN" (which might be +Y or -Y)
                direction = "DOWN" if error_y > 0 else "UP"
                print(f"Error Y: {error_y: 4d}px  -> Go {direction} by {abs(move_y_cm):.2f} cm")
            
            if is_centered:
                print("--- CENTERED ---")


            # --- VISUALIZATION ---
            # Draw bounding box (Green)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Draw object center (Red)
            cv2.circle(frame, (obj_center_x, obj_center_y), 5, (0, 0, 255), -1)
            # Draw line from frame center to object center (Yellow)
            cv2.line(frame, (frame_center_x, frame_center_y), (obj_center_x, obj_center_y), (0, 255, 255), 2)
            
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking Failed", (100, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            print("Tracking Failed")

        # Draw frame center (Blue)
        cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1)
        
        # Display the resulting frame
        cv2.imshow("Object Tracker (Vision Test)", frame)

        # Exit loop on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quitting...")
            break

    # 4. Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("Done.")

if __name__ == "__main__":
    main()