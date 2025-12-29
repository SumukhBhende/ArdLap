import cv2
import serial
import time
import math
import logging
import sys
import collections
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass, field

# Import your helper classes
from gcode_translator import GCodeTranslator
from jog_controller import JogController

# Model inference imports
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    logging.warning("Ultralytics not available.")

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)

@dataclass
class Config:
    """System configuration parameters"""
    # Serial Communication
    SERIAL_PORT: str = 'COM3' 
    BAUD_RATE: int = 115200
    SERIAL_TIMEOUT: float = 0.01  # Reduced to prevent lag
    
    # Camera Settings
    CAM_INDEX: int = 1
    CAM_WIDTH: int = 640
    CAM_HEIGHT: int = 480
    CAM_FPS: int = 30
    
    # Model Configuration
    MODEL_PATH: str = './adifin/best.pt'  
    MODEL_CONFIDENCE: float = 0.5              
    MODEL_IOU: float = 0.45                    
    TARGET_CLASS_NAME: Optional[str] = "diver"    
    
    # Tracking Safety
    MIN_TRACKING_SIZE: int = 20
    SMOOTHING_BUFFER: int = 5
    STARTUP_SETTLE_FRAMES: int = 10
    NO_DETECTION_TIMEOUT: int = 30             
    
    # Control Parameters (Tuned for Jogging)
    KP_X: float = 0.006     # Increased for better response
    KD_X: float = 0.001
    DEADBAND_X: float = 40.0
    INVERT_X: bool = False
    
    KP_DEPTH: float = 0.012
    KD_DEPTH: float = 0.002
    DEADBAND_DEPTH: float = 12.0
    INVERT_Y: bool = True
    
    # Motion Safety - RELAXED TO AVOID CHOKING JOGGER
    MAX_STEP_SIZE: float = 5.0    # Increased from 0.3 to allow feedrate to climb
    MAX_VELOCITY: float = 10.0    # Increased to prevent clamping PID output
    MAX_COMMAND_RATE: float = 30.0
    
    RECONNECT_ATTEMPTS: int = 3
    RECONNECT_DELAY: float = 2.0

class SerialHandler:
    """Robust serial communication for GRBL"""
    def __init__(self, config: Config):
        self.config = config
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.last_command_time = time.time()
        
    def connect(self) -> bool:
        for attempt in range(self.config.RECONNECT_ATTEMPTS):
            try:
                logging.info(f"Connecting to {self.config.SERIAL_PORT}...")
                self.serial_port = serial.Serial(
                    port=self.config.SERIAL_PORT,
                    baudrate=self.config.BAUD_RATE,
                    timeout=self.config.SERIAL_TIMEOUT,
                    write_timeout=0.1 # Low timeout to prevent blocking
                )
                time.sleep(2)
                
                if self.serial_port.is_open:
                    self.is_connected = True
                    # INITIALIZE GRBL
                    self.serial_port.write(b"\r\n\r\n") # Wake up
                    time.sleep(1)
                    self.serial_port.write(b"$X\n")     # UNLOCK ALARM
                    self.serial_port.write(b"G21\n")    # Metric units
                    self.serial_port.write(b"G91\n")    # Relative mode
                    self.serial_port.flushInput()
                    logging.info("GRBL Unlocked and Ready.")
                    return True
            except Exception as e:
                logging.error(f"Serial Error: {e}")
                time.sleep(self.config.RECONNECT_DELAY)
        return False
    
    def send_jog(self, jog_cmd: str):
        """Sends the raw Jog command"""
        if self.is_connected and self.serial_port:
            try:
                self.serial_port.write((jog_cmd + '\n').encode())
                return True
            except Exception:
                self.is_connected = False
        return False

    def jog_cancel(self):
        """Immediate Jog Stop"""
        if self.is_connected and self.serial_port:
            self.serial_port.write(b'\x85') 

    def close(self):
        if self.serial_port: self.serial_port.close()

class CameraHandler:
    """Manages OpenCV VideoCapture"""
    def __init__(self, config: Config):
        self.config = config
        self.cap = None
        self.frame_center_x = 0
        self.frame_center_y = 0
        
    def initialize(self) -> bool:
        self.cap = cv2.VideoCapture(self.config.CAM_INDEX)
        if not self.cap.isOpened(): return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAM_HEIGHT)
        self.frame_center_x = int(self.cap.get(3)) // 2
        self.frame_center_y = int(self.cap.get(4)) // 2
        return True
    
    def read_frame(self):
        return self.cap.read()

    def release(self):
        if self.cap: self.cap.release()

class ModelDetector:
    """Object detection wrapper"""
    def __init__(self, config: Config):
        self.config = config
        self.model = None
        self.is_loaded = False
        
    def load_model(self) -> bool:
        try:
            if YOLO_AVAILABLE:
                self.model = YOLO(self.config.MODEL_PATH)
                self.is_loaded = True
                return True
        except Exception as e:
            logging.error(f"Model Load Fail: {e}")
        return False
    
    def detect(self, frame):
        if not self.is_loaded: return []
        results = self.model(frame, conf=self.config.MODEL_CONFIDENCE, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                detections.append((int(x1), int(y1), int(x2-x1), int(y2-y1), float(box.conf[0]), int(box.cls[0])))
        return detections

    def get_class_name(self, class_id):
        return self.model.names[class_id] if self.model else str(class_id)

class ObjectTracker:
    """PD Control and Object State"""
    def __init__(self, config: Config, cx, cy, detector: ModelDetector):
        self.config, self.detector = config, detector
        self.target_center_x, self.target_center_y = cx, cy
        self.target_size = 0.0
        self.is_initialized = False
        self.frames_without_detection = 0
        self.prev_error_x = 0.0
        self.prev_error_depth = 0.0
        self.history_x = collections.deque(maxlen=config.SMOOTHING_BUFFER)
        self.history_size = collections.deque(maxlen=config.SMOOTHING_BUFFER)

    def initialize_target(self, frame):
        # Simplified: Auto-selects first valid detection
        dets = self.detector.detect(frame)
        if dets:
            x, y, w, h, conf, cls_id = dets[0]
            self.target_size = (w + h) / 2.0
            self.is_initialized = True
            logging.info(f"Target locked: {self.detector.get_class_name(cls_id)}")
            return True
        return False

    def update_and_calculate_move(self, frame):
        if not self.is_initialized: return "NOT INIT", (0.0, 0.0), None
        
        dets = self.detector.detect(frame)
        if not dets:
            self.frames_without_detection += 1
            if self.frames_without_detection > self.config.NO_DETECTION_TIMEOUT:
                return "STOP - LOST", (0.0, 0.0), None
            return "SEARCHING", (0.0, 0.0), None

        # Selection logic (simplified to first detection for stability)
        x, y, w, h, conf, cls_id = dets[0]
        self.frames_without_detection = 0
        cx, cy = x + w//2, y + h//2
        curr_sz = (w + h) / 2.0
        
        self.history_x.append(cx)
        self.history_size.append(curr_sz)
        
        avg_x = sum(self.history_x) / len(self.history_x)
        avg_sz = sum(self.history_size) / len(self.history_size)

        # PD Loop
        err_x = self.target_center_x - avg_x
        move_x = 0.0
        if abs(err_x) > self.config.DEADBAND_X:
            move_x = (err_x * self.config.KP_X) + ((err_x - self.prev_error_x) * self.config.KD_X)
        self.prev_error_x = err_x

        err_sz = self.target_size - avg_sz
        move_y = 0.0
        if abs(err_sz) > self.config.DEADBAND_DEPTH:
            move_y = (err_sz * self.config.KP_DEPTH) + ((err_sz - self.prev_error_depth) * self.config.KD_DEPTH)
        self.prev_error_depth = err_sz

        # Apply Inversions
        if self.config.INVERT_X: move_x *= -1
        if self.config.INVERT_Y: move_y *= -1

        # Clamping with higher limits
        move_x = np.clip(move_x, -self.config.MAX_STEP_SIZE, self.config.MAX_STEP_SIZE)
        move_y = np.clip(move_y, -self.config.MAX_STEP_SIZE, self.config.MAX_STEP_SIZE)

        return "TRACKING", (move_x, move_y), (x, y, w, h)

class TrackingSystem:
    """CoreXY Main Controller"""
    def __init__(self, config: Config):
        self.config = config
        self.serial = SerialHandler(config)
        self.camera = CameraHandler(config)
        self.detector = ModelDetector(config)
        self.motor = GCodeTranslator() # For position tracking
        self.jog = JogController(max_feed=1500, min_feed=250)
        self.tracker = None

    def run(self):
        if not self.detector.load_model() or not self.camera.initialize(): return
        self.serial.connect()
        self.tracker = ObjectTracker(self.config, self.camera.frame_center_x, self.camera.frame_center_y, self.detector)

        logging.info("System Ready. Press 's' to start tracking.")
        
        while True:
            ret, frame = self.camera.read_frame()
            if not ret: break

            status, moves, bbox = self.tracker.update_and_calculate_move(frame)
            vx, vy = moves

            if bbox and status == "TRACKING":
                x, y, w, h = bbox
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # --- APPLY COREXY KINEMATICS TO JOG ---
                # MotorA = X - Y, MotorB = X + Y
                ma_dist = vx - vy
                mb_dist = vx + vy
                
                speed = math.sqrt(vx**2 + vy**2)
                feed = int(np.clip(speed * 1500, self.jog.min_feed, self.jog.max_feed))
                
                # Send the Jog Command
                jog_cmd = f"$J=G91 X{ma_dist*1000:.1f} Y{mb_dist*1000:.1f} F{feed}"
                self.serial.send_jog(jog_cmd)
                self.jog.is_jogging = True
            else:
                if self.jog.is_jogging:
                    self.serial.jog_cancel()
                    self.jog.is_jogging = False

            cv2.imshow("Diver Tracker", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('s'): self.tracker.initialize_target(frame)

        self.serial.close()
        self.camera.release()

if __name__ == "__main__":
    TrackingSystem(Config()).run()