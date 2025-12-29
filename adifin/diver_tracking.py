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
    # Serial Communication - Ensure this matches your Device Manager
    SERIAL_PORT: str = 'COM3' 
    BAUD_RATE: int = 115200
    SERIAL_TIMEOUT: float = 0.01 
    
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
    
    # Control Parameters (PID Tuning)
    KP_X: float = 0.008     # Gain for lateral movement
    KD_X: float = 0.001
    DEADBAND_X: float = 40.0
    INVERT_X: bool = False
    
    KP_DEPTH: float = 0.015  # Gain for Z/Size movement
    KD_DEPTH: float = 0.002
    DEADBAND_DEPTH: float = 12.0
    INVERT_Y: bool = True
    
    # Jogging Scale
    FEED_SCALE: float = 2000.0  # Multiplier for PID output to mm/min
    MAX_STEP_SIZE: float = 10.0 # Allowed "Delta" per frame
    MAX_COMMAND_RATE: float = 30.0

class SerialHandler:
    """Robust serial communication for GRBL"""
    def __init__(self, config: Config):
        self.config = config
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        
    def connect(self) -> bool:
        try:
            logging.info(f"Connecting to {self.config.SERIAL_PORT}...")
            self.serial_port = serial.Serial(
                port=self.config.SERIAL_PORT,
                baudrate=self.config.BAUD_RATE,
                timeout=self.config.SERIAL_TIMEOUT,
                write_timeout=0.1
            )
            time.sleep(2) # Wait for GRBL boot
            
            if self.serial_port.is_open:
                self.is_connected = True
                # CRITICAL: INITIALIZE GRBL AND DISABLE LIMITS
                init_sequence = [
                    b"\r\n\r\n",   # Wake up
                    b"$X\n",       # UNLOCK ALARM (Immediate fix for non-movement)
                    b"$21=0\n",    # Disable Hard Limits
                    b"$20=0\n",    # Disable Soft Limits
                    b"G21\n",      # Metric
                    b"G91\n"       # Relative
                ]
                for cmd in init_sequence:
                    self.serial_port.write(cmd)
                    time.sleep(0.1)
                
                logging.info("GRBL Unlocked and Limits Disabled.")
                return True
        except Exception as e:
            logging.error(f"Serial Connection Failed: {e}")
        return False
    
    def send_jog(self, jog_cmd: str):
        if self.is_connected and self.serial_port:
            try:
                self.serial_port.write((jog_cmd + '\n').encode())
                return True
            except Exception:
                self.is_connected = False
        return False

    def jog_cancel(self):
        if self.is_connected and self.serial_port:
            self.serial_port.write(b'\x85') 

    def close(self):
        if self.serial_port: self.serial_port.close()

class CameraHandler:
    def __init__(self, config: Config):
        self.config = config
        self.cap = None
        
    def initialize(self) -> bool:
        self.cap = cv2.VideoCapture(self.config.CAM_INDEX)
        if not self.cap.isOpened(): return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAM_HEIGHT)
        return True
    
    def read_frame(self):
        return self.cap.read()

    def release(self):
        if self.cap: self.cap.release()

class ModelDetector:
    def __init__(self, config: Config):
        self.config = config
        self.model = None
        self.is_loaded = False
        
    def load_model(self) -> bool:
        if YOLO_AVAILABLE:
            self.model = YOLO(self.config.MODEL_PATH)
            self.is_loaded = True
            return True
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

class ObjectTracker:
    def __init__(self, config: Config, cx, cy, detector: ModelDetector):
        self.config, self.detector = config, detector
        self.center_x, self.center_y = cx, cy
        self.target_size = 0.0
        self.is_initialized = False
        self.prev_error_x = 0.0
        self.prev_error_depth = 0.0
        self.history_x = collections.deque(maxlen=config.SMOOTHING_BUFFER)
        self.history_size = collections.deque(maxlen=config.SMOOTHING_BUFFER)

    def initialize_target(self, frame):
        dets = self.detector.detect(frame)
        if dets:
            x, y, w, h, conf, cls_id = dets[0]
            self.target_size = (w + h) / 2.0
            self.is_initialized = True
            logging.info("Target Locked.")
            return True
        return False

    def update_and_calculate_move(self, frame):
        if not self.is_initialized: return "NOT INIT", (0.0, 0.0), None
        
        dets = self.detector.detect(frame)
        if not dets: return "SEARCHING", (0.0, 0.0), None

        x, y, w, h, conf, cls_id = dets[0]
        cx = x + w//2
        curr_sz = (w + h) / 2.0
        
        self.history_x.append(cx)
        self.history_size.append(curr_sz)
        
        avg_x = sum(self.history_x) / len(self.history_x)
        avg_sz = sum(self.history_size) / len(self.history_size)

        # PD Control Logic
        err_x = self.center_x - avg_x
        move_x = 0.0
        if abs(err_x) > self.config.DEADBAND_X:
            move_x = (err_x * self.config.KP_X) + ((err_x - self.prev_error_x) * self.config.KD_X)
        self.prev_error_x = err_x

        err_sz = self.target_size - avg_sz
        move_y = 0.0
        if abs(err_sz) > self.config.DEADBAND_DEPTH:
            move_y = (err_sz * self.config.KP_DEPTH) + ((err_sz - self.prev_error_depth) * self.config.KD_DEPTH)
        self.prev_error_depth = err_sz

        if self.config.INVERT_X: move_x *= -1
        if self.config.INVERT_Y: move_y *= -1

        move_x = np.clip(move_x, -self.config.MAX_STEP_SIZE, self.config.MAX_STEP_SIZE)
        move_y = np.clip(move_y, -self.config.MAX_STEP_SIZE, self.config.MAX_STEP_SIZE)

        return "TRACKING", (move_x, move_y), (x, y, w, h)

class TrackingSystem:
    def __init__(self, config: Config):
        self.config = config
        self.serial = SerialHandler(config)
        self.camera = CameraHandler(config)
        self.detector = ModelDetector(config)
        self.is_moving = False

    def run(self):
        if not self.detector.load_model() or not self.camera.initialize(): return
        self.serial.connect()
        
        cx = self.camera.cap.get(3) // 2
        cy = self.camera.cap.get(4) // 2
        self.tracker = ObjectTracker(self.config, cx, cy, self.detector)

        while True:
            ret, frame = self.camera.read_frame()
            if not ret: break

            status, (vx, vy), bbox = self.tracker.update_and_calculate_move(frame)

            if bbox and status == "TRACKING":
                x, y, w, h = bbox
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # --- COREXY TRANSFORM ---
                # MotorA = X - Y, MotorB = X + Y
                ma = vx - vy
                mb = vx + vy
                
                speed = math.sqrt(vx**2 + vy**2)
                feed = int(np.clip(speed * self.config.FEED_SCALE, 200, 2000))
                
                # Multiply ma/mb by a high distance (1000) to keep the motor "pushed"
                jog_cmd = f"$J=G91 X{ma*1000:.2f} Y{mb*1000:.2f} F{feed}"
                self.serial.send_jog(jog_cmd)
                self.is_moving = True
            else:
                if self.is_moving:
                    self.serial.jog_cancel()
                    self.is_moving = False

            cv2.imshow("Diver Tracker", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('s'): self.tracker.initialize_target(frame)

        self.serial.close()
        self.camera.release()

if __name__ == "__main__":
    TrackingSystem(Config()).run()