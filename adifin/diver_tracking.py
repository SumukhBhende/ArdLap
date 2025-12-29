"""
Robust Real-Time Diver Tracking System - FIXED VERSION
Key fixes:
- Proper derivative initialization
- Tuned control gains
- Movement velocity limiting
- Startup settling period
"""

"""
Robust Real-Time Object Tracking System - CUSTOM MODEL VERSION
Uses trained detection model instead of OpenCV tracker
"""

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
from adifin.gcode_translator import GCodeTranslator
from adifin.jog_controller import JogController


# Model inference imports
try:
    from ultralytics import YOLO  # For YOLOv8/YOLOv11
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    logging.warning("Ultralytics not available. Install with: pip install ultralytics")

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    logging.warning("PyTorch not available. Install with: pip install torch")

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('tracking.log'),
        logging.StreamHandler(sys.stdout)
    ]
)

# ==================== CONFIGURATION ====================
@dataclass
class Config:
    """System configuration parameters"""
    # Serial Communication
    SERIAL_PORT: str = 'COM3' # Update as needed
    BAUD_RATE: int = 115200
    SERIAL_TIMEOUT: float = 0.1
    
    # Camera Settings
    CAM_INDEX: int = 1
    CAM_WIDTH: int = 640
    CAM_HEIGHT: int = 480
    CAM_FPS: int = 30
    
    # Model Configuration
    MODEL_PATH: str = './adifin/best.pt'  
    MODEL_CONFIDENCE: float = 0.5              # Detection confidence threshold
    MODEL_IOU: float = 0.45                    # NMS IOU threshold
    TARGET_CLASS: Optional[int] = None         # None = any class, or specific class ID
    TARGET_CLASS_NAME: Optional[str] = "diver"    # Or specify by class name
    
    # Tracking Safety
    MIN_TRACKING_SIZE: int = 20
    SMOOTHING_BUFFER: int = 5
    STARTUP_SETTLE_FRAMES: int = 10
    NO_DETECTION_TIMEOUT: int = 30             # Frames to wait before stopping
    
    # Control Parameters
    # Lateral (X-Axis)
    KP_X: float = 0.004
    KD_X: float = 0.001
    DEADBAND_X: float = 50.0
    INVERT_X: bool = False
    
    # Depth (Y-Axis / Size)
    KP_DEPTH: float = 0.010
    KD_DEPTH: float = 0.002
    DEADBAND_DEPTH: float = 12.0
    INVERT_Y: bool = True
    
    # Motion Safety
    MAX_STEP_SIZE: float = 0.3
    MAX_VELOCITY: float = 1.5
    MAX_COMMAND_RATE: float = 25.0
    
    # Connection
    RECONNECT_ATTEMPTS: int = 3
    RECONNECT_DELAY: float = 2.0

# ==================== SERIAL HANDLER ====================
class SerialHandler:
    """Robust serial communication with automatic reconnection"""
    
    def __init__(self, config: Config):
        self.config = config
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.command_count = 0
        self.last_command_time = time.time()
        
    def connect(self) -> bool:
        """Establish serial connection with retry logic"""
        for attempt in range(self.config.RECONNECT_ATTEMPTS):
            try:
                logging.info(f"Attempting serial connection to {self.config.SERIAL_PORT}...")
                
                self.serial_port = serial.Serial(
                    port=self.config.SERIAL_PORT,
                    baudrate=self.config.BAUD_RATE,
                    timeout=self.config.SERIAL_TIMEOUT,
                    write_timeout=1.0
                )
                
                time.sleep(2)  # Hardware wake-up
                
                if self.serial_port.is_open:
                    self.is_connected = True
                    # Initialize GRBL
                    self.serial_port.write(b"\r\n\r\n")
                    time.sleep(1)
                    self.serial_port.flushInput()
                    self.serial_port.write(b"G92 X0 Y0\n")
                    self.serial_port.write(b"G21 G91\n")
                    self.serial_port.write(b"G1 F800\n")  # Reduced speed for stability
                    
                    logging.info(f"Serial connection established on {self.config.SERIAL_PORT}")
                    return True
                    
            except serial.SerialException as e:
                logging.error(f"Serial connection failed: {e}")
                time.sleep(self.config.RECONNECT_DELAY)
            except Exception as e:
                logging.error(f"Unexpected error: {e}")
                time.sleep(self.config.RECONNECT_DELAY)
        
        return False
    
    def send_command(self, gcode: str) -> bool:
        """Send G-code command with rate limiting"""
        if not self.is_connected or not self.serial_port:
            return False
        
        # Rate limiting
        current_time = time.time()
        time_since_last = current_time - self.last_command_time
        min_interval = 1.0 / self.config.MAX_COMMAND_RATE
        
        if time_since_last < min_interval:
            time.sleep(min_interval - time_since_last)
        
        try:
            command = (gcode + '\n').encode('utf-8')
            self.serial_port.write(command)
            self.command_count += 1
            self.last_command_time = time.time()
            return True
            
        except Exception as e:
            logging.error(f"Serial send error: {e}")
            self.is_connected = False
            return False
    
    def send_jog(self, jog_cmd: str):
        """
        Send jog command WITHOUT rate limiting
        """
        if not self.is_connected or not self.serial_port:
            return False

        try:
            self.serial_port.write((jog_cmd + '\n').encode())
            return True
        except Exception as e:
            logging.error(f"Jog send error: {e}")
            self.is_connected = False
            return False


    def jog_cancel(self):
        if self.is_connected and self.serial_port:
            self.serial_port.write(b'\x85')  # GRBL Jog Cancel

    def close(self):
        """Close serial connection safely"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"G0 X0 Y0\n")
                time.sleep(0.1)
                self.serial_port.close()
            except:
                pass
        self.is_connected = False


# ==================== CAMERA HANDLER ====================
class CameraHandler:
    """Robust camera management"""
    
    def __init__(self, config: Config):
        self.config = config
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_opened = False
        self.frame_center_x = 0
        self.frame_center_y = 0
        
    def initialize(self) -> bool:
        """Initialize camera"""
        try:
            self.cap = cv2.VideoCapture(self.config.CAM_INDEX)
            if not self.cap.isOpened():
                logging.error(f"Cannot open camera {self.config.CAM_INDEX}")
                return False
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAM_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAM_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, self.config.CAM_FPS)
            
            # Update actual dimensions
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.frame_center_x = w // 2
            self.frame_center_y = h // 2
            
            self.is_opened = True
            logging.info(f"Camera initialized: {w}x{h}")
            return True
        except Exception as e:
            logging.error(f"Camera init error: {e}")
            return False
    
    def read_frame(self) -> Tuple[bool, Optional[any]]:
        if not self.is_opened or not self.cap:
            return False, None
        return self.cap.read()
    
    def release(self):
        if self.cap:
            self.cap.release()
        self.is_opened = False

# ==================== MODEL DETECTOR ====================
class ModelDetector:
    """
    Custom model-based object detector
    Supports YOLO, PyTorch, TensorFlow, ONNX models
    """
    
    def __init__(self, config: Config):
        self.config = config
        self.model = None
        self.model_type = None
        self.is_loaded = False
        self.class_names = []
        
    def load_model(self) -> bool:
        """Load detection model"""
        try:
            model_path = self.config.MODEL_PATH
            
            # Detect model type from extension
            if model_path.endswith('.pt') and YOLO_AVAILABLE:
                # YOLO format (Ultralytics)
                logging.info(f"Loading YOLO model from {model_path}")
                self.model = YOLO(model_path)
                self.model_type = 'yolo'
                self.class_names = self.model.names
                logging.info(f"YOLO model loaded. Classes: {self.class_names}")
                
            elif model_path.endswith('.pt') and TORCH_AVAILABLE:
                # PyTorch model
                logging.info(f"Loading PyTorch model from {model_path}")
                self.model = torch.load(model_path)
                self.model.eval()
                self.model_type = 'pytorch'
                logging.info("PyTorch model loaded")
                
            # elif model_path.endswith('.onnx'):
            #     # ONNX model
            #     import onnxruntime as ort
            #     logging.info(f"Loading ONNX model from {model_path}")
            #     self.model = ort.InferenceSession(model_path)
            #     self.model_type = 'onnx'
            #     logging.info("ONNX model loaded")
                
            else:
                logging.error(f"Unsupported model format: {model_path}")
                return False
            
            self.is_loaded = True
            return True
            
        except Exception as e:
            logging.error(f"Failed to load model: {e}")
            return False
    
    def detect(self, frame) -> List[Tuple[int, int, int, int, float, int]]:
        """
        Run detection on frame
        
        Returns:
            List of detections: [(x, y, w, h, confidence, class_id), ...]
        """
        if not self.is_loaded or self.model is None:
            return []
        
        try:
            if self.model_type == 'yolo':
                return self._detect_yolo(frame)
            elif self.model_type == 'pytorch':
                return self._detect_pytorch(frame)
            elif self.model_type == 'onnx':
                return self._detect_onnx(frame)
            else:
                return []
                
        except Exception as e:
            logging.error(f"Detection error: {e}")
            return []
    
    def _detect_yolo(self, frame) -> List[Tuple]:
        """YOLO detection"""
        results = self.model(
            frame, 
            conf=self.config.MODEL_CONFIDENCE,
            iou=self.config.MODEL_IOU,
            verbose=False
        )
        
        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get box coordinates (xyxy format)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Convert to xywh
                x = int(x1)
                y = int(y1)
                w = int(x2 - x1)
                h = int(y2 - y1)
                
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                
                detections.append((x, y, w, h, confidence, class_id))
        
        return detections
    
    def _detect_pytorch(self, frame) -> List[Tuple]:
        """
        PyTorch model detection
        Implement based on your specific model architecture
        """
        # Example implementation - modify based on your model
        # This is a placeholder - you'll need to adapt to your model's input/output format
        
        # Preprocess
        input_tensor = torch.from_numpy(frame).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        
        # Inference
        with torch.no_grad():
            outputs = self.model(input_tensor)
        
        # Postprocess (example - modify for your model)
        detections = []
        # Parse outputs and convert to [(x, y, w, h, conf, class_id), ...]
        
        return detections
    
    def _detect_onnx(self, frame) -> List[Tuple]:
        """
        ONNX model detection
        Implement based on your specific model
        """
        # Example implementation - modify based on your model
        input_name = self.model.get_inputs()[0].name
        
        # Preprocess
        input_tensor = cv2.resize(frame, (640, 640))
        input_tensor = input_tensor.transpose(2, 0, 1)
        input_tensor = np.expand_dims(input_tensor, 0).astype(np.float32) / 255.0
        
        # Inference
        outputs = self.model.run(None, {input_name: input_tensor})
        
        # Postprocess (example - modify for your model)
        detections = []
        # Parse outputs and convert to [(x, y, w, h, conf, class_id), ...]
        
        return detections
    
    def get_class_name(self, class_id: int) -> str:
        """Get class name from ID"""
        if class_id < len(self.class_names):
            return self.class_names[class_id]
        return f"class_{class_id}"
    

# ==================== SMART TRACKING LOGIC (MODEL-BASED) ====================
class ObjectTracker:
    """
    Model-based tracker with PD Control
    Uses custom detection model instead of OpenCV tracker
    """
    
    def __init__(self, config: Config, frame_center_x: int, frame_center_y: int, detector: ModelDetector):
        self.config = config
        self.detector = detector
        
        # Targets
        self.target_center_x = frame_center_x
        self.target_center_y = frame_center_y
        self.target_size = 0.0
        
        # State
        self.is_initialized = False
        self.tracking_quality = 0.0
        self.last_status = "IDLE"
        self.frame_count = 0
        self.frames_without_detection = 0
        
        # Target selection
        self.target_class_id = None
        self.target_class_name = None
        self.last_detection = None
        
        # PD Control History
        self.prev_error_x = 0.0
        self.prev_error_depth = 0.0
        self.error_initialized = False
        
        # Velocity tracking
        self.last_move_x = 0.0
        self.last_move_y = 0.0
        self.last_move_time = time.time()
        
        # Smoothing Buffers
        self.history_x = collections.deque(maxlen=config.SMOOTHING_BUFFER)
        self.history_y = collections.deque(maxlen=config.SMOOTHING_BUFFER)
        self.history_size = collections.deque(maxlen=config.SMOOTHING_BUFFER)
        
    def initialize_target(self, frame) -> bool:
        """
        Initialize tracking by detecting objects and letting user select
        """
        try:
            logging.info("Running detection to find objects...")
            
            # Run detection
            detections = self.detector.detect(frame)
            
            if not detections:
                logging.error("No objects detected in frame")
                return False
            
            # Draw all detections and let user choose
            display_frame = frame.copy()
            for idx, (x, y, w, h, conf, cls_id) in enumerate(detections):
                cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                class_name = self.detector.get_class_name(cls_id)
                label = f"{idx}: {class_name} {conf:.2f}"
                cv2.putText(display_frame, label, (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow("Select Target (press number key)", display_frame)
            
            # Wait for user to select target by pressing number key
            logging.info(f"Found {len(detections)} objects. Press number key (0-9) to select target...")
            
            while True:
                key = cv2.waitKey(0) & 0xFF
                
                # Check if number key pressed
                if ord('0') <= key <= ord('9'):
                    selected_idx = key - ord('0')
                    if selected_idx < len(detections):
                        x, y, w, h, conf, cls_id = detections[selected_idx]
                        self.target_class_id = cls_id
                        self.target_class_name = self.detector.get_class_name(cls_id)
                        
                        # Set target size
                        self.target_size = (w + h) / 2.0
                        
                        # Initialize error to prevent derivative kick
                        cx = x + (w // 2)
                        self.prev_error_x = self.target_center_x - cx
                        self.prev_error_depth = 0.0
                        self.error_initialized = True
                        
                        # Clear history
                        self.history_x.clear()
                        self.history_y.clear()
                        self.history_size.clear()
                        self.frame_count = 0
                        self.frames_without_detection = 0
                        
                        self.is_initialized = True
                        logging.info(f"Tracking initialized for: {self.target_class_name} (ID: {cls_id})")
                        logging.info(f"Target size: {self.target_size:.1f}")
                        
                        cv2.destroyWindow("Select Target (press number key)")
                        return True
                elif key == ord('q'):
                    break
            
        except Exception as e:
            logging.error(f"Target initialization failed: {e}")
            return False
    
    def _filter_detections(self, detections: List[Tuple]) -> List[Tuple]:
        """
        Filter detections to match target class and minimum size
        """
        filtered = []
        
        for det in detections:
            x, y, w, h, conf, cls_id = det
            
            # Check class match by name (UPDATED)
            if self.config.TARGET_CLASS_NAME is not None:
                detected_class_name = self.detector.get_class_name(cls_id)
                if detected_class_name.lower() != self.config.TARGET_CLASS_NAME.lower():
                    continue
            # Or check by class ID
            elif self.config.TARGET_CLASS is not None:
                if cls_id != self.config.TARGET_CLASS:
                    continue
            elif self.target_class_id is not None:
                # If tracking already initialized, stick to that class
                if cls_id != self.target_class_id:
                    continue
            
            # Check minimum size
            size = (w + h) / 2.0
            if size < self.config.MIN_TRACKING_SIZE:
                continue
            
            filtered.append(det)
        
        return filtered
    
    def _select_best_detection(self, detections: List[Tuple]) -> Optional[Tuple]:
        """
        Select best detection based on:
        1. Proximity to last known position (if available)
        2. Size similarity to target
        3. Confidence score
        """
        if not detections:
            return None
        
        if len(detections) == 1:
            return detections[0]
        
        # Score each detection
        scores = []
        for det in detections:
            x, y, w, h, conf, cls_id = det
            cx = x + (w // 2)
            cy = y + (h // 2)
            size = (w + h) / 2.0
            
            score = conf  # Start with confidence
            
            # Bonus for proximity to last detection
            if self.last_detection:
                lx, ly, lw, lh, _, _ = self.last_detection
                lcx = lx + (lw // 2)
                lcy = ly + (lh // 2)
                
                distance = math.sqrt((cx - lcx)**2 + (cy - lcy)**2)
                proximity_score = 1.0 / (1.0 + distance / 100.0)  # Normalize
                score += proximity_score
            
            # Bonus for size similarity to target
            if self.target_size > 0:
                size_ratio = min(size, self.target_size) / max(size, self.target_size)
                score += size_ratio
            
            scores.append(score)
        
        # Return detection with highest score
        best_idx = scores.index(max(scores))
        return detections[best_idx]

    def update_and_calculate_move(self, frame) -> Tuple[str, Tuple[float, float], Optional[Tuple]]:
        """
        Run detection, track object, calculate control output
        """
        if not self.is_initialized:
            return "NOT INIT", (0.0, 0.0), None

        # Run detection
        all_detections = self.detector.detect(frame)
        
        # Filter detections
        filtered_detections = self._filter_detections(all_detections)
        
        # Select best detection
        best_detection = self._select_best_detection(filtered_detections)
        
        if best_detection is None:
            self.frames_without_detection += 1
            self.tracking_quality = 0.0
            
            if self.frames_without_detection > self.config.NO_DETECTION_TIMEOUT:
                return "STOP - NO DETECTION", (0.0, 0.0), None
            else:
                return f"SEARCHING ({self.frames_without_detection}/{self.config.NO_DETECTION_TIMEOUT})", (0.0, 0.0), None
        
        # Reset no-detection counter
        self.frames_without_detection = 0
        self.frame_count += 1
        
        # Parse detection
        x, y, w, h, conf, cls_id = best_detection
        self.last_detection = best_detection
        
        current_size = (w + h) / 2.0
        cx = x + (w // 2)
        cy = y + (h // 2)
        
        self.tracking_quality = conf
        
        # Smoothing
        self.history_x.append(cx)
        self.history_y.append(cy)
        self.history_size.append(current_size)
        
        # Wait for buffer to fill
        if len(self.history_x) < 3:
            return "SETTLING", (0.0, 0.0), (x, y, w, h)
        
        avg_x = sum(self.history_x) / len(self.history_x)
        avg_y = sum(self.history_y) / len(self.history_y)
        avg_sz = sum(self.history_size) / len(self.history_size)
        
        # --- PD CONTROL LOOP ---
        
        # 1. Lateral Control (X-Axis)
        curr_err_x = self.target_center_x - avg_x
        move_x = 0.0
        
        if abs(curr_err_x) > self.config.DEADBAND_X:
            deriv_x = curr_err_x - self.prev_error_x
            output_x = (curr_err_x * self.config.KP_X) + (deriv_x * self.config.KD_X)
            
            move_x = output_x
            if self.config.INVERT_X:
                move_x *= -1
        
        self.prev_error_x = curr_err_x

        # 2. Depth Control (Y-Axis / Size)
        curr_err_depth = self.target_size - avg_sz
        move_y = 0.0
        
        if abs(curr_err_depth) > self.config.DEADBAND_DEPTH:
            deriv_depth = curr_err_depth - self.prev_error_depth
            output_depth = (curr_err_depth * self.config.KP_DEPTH) + (deriv_depth * self.config.KD_DEPTH)
            
            move_y = output_depth
            if self.config.INVERT_Y:
                move_y *= -1

        self.prev_error_depth = curr_err_depth

        # --- VELOCITY LIMITING ---
        current_time = time.time()
        dt = current_time - self.last_move_time
        
        if dt > 0:
            vel_x = abs(move_x - self.last_move_x) / dt
            vel_y = abs(move_y - self.last_move_y) / dt
            
            if vel_x > self.config.MAX_VELOCITY:
                move_x = self.last_move_x + (self.config.MAX_VELOCITY * dt * (1 if move_x > self.last_move_x else -1))
            if vel_y > self.config.MAX_VELOCITY:
                move_y = self.last_move_y + (self.config.MAX_VELOCITY * dt * (1 if move_y > self.last_move_y else -1))
        
        # Position limiting
        move_x = max(min(move_x, self.config.MAX_STEP_SIZE), -self.config.MAX_STEP_SIZE)
        move_y = max(min(move_y, self.config.MAX_STEP_SIZE), -self.config.MAX_STEP_SIZE)
        
        self.last_move_x = move_x
        self.last_move_y = move_y
        self.last_move_time = current_time
        
        # --- STARTUP SETTLING ---
        if self.frame_count < self.config.STARTUP_SETTLE_FRAMES:
            return f"SETTLING ({self.frame_count}/{self.config.STARTUP_SETTLE_FRAMES})", (0.0, 0.0), (x, y, w, h)

        class_name = self.detector.get_class_name(cls_id)
        return f"TRACKING {class_name}", (move_x, move_y), (x, y, w, h)
        
    def reset_target_size(self, w, h):
        """Reset target size to current object size"""
        old_target = self.target_size
        self.target_size = (w + h) / 2.0
        self.prev_error_depth = 0.0
        logging.info(f"Target size reset: {old_target:.1f} -> {self.target_size:.1f}")


# ==================== MAIN SYSTEM ====================
class TrackingSystem:
    def __init__(self, config: Config):
        self.config = config
        self.serial = SerialHandler(config)
        self.camera = CameraHandler(config)
        self.motor = GCodeTranslator()
        self.detector = ModelDetector(config)
        self.tracker = None
        self.is_running = False
        self.jog = JogController(
            max_feed=1200,
            min_feed=150,
            jog_distance=10000,
            accel_limit=2500
        )
    def run(self):
        # 1. Load model
        logging.info("Loading detection model...")
        if not self.detector.load_model():
            logging.error("Failed to load model. Exiting.")
            return
        
        # 2. Initialize camera
        if not self.camera.initialize(): 
            return
        
        # 3. Connect to serial
        self.serial.connect()
        
        # 4. Initialize tracker with detector
        self.tracker = ObjectTracker(
            self.config, 
            self.camera.frame_center_x,
            self.camera.frame_center_y,
            self.detector
        )
        
        # 5. Initial target selection
        ret, frame = self.camera.read_frame()
        if ret:
            if not self.tracker.initialize_target(frame):
                logging.error("Target selection cancelled.")
                return

        logging.info("Starting Control Loop...")
        self.is_running = True
        
        try:
            while self.is_running:
                ret, frame = self.camera.read_frame()
                if not ret: 
                    continue

                # Update tracking and get moves
                status, moves, bbox = self.tracker.update_and_calculate_move(frame)
                move_x, move_y = moves
                
                status_color = (0, 255, 0)
                
                if bbox:
                    x, y, w, h = bbox
                    
                    # Draw bounding box
                    if "LOST" in status or "SEARCHING" in status:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        status_color = (0, 0, 255)
                    elif "SETTLING" in status:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                        status_color = (0, 255, 255)
                    else:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        
                        # Draw center point
                        cx = x + (w // 2)
                        cy = y + (h // 2)
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                        
                        # Send move command if significant
                        if abs(move_x) > 1e-4 or abs(move_y) > 1e-4:
                            # ---- VELOCITY JOG CONTROL ----
                            jog_cmd = self.jog.velocity_to_jog(move_x, move_y)
                            if status.startswith("STOP"):
                                self.serial.jog_cancel()
                            elif jog_cmd:
                                self.serial.send_jog(jog_cmd)

                else:
                    status_color = (0, 0, 255)
                
                # Draw HUD
                self._draw_hud(frame, status, status_color)
                cv2.imshow("Model-Based Tracker", frame)
                
                # Handle input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    if bbox: 
                        self.tracker.reset_target_size(bbox[2], bbox[3])
                elif key == ord('h'):
                    self.serial.send_command(self.motor.go_home())
                elif key == ord('x'):
                    # Re-initialize target
                    self.tracker.initialize_target(frame)
                elif key == ord('p'):
                    self.serial.jog_cancel()
                    # Pause
                    logging.info("Paused. Press 'p' to resume.")
                    while True:
                        ret, frame = self.camera.read_frame()
                        if ret:
                            self._draw_hud(frame, "PAUSED", (255, 255, 0))
                            cv2.imshow("Model-Based Tracker", frame)
                        if cv2.waitKey(1) & 0xFF == ord('p'):
                            break

        except KeyboardInterrupt:
            logging.info("Stopping...")
        finally:
            self._shutdown()

    def _draw_hud(self, frame, status, color):
        # Top bar
        cv2.rectangle(frame, (0, 0), (640, 40), (0, 0, 0), -1)
        pos = self.motor.get_position()
        
        # Limit warnings
        limit_warning = ""
        if abs(pos.x) > 2.5 or abs(pos.y) > 2.5:
            limit_warning = " ⚠ NEAR LIMIT"
        
        stats = f"POS: ({pos.x:.2f}, {pos.y:.2f}){limit_warning} | {status}"
        cv2.putText(frame, stats, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        
        # Crosshair
        cx = self.camera.frame_center_x
        cy = self.camera.frame_center_y
        cv2.line(frame, (cx, 0), (cx, 480), (100, 100, 100), 1)
        cv2.line(frame, (0, cy), (640, cy), (100, 100, 100), 1)
        
        # Help text
        help_text = "Q:Quit R:ResetSize H:Home X:NewTarget P:Pause"
        cv2.putText(frame, help_text, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    def _shutdown(self):
        logging.info("Shutting down...")
        if self.serial.is_connected:
            self.serial.send_command(self.motor.go_home())
            time.sleep(1)
        self.serial.close()
        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    config = Config()
    system = TrackingSystem(config)
    system.run()