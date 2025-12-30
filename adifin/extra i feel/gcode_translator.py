"""
Enhanced G-Code Translator with Robust Error Handling
CoreXY/H-Bot Kinematics Implementation
"""

import logging
from typing import Optional, Tuple
from dataclasses import dataclass

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


@dataclass
class Position:
    """Represents a 2D position with validation"""
    x: float
    y: float
    
    def __post_init__(self):
        if not isinstance(self.x, (int, float)) or not isinstance(self.y, (int, float)):
            raise TypeError("Position coordinates must be numeric")


class GCodeTranslator:
    """
    Translates Cartesian coordinates to CoreXY kinematics with safety limits
    
    Kinematics:
        Motor_A = X - Y
        Motor_B = X + Y
    """
    
    def __init__(self, max_limit: float = 2.8, min_limit: float = -2.8):  # REDUCED limits for safety margin
        """
        Initialize translator with position tracking and limits
        
        Args:
            max_limit: Maximum position boundary (units)
            min_limit: Minimum position boundary (units)
        """
        self.current_x = 0.0
        self.current_y = 0.0
        self.MAX_LIMIT = max_limit
        self.MIN_LIMIT = min_limit
        self._command_count = 0
        self._limit_violations = 0
        self._at_limit = False  # NEW: Track if we're stuck at boundary
        
        logging.info(f"GCodeTranslator initialized: Limits [{self.MIN_LIMIT}, {self.MAX_LIMIT}]")
    
    def _clamp_value(self, value: float) -> float:
        """Helper to clamp value within limits"""
        return max(self.MIN_LIMIT, min(value, self.MAX_LIMIT))
    
    def _is_at_limit(self, value: float) -> bool:
        """Check if value is at boundary"""
        return abs(value - self.MAX_LIMIT) < 0.01 or abs(value - self.MIN_LIMIT) < 0.01

    def _format_coordinate(self, value: float) -> str:
        """Format coordinate value for G-code output"""
        if abs(value - round(value)) < 1e-9:
            return f"{int(round(value))}"
        return f"{value:.3f}"
    
    def generate_move_command(self, x_delta: float, y_delta: float) -> Optional[str]:
        """
        Generate relative G-code move command with CoreXY kinematics.
        Implements proportional scaling when approaching limits.
        
        Args:
            x_delta: Change in X position (user space)
            y_delta: Change in Y position (user space)
            
        Returns:
            G-code string if move is possible, None if completely blocked.
        """
        try:
            # Validate input types
            if not isinstance(x_delta, (int, float)) or not isinstance(y_delta, (int, float)):
                logging.error(f"Invalid input types: x_delta={type(x_delta)}, y_delta={type(y_delta)}")
                return None
            
            # 1. Calculate desired target position
            target_x = self.current_x + x_delta
            target_y = self.current_y + y_delta
            
            # 2. Calculate how much movement is actually allowed
            # Check each axis independently
            allowed_x = target_x
            allowed_y = target_y
            
            # Clamp X axis
            if target_x > self.MAX_LIMIT:
                allowed_x = self.MAX_LIMIT
            elif target_x < self.MIN_LIMIT:
                allowed_x = self.MIN_LIMIT
            
            # Clamp Y axis
            if target_y > self.MAX_LIMIT:
                allowed_y = self.MAX_LIMIT
            elif target_y < self.MIN_LIMIT:
                allowed_y = self.MIN_LIMIT
            
            # 3. Calculate actual deltas we can perform
            actual_x_delta = allowed_x - self.current_x
            actual_y_delta = allowed_y - self.current_y
            
            # 4. Check if we're completely stuck (at limit and trying to go further)
            x_at_limit = self._is_at_limit(self.current_x)
            y_at_limit = self._is_at_limit(self.current_y)
            
            # If both axes stuck and trying to move further out, return None
            if (abs(actual_x_delta) < 1e-6 and abs(actual_y_delta) < 1e-6):
                if not self._at_limit:
                    logging.warning(f"Position limit reached: ({self.current_x:.2f}, {self.current_y:.2f})")
                    self._at_limit = True
                return None
            
            # If we were at limit but now can move, log recovery
            if self._at_limit and (abs(actual_x_delta) > 1e-6 or abs(actual_y_delta) > 1e-6):
                logging.info(f"Recovered from limit, resuming tracking")
                self._at_limit = False
            
            # 5. Apply proportional scaling near boundaries for smoother approach
            SOFT_LIMIT_THRESHOLD = 0.3  # Start scaling when within 0.3 units of limit
            
            # Scale X if near limit
            if not x_at_limit:
                dist_to_max_x = self.MAX_LIMIT - self.current_x
                dist_to_min_x = self.current_x - self.MIN_LIMIT
                
                if dist_to_max_x < SOFT_LIMIT_THRESHOLD and actual_x_delta > 0:
                    scale_factor = dist_to_max_x / SOFT_LIMIT_THRESHOLD
                    actual_x_delta *= max(0.2, scale_factor)  # Min 20% speed
                elif dist_to_min_x < SOFT_LIMIT_THRESHOLD and actual_x_delta < 0:
                    scale_factor = dist_to_min_x / SOFT_LIMIT_THRESHOLD
                    actual_x_delta *= max(0.2, scale_factor)
            
            # Scale Y if near limit
            if not y_at_limit:
                dist_to_max_y = self.MAX_LIMIT - self.current_y
                dist_to_min_y = self.current_y - self.MIN_LIMIT
                
                if dist_to_max_y < SOFT_LIMIT_THRESHOLD and actual_y_delta > 0:
                    scale_factor = dist_to_max_y / SOFT_LIMIT_THRESHOLD
                    actual_y_delta *= max(0.2, scale_factor)
                elif dist_to_min_y < SOFT_LIMIT_THRESHOLD and actual_y_delta < 0:
                    scale_factor = dist_to_min_y / SOFT_LIMIT_THRESHOLD
                    actual_y_delta *= max(0.2, scale_factor)
            
            # 6. Apply CoreXY kinematics transformation
            motor_a = actual_x_delta - actual_y_delta
            motor_b = actual_x_delta + actual_y_delta
            
            # 7. Update internal state
            self.current_x += actual_x_delta
            self.current_y += actual_y_delta
            self._command_count += 1
            
            # Format G-code command
            motor_a_str = self._format_coordinate(motor_a)
            motor_b_str = self._format_coordinate(motor_b)
            
            gcode = f"G91 X{motor_a_str} Y{motor_b_str}"
            
            if self._command_count % 100 == 0:
                logging.info(f"Commands sent: {self._command_count}, Position: ({self.current_x:.3f}, {self.current_y:.3f})")
            
            return gcode
            
        except Exception as e:
            logging.error(f"Error generating move command: {e}")
            return None
    
    def go_home(self) -> str:
        """
        Generate absolute positioning command to return to origin
        """
        self.current_x = 0.0
        self.current_y = 0.0
        logging.info("Homing command generated")
        return "G90 X0 Y0"
    
    def get_position(self) -> Position:
        """Get current position"""
        return Position(self.current_x, self.current_y)
    
    def get_statistics(self) -> dict:
        """Get operation statistics"""
        return {
            'commands_sent': self._command_count,
            'limit_violations': self._limit_violations,
            'current_position': (self.current_x, self.current_y)
        }
    
    def reset_statistics(self):
        """Reset command counters"""
        self._command_count = 0
        self._limit_violations = 0
        logging.info("Statistics reset")


# Self-test verification with SERIAL OUTPUT
if __name__ == "__main__":
    import time
    
    print("=== G-Code Translator Logic Test ===\n")
    translator = GCodeTranslator(max_limit=3.2, min_limit=-3.2)

    # Test 1: Normal move
    print("Test 1: Normal Move")
    cmd = translator.generate_move_command(1.0, 1.0)
    print(f"Cmd: {cmd} | Pos: {translator.get_position()}")
    assert cmd is not None
    
    # Test 2: Move hitting limit (Clamping check)
    print("\nTest 2: Move hitting limit (should clamp, not fail)")
    # Current is (1,1). Move +5 X. Should clamp X to 3.2.
    # Delta should be 3.2 - 1.0 = 2.2
    cmd = translator.generate_move_command(5.0, 0.0)
    print(f"Cmd: {cmd} | Pos: {translator.get_position()}")
    pos = translator.get_position()
    assert abs(pos.x - 3.2) < 0.001, f"Failed to clamp X. Got {pos.x}"
    assert cmd is not None, "Should generate clamped move"

    # Test 3: Recovery move (Coming back)
    print("\nTest 3: Recovery move (Should allow moving back)")
    # Current X is 3.2. Move -1.0.
    cmd = translator.generate_move_command(-1.0, 0.0)
    print(f"Cmd: {cmd} | Pos: {translator.get_position()}")
    pos = translator.get_position()
    assert abs(pos.x - 2.2) < 0.001
    assert cmd is not None

    print("\n✓ Logic Verified")