"""
Real Robot Controller - Continuous Proportional

Moves forward continuously at low speed while steering
proportionally based on detected lane offset.
"""


class RealRobotController:
    """
    Continuous proportional controller for real TurtleBot3.

    Always moves forward at low speed, adjusts angular velocity
    based on lane offset. Slows down in curves.
    """

    def __init__(self):
        # Forward speed (1/4 of original step speed)
        self.forward_speed = 0.02

        # Steering gain: angular = -offset * angular_gain
        self.angular_gain = 0.2

        # Max angular velocity (rad/s)
        self.max_angular = 0.3

        # Slow down factor when offset is large (sharp curve)
        self.curve_slowdown_threshold = 0.3

        # Safety: stop if no lines for too many frames
        self.no_line_count = 0
        self.max_no_line_count = 20

        # Track last known offset for no-line fallback
        self.last_offset = 0.0

    def calculate_velocity(self, offset, curvature=float('inf'),
                           lines_detected=True, confidence=0.0):
        """
        Continuous velocity calculation.

        Args:
            offset: Normalized lane offset [-1, 1]
                    Positive = lane center is to the right
            curvature: Not used
            lines_detected: Whether any lines were detected
            confidence: Detection confidence [0, 1]

        Returns:
            (linear_velocity, angular_velocity) tuple
        """
        # Safety: stop if no lines for too long
        if not lines_detected or confidence < 0.3:
            self.no_line_count += 1
            if self.no_line_count > self.max_no_line_count:
                return 0.0, 0.0
            # Use last known offset to keep steering
            offset = self.last_offset
        else:
            self.no_line_count = 0
            self.last_offset = offset

        # Proportional steering: positive offset → turn right → negative angular
        angular = -offset * self.angular_gain
        angular = max(-self.max_angular, min(self.max_angular, angular))

        # Forward speed: slow down in sharp curves
        linear = self.forward_speed
        if abs(offset) > self.curve_slowdown_threshold:
            linear *= 0.5

        return linear, angular

    def get_state(self):
        """Return current state description."""
        if self.no_line_count > self.max_no_line_count:
            return 'stopped'
        return 'moving'

    def reset(self):
        """Reset controller state."""
        self.no_line_count = 0
        self.last_offset = 0.0
