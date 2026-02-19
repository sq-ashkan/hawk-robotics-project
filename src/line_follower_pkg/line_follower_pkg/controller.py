"""
PID Controller Module

Implements PID control for line following with CTE-based steering
and curvature-aware speed adjustment.
"""

import time
import numpy as np


class PIDController:
    """PID Controller for smooth robot control."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.1):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

        # Anti-windup limits
        self.integral_max = 1.0
        self.integral_min = -1.0

        # Output limits
        self.output_max = 1.0
        self.output_min = -1.0

    def reset(self):
        """Reset controller state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def update(self, error, current_time=None):
        """
        Update PID controller with new error value.

        Args:
            error: Current error (setpoint - measured)
            current_time: Current timestamp (optional, uses time.time() if None)

        Returns:
            Control output
        """
        if current_time is None:
            current_time = time.time()

        # Calculate dt
        if self.prev_time is None:
            dt = 0.1  # Default dt for first iteration
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.1

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(self.integral_min, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Limit output
        output = max(self.output_min, min(self.output_max, output))

        # Update state
        self.prev_error = error
        self.prev_time = current_time

        return output


class LineFollowerController:
    """
    Enhanced line follower controller with curvature-aware speed control.

    Uses CTE (Cross-Track Error) for steering and curvature radius
    for adaptive speed adjustment.
    """

    def __init__(self):
        # PID for steering (angular velocity)
        self.steering_pid = PIDController(kp=2.0, ki=0.02, kd=0.3)

        # Dead zone - don't steer if very close to center
        self.dead_zone = 0.05  # 5% of normalized range

        # Speed parameters (m/s)
        self.base_speed = 0.12
        self.min_speed = 0.06
        self.max_speed = 0.15

        # Turn parameters
        self.max_angular_velocity = 1.5  # rad/s

        # Curvature thresholds (pixels)
        self.sharp_curve_threshold = 300
        self.medium_curve_threshold = 800
        self.gentle_curve_threshold = 2000

        # State
        self.no_line_count = 0
        self.max_no_line_count = 0  # Stop immediately if no lines detected

        # Last known values for recovery
        self.last_cte = 0.0
        self.last_curvature = float('inf')

    def calculate_velocity(self, cte, curvature=float('inf'), lines_detected=True):
        """
        Calculate robot velocity based on CTE and curvature.

        Args:
            cte: Cross-Track Error, normalized [-1, 1]
                 Positive = lane center is to the right of robot
            curvature: Curvature radius in pixels (smaller = sharper curve)
            lines_detected: Whether any lines were detected

        Returns:
            (linear_velocity, angular_velocity) tuple
        """
        if not lines_detected:
            self.no_line_count += 1
            if self.no_line_count > self.max_no_line_count:
                # Stop if no lines detected
                return 0.0, 0.0
            else:
                # Use last known values to try to recover
                cte = self.last_cte
                curvature = self.last_curvature
        else:
            self.no_line_count = 0
            self.last_cte = cte
            self.last_curvature = curvature

        # Apply dead zone and calculate angular velocity
        if abs(cte) < self.dead_zone:
            angular = 0.0
            # Don't update PID to avoid integral windup
        else:
            # Reduce effective error by dead zone amount
            effective_cte = cte - np.sign(cte) * self.dead_zone
            # Positive CTE means lane center is to the right, so turn right (negative angular)
            angular = -self.steering_pid.update(effective_cte)
            angular = np.clip(angular, -self.max_angular_velocity, self.max_angular_velocity)

        # Adjust speed based on curvature
        if curvature < self.sharp_curve_threshold:
            speed_factor = 0.4
        elif curvature < self.medium_curve_threshold:
            speed_factor = 0.6
        elif curvature < self.gentle_curve_threshold:
            speed_factor = 0.8
        else:
            speed_factor = 1.0

        # Also reduce speed based on angular velocity (additional safety)
        angular_speed_factor = 1.0 - 0.3 * abs(angular) / self.max_angular_velocity

        # Combined speed factor
        combined_factor = min(speed_factor, angular_speed_factor)

        linear = self.base_speed * combined_factor
        linear = np.clip(linear, self.min_speed, self.max_speed)

        return linear, angular

    def calculate_velocity_simple(self, offset, lines_detected=True):
        """
        Simple velocity calculation (legacy compatibility).

        Args:
            offset: Normalized lane offset [-1, 1]
            lines_detected: Whether any lines were detected

        Returns:
            (linear_velocity, angular_velocity) tuple
        """
        return self.calculate_velocity(offset, float('inf'), lines_detected)

    def reset(self):
        """Reset controller state."""
        self.steering_pid.reset()
        self.no_line_count = 0
        self.last_cte = 0.0
        self.last_curvature = float('inf')


# Legacy function for backward compatibility
def calculate_velocity(center_offset):
    """Calculate linear and angular velocity based on center offset (legacy function)."""
    controller = LineFollowerController()
    linear, angular = controller.calculate_velocity_simple(center_offset)
    return linear, angular
