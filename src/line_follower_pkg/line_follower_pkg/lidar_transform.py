"""
LiDAR Transform Module

Transforms LiDAR data from polar coordinates to bird's-eye view image coordinates.
"""

import numpy as np
import math


class LidarTransform:
    """Handles transformation of LiDAR data to image coordinates."""

    def __init__(self, image_width=640, image_height=480):
        """
        Initialize LiDAR transform parameters.

        Args:
            image_width: Width of the bird's-eye view image
            image_height: Height of the bird's-eye view image
        """
        self.image_width = image_width
        self.image_height = image_height

        # TurtleBot3 Waffle LiDAR specs
        # LiDAR is mounted at the center-top of the robot
        # Camera is mounted at the front

        # Transform parameters (meters)
        # These define the relationship between LiDAR frame and bird's-eye view
        self.lidar_to_camera_x = 0.064  # LiDAR is behind camera
        self.lidar_to_camera_y = 0.0    # Centered

        # Bird's-eye view mapping parameters
        # How many meters are visible in the bird's-eye view
        self.view_ahead = 3.0    # meters ahead of robot
        self.view_behind = 0.5   # meters behind
        self.view_left = 1.5     # meters to left
        self.view_right = 1.5    # meters to right

        # Calculate pixels per meter
        self.pixels_per_meter_x = self.image_width / (self.view_left + self.view_right)
        self.pixels_per_meter_y = self.image_height / (self.view_ahead + self.view_behind)

        # Image center point (where robot is in bird's-eye view)
        self.robot_image_x = int(self.image_width / 2)
        self.robot_image_y = int(self.image_height - (self.view_behind * self.pixels_per_meter_y))

    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        """
        Convert LiDAR polar coordinates to Cartesian coordinates.

        Args:
            ranges: Array of distance measurements
            angle_min: Starting angle (radians)
            angle_increment: Angle between measurements (radians)

        Returns:
            List of (x, y) tuples in robot frame (meters)
        """
        points = []
        for i, r in enumerate(ranges):
            # Skip invalid readings
            if math.isinf(r) or math.isnan(r) or r <= 0.01 or r > 10.0:
                continue

            angle = angle_min + i * angle_increment

            # Convert to Cartesian (robot frame: x forward, y left)
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            points.append((x, y, r))  # Include range for coloring

        return points

    def cartesian_to_image(self, x, y):
        """
        Convert Cartesian coordinates (robot frame) to image coordinates.

        Args:
            x: Forward distance in meters (positive = ahead)
            y: Lateral distance in meters (positive = left)

        Returns:
            (img_x, img_y) pixel coordinates or None if out of bounds
        """
        # Apply LiDAR to camera offset
        x_cam = x - self.lidar_to_camera_x
        y_cam = y - self.lidar_to_camera_y

        # Convert to image coordinates
        # In image: x is horizontal (right positive), y is vertical (down positive)
        img_x = self.robot_image_x - int(y_cam * self.pixels_per_meter_x)
        img_y = self.robot_image_y - int(x_cam * self.pixels_per_meter_y)

        # Check bounds
        if 0 <= img_x < self.image_width and 0 <= img_y < self.image_height:
            return (img_x, img_y)
        return None

    def transform_scan(self, ranges, angle_min, angle_increment):
        """
        Transform full LiDAR scan to image coordinates.

        Args:
            ranges: Array of distance measurements
            angle_min: Starting angle (radians)
            angle_increment: Angle between measurements (radians)

        Returns:
            List of (img_x, img_y, range) tuples for valid points
        """
        # Convert to Cartesian
        cart_points = self.polar_to_cartesian(ranges, angle_min, angle_increment)

        # Convert to image coordinates
        image_points = []
        for x, y, r in cart_points:
            img_coord = self.cartesian_to_image(x, y)
            if img_coord is not None:
                image_points.append((img_coord[0], img_coord[1], r))

        return image_points

    def get_front_points(self, ranges, angle_min, angle_increment, fov_degrees=120):
        """
        Get only front-facing LiDAR points within specified FOV.

        Args:
            ranges: Array of distance measurements
            angle_min: Starting angle (radians)
            angle_increment: Angle between measurements (radians)
            fov_degrees: Field of view to consider (centered on front)

        Returns:
            List of (img_x, img_y, range) tuples
        """
        fov_rad = math.radians(fov_degrees)
        half_fov = fov_rad / 2

        image_points = []
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0.01 or r > 10.0:
                continue

            angle = angle_min + i * angle_increment

            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Check if within front FOV
            if abs(angle) <= half_fov:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                img_coord = self.cartesian_to_image(x, y)
                if img_coord is not None:
                    image_points.append((img_coord[0], img_coord[1], r))

        return image_points


def range_to_color(r, min_range=0.2, max_range=3.0):
    """
    Convert range to BGR color (red=close, green=far).

    Args:
        r: Range in meters
        min_range: Minimum range for color scale
        max_range: Maximum range for color scale

    Returns:
        (B, G, R) color tuple
    """
    # Normalize range to 0-1
    normalized = (r - min_range) / (max_range - min_range)
    normalized = max(0.0, min(1.0, normalized))

    # Interpolate from red (close) to green (far)
    r_val = int(255 * (1 - normalized))
    g_val = int(255 * normalized)
    b_val = 0

    return (b_val, g_val, r_val)
