#!/usr/bin/env python3
"""
Homography Calibration Tool for Bird's-Eye View Transformation

This interactive tool allows you to select 4 points in the camera image
to define the perspective transformation for the bird's-eye view.

Usage:
    ros2 run line_follower_pkg calibrate_homography

Controls:
    Left Click  - Select/move a point
    R           - Reset points to default
    P           - Preview bird's-eye view transformation
    S           - Save homography points to config file
    Q           - Quit
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from .birdseye import (
    get_default_source_points,
    get_default_destination_points,
    save_homography_points,
    load_homography_points,
    DEFAULT_IMAGE_WIDTH,
    DEFAULT_IMAGE_HEIGHT,
    DEFAULT_OUTPUT_WIDTH,
    DEFAULT_OUTPUT_HEIGHT
)


class HomographyCalibrator(Node):
    """ROS2 Node for interactive homography calibration."""

    def __init__(self):
        super().__init__('homography_calibrator')

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Current frame
        self.current_frame = None
        self.image_width = DEFAULT_IMAGE_WIDTH
        self.image_height = DEFAULT_IMAGE_HEIGHT

        # Source points (trapezoid in original image)
        self.source_points = None

        # Destination points (rectangle in bird's-eye view)
        self.destination_points = get_default_destination_points()

        # Point being dragged
        self.dragging_point = -1
        self.point_radius = 10

        # Point colors (BGR)
        self.colors = [
            (0, 0, 255),    # Red - Top-left
            (0, 255, 0),    # Green - Top-right
            (255, 0, 0),    # Blue - Bottom-right
            (0, 255, 255),  # Yellow - Bottom-left
        ]
        self.point_names = ['Top-Left', 'Top-Right', 'Bottom-Right', 'Bottom-Left']

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Config directory
        self.config_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config'
        )
        os.makedirs(self.config_dir, exist_ok=True)

        self.get_logger().info('Homography Calibrator Node started')
        self.get_logger().info('Controls: Click to select points, R=reset, P=preview, S=save, Q=quit')

    def image_callback(self, msg: Image):
        """Process incoming camera images."""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_height, self.image_width = self.current_frame.shape[:2]

            # Initialize source points if not done yet
            if self.source_points is None:
                # Try to load from config first
                points = load_homography_points()
                if points is not None:
                    self.source_points = points['source_points']
                    self.destination_points = points['destination_points']
                    self.get_logger().info('Loaded existing homography points')
                else:
                    self.source_points = get_default_source_points(
                        self.image_width, self.image_height
                    )
                    self.get_logger().info('Using default homography points')

        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for point selection."""
        if self.source_points is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            # Check if clicking near any point
            for i, point in enumerate(self.source_points):
                dist = np.sqrt((x - point[0])**2 + (y - point[1])**2)
                if dist < self.point_radius * 2:
                    self.dragging_point = i
                    break

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.dragging_point >= 0:
                # Update point position
                self.source_points[self.dragging_point] = [x, y]

        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging_point = -1

    def draw_overlay(self, image: np.ndarray) -> np.ndarray:
        """Draw source points and trapezoid on image."""
        overlay = image.copy()

        if self.source_points is None:
            return overlay

        # Draw filled trapezoid with transparency
        pts = self.source_points.astype(np.int32)
        cv2.fillPoly(overlay, [pts], (0, 255, 0), lineType=cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.2, image, 0.8, 0, overlay)

        # Draw trapezoid edges
        for i in range(4):
            pt1 = tuple(pts[i])
            pt2 = tuple(pts[(i + 1) % 4])
            cv2.line(overlay, pt1, pt2, (0, 255, 0), 2, cv2.LINE_AA)

        # Draw points
        for i, point in enumerate(self.source_points):
            center = (int(point[0]), int(point[1]))

            # Outer circle
            cv2.circle(overlay, center, self.point_radius, self.colors[i], -1, cv2.LINE_AA)

            # Inner circle
            cv2.circle(overlay, center, self.point_radius - 4, (255, 255, 255), -1, cv2.LINE_AA)

            # Point number
            cv2.putText(
                overlay,
                str(i + 1),
                (center[0] - 5, center[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2
            )

            # Point name
            text_offset = (-50, -20) if i < 2 else (-50, 30)
            cv2.putText(
                overlay,
                self.point_names[i],
                (center[0] + text_offset[0], center[1] + text_offset[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                self.colors[i],
                1
            )

        # Draw instructions
        instructions = [
            'Click and drag points to adjust',
            'R: Reset | P: Preview | S: Save | Q: Quit'
        ]
        for i, text in enumerate(instructions):
            cv2.putText(
                overlay,
                text,
                (10, 25 + i * 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

        return overlay

    def preview_birdseye(self) -> np.ndarray:
        """Generate preview of bird's-eye view transformation."""
        if self.current_frame is None or self.source_points is None:
            return None

        # Calculate homography matrix
        H = cv2.getPerspectiveTransform(
            self.source_points.astype(np.float32),
            self.destination_points.astype(np.float32)
        )

        # Transform image
        birdseye = cv2.warpPerspective(
            self.current_frame,
            H,
            (DEFAULT_OUTPUT_WIDTH, DEFAULT_OUTPUT_HEIGHT),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )

        # Draw grid on bird's-eye view to show transformation
        grid_color = (100, 100, 100)
        grid_spacing = 50

        for x in range(0, birdseye.shape[1], grid_spacing):
            cv2.line(birdseye, (x, 0), (x, birdseye.shape[0]), grid_color, 1)
        for y in range(0, birdseye.shape[0], grid_spacing):
            cv2.line(birdseye, (0, y), (birdseye.shape[1], y), grid_color, 1)

        # Add title
        cv2.putText(
            birdseye,
            "Bird's-Eye View Preview",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        return birdseye

    def reset_points(self):
        """Reset source points to default."""
        self.source_points = get_default_source_points(
            self.image_width, self.image_height
        )
        self.get_logger().info('Points reset to default')

    def save_calibration(self):
        """Save current homography points to config file."""
        if self.source_points is None:
            self.get_logger().warn('No points to save')
            return False

        save_homography_points(
            self.source_points.astype(np.float32),
            self.destination_points.astype(np.float32)
        )
        self.get_logger().info('Homography points saved')
        return True

    def run_interactive(self):
        """Run interactive calibration with OpenCV windows."""
        cv2.namedWindow('Homography Calibration', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Homography Calibration', self.mouse_callback)

        preview_window_open = False

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.current_frame is not None:
                # Draw overlay on camera image
                display_image = self.draw_overlay(self.current_frame)
                cv2.imshow('Homography Calibration', display_image)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('r') or key == ord('R'):
                self.reset_points()

            elif key == ord('p') or key == ord('P'):
                # Toggle preview window
                preview = self.preview_birdseye()
                if preview is not None:
                    cv2.imshow("Bird's-Eye Preview", preview)
                    preview_window_open = True

            elif key == ord('s') or key == ord('S'):
                self.save_calibration()

            elif key == ord('q') or key == ord('Q'):
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HomographyCalibrator()

    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
