#!/usr/bin/env python3
"""
Camera Calibration Node for TurtleBot3

This node captures images from the robot's camera and uses a chessboard pattern
to calculate the camera's intrinsic parameters (camera matrix and distortion coefficients).

Usage:
    ros2 run line_follower_pkg calibrate_camera

Controls:
    SPACE - Capture current frame for calibration
    C     - Run calibration with captured frames
    S     - Save calibration to file
    Q     - Quit
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from datetime import datetime


class CameraCalibrator(Node):
    """ROS2 Node for camera calibration using chessboard pattern."""

    def __init__(self):
        super().__init__('camera_calibrator')

        # Chessboard parameters
        # Inner corners (not squares!)
        self.CHESSBOARD_SIZE = (8, 6)  # (cols, rows) inner corners
        self.SQUARE_SIZE = 0.025  # Size of square in meters (2.5 cm)

        # Calibration flags
        self.calibration_flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH +
            cv2.CALIB_CB_NORMALIZE_IMAGE +
            cv2.CALIB_CB_FAST_CHECK
        )

        # Corner refinement criteria
        self.criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,  # max iterations
            0.001  # epsilon
        )

        # Prepare object points (3D points in real world space)
        # Like (0,0,0), (1,0,0), (2,0,0) ... (7,5,0)
        self.objp = np.zeros(
            (self.CHESSBOARD_SIZE[0] * self.CHESSBOARD_SIZE[1], 3),
            np.float32
        )
        self.objp[:, :2] = np.mgrid[
            0:self.CHESSBOARD_SIZE[0],
            0:self.CHESSBOARD_SIZE[1]
        ].T.reshape(-1, 2)
        self.objp *= self.SQUARE_SIZE

        # Storage for calibration points
        self.obj_points = []  # 3D points in real world
        self.img_points = []  # 2D points in image plane
        self.image_size = None

        # Calibration results
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.calibrated = False

        # CV Bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()

        # Current frame
        self.current_frame = None

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

        self.get_logger().info('Camera Calibrator Node started')
        self.get_logger().info(f'Chessboard size: {self.CHESSBOARD_SIZE[0]}x{self.CHESSBOARD_SIZE[1]} inner corners')
        self.get_logger().info(f'Square size: {self.SQUARE_SIZE * 100:.1f} cm')
        self.get_logger().info('Controls: SPACE=capture, C=calibrate, S=save, Q=quit')

    def image_callback(self, msg: Image):
        """Process incoming camera images."""
        try:
            # Convert ROS Image to OpenCV format
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            if self.image_size is None:
                self.image_size = (self.current_frame.shape[1], self.current_frame.shape[0])
                self.get_logger().info(f'Image size: {self.image_size[0]}x{self.image_size[1]}')

        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def find_chessboard(self, image: np.ndarray) -> tuple:
        """
        Find chessboard corners in an image.

        Returns:
            tuple: (success, corners, display_image)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray,
            self.CHESSBOARD_SIZE,
            self.calibration_flags
        )

        display_image = image.copy()

        if ret:
            # Refine corner positions for better accuracy
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), self.criteria
            )

            # Draw corners on display image
            cv2.drawChessboardCorners(
                display_image,
                self.CHESSBOARD_SIZE,
                corners_refined,
                ret
            )

            return True, corners_refined, display_image
        else:
            return False, None, display_image

    def capture_frame(self):
        """Capture current frame for calibration."""
        if self.current_frame is None:
            self.get_logger().warn('No frame available')
            return False

        ret, corners, _ = self.find_chessboard(self.current_frame)

        if ret:
            self.obj_points.append(self.objp)
            self.img_points.append(corners)
            count = len(self.obj_points)
            self.get_logger().info(f'Captured frame {count}')
            return True
        else:
            self.get_logger().warn('Chessboard not detected in current frame')
            return False

    def calibrate(self):
        """Run camera calibration with captured frames."""
        if len(self.obj_points) < 10:
            self.get_logger().warn(
                f'Need at least 10 frames for calibration, have {len(self.obj_points)}'
            )
            return False

        self.get_logger().info(f'Calibrating with {len(self.obj_points)} frames...')

        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = \
            cv2.calibrateCamera(
                self.obj_points,
                self.img_points,
                self.image_size,
                None,
                None
            )

        if ret:
            self.calibrated = True

            # Calculate reprojection error
            total_error = 0
            for i in range(len(self.obj_points)):
                img_points_proj, _ = cv2.projectPoints(
                    self.obj_points[i],
                    self.rvecs[i],
                    self.tvecs[i],
                    self.camera_matrix,
                    self.dist_coeffs
                )
                error = cv2.norm(
                    self.img_points[i],
                    img_points_proj,
                    cv2.NORM_L2
                ) / len(img_points_proj)
                total_error += error

            mean_error = total_error / len(self.obj_points)

            self.get_logger().info('Calibration successful!')
            self.get_logger().info(f'Reprojection error: {mean_error:.4f} pixels')
            self.get_logger().info(f'Camera Matrix:\n{self.camera_matrix}')
            self.get_logger().info(f'Distortion Coefficients:\n{self.dist_coeffs.ravel()}')

            return True
        else:
            self.get_logger().error('Calibration failed')
            return False

    def save_calibration(self, filename: str = None):
        """Save calibration results to YAML file."""
        if not self.calibrated:
            self.get_logger().warn('No calibration data to save')
            return False

        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.config_dir, f'camera_calibration.yaml')

        # Prepare data for saving
        calibration_data = {
            'image_width': self.image_size[0],
            'image_height': self.image_size[1],
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': self.camera_matrix.flatten().tolist()
            },
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': self.dist_coeffs.flatten().tolist()
            },
            'calibration_date': datetime.now().isoformat(),
            'num_frames': len(self.obj_points),
            'chessboard_size': list(self.CHESSBOARD_SIZE),
            'square_size_meters': self.SQUARE_SIZE
        }

        with open(filename, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)

        self.get_logger().info(f'Calibration saved to: {filename}')
        return True

    def run_interactive(self):
        """Run interactive calibration with OpenCV window."""
        cv2.namedWindow('Camera Calibration', cv2.WINDOW_NORMAL)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.current_frame is not None:
                # Find chessboard and display
                ret, corners, display_image = self.find_chessboard(self.current_frame)

                # Draw status
                status_text = f'Captured: {len(self.obj_points)} frames'
                if self.calibrated:
                    status_text += ' | CALIBRATED'
                cv2.putText(
                    display_image, status_text,
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2
                )

                # Draw instructions
                instructions = 'SPACE=capture | C=calibrate | S=save | Q=quit'
                cv2.putText(
                    display_image, instructions,
                    (10, display_image.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1
                )

                # Show chessboard detection status
                if ret:
                    cv2.putText(
                        display_image, 'Chessboard DETECTED',
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2
                    )
                else:
                    cv2.putText(
                        display_image, 'Chessboard NOT detected',
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2
                    )

                cv2.imshow('Camera Calibration', display_image)

            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):  # SPACE - capture
                self.capture_frame()
            elif key == ord('c') or key == ord('C'):  # C - calibrate
                self.calibrate()
            elif key == ord('s') or key == ord('S'):  # S - save
                self.save_calibration()
            elif key == ord('q') or key == ord('Q'):  # Q - quit
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()

    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
