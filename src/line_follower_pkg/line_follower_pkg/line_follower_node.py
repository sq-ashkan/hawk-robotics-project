"""
Line Follower Node for TurtleBot3

Main ROS2 node implementing the image processing pipeline for autonomous
line following. Processes camera images through undistortion, bird's-eye
transformation, line detection, and PID-based velocity control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

from .calibration import load_calibration, undistort, init_undistort_maps
from .birdseye import calculate_homography, transform_to_birdseye
from .line_detector import LineDetector
from .controller import LineFollowerController
from .lidar_transform import LidarTransform, range_to_color


class LineFollowerNode(Node):
    """Main ROS2 node for line following."""

    def __init__(self):
        super().__init__('line_follower_node')

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Load camera calibration parameters
        self.camera_params = load_calibration()
        if self.camera_params is not None:
            self.get_logger().info('Camera calibration loaded successfully')
            init_undistort_maps(self.camera_params)
        else:
            self.get_logger().warn('No camera calibration found - using raw images')

        # Calculate homography matrix for bird's-eye view
        self.homography_matrix = calculate_homography()
        if self.homography_matrix is not None:
            self.get_logger().info('Homography matrix calculated')
        else:
            self.get_logger().warn('Using default homography - may need calibration')

        # Initialize line detector
        self.line_detector = LineDetector()
        self.get_logger().info('Line detector initialized')

        # Initialize controller
        self.controller = LineFollowerController()
        self.get_logger().info('PID controller initialized')

        # Initialize LiDAR transform
        self.lidar_transform = LidarTransform()
        self.get_logger().info('LiDAR transform initialized')

        # LiDAR data storage
        self.last_scan = None

        # QoS profile for sensor topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to raw camera image
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )

        # Subscribe to LiDAR scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Publishers
        self.rectified_pub = self.create_publisher(
            Image,
            '/camera/rectified_image',
            10
        )

        self.birdseye_pub = self.create_publisher(
            Image,
            '/camera/birdseye_image',
            10
        )

        self.birdseye_lidar_pub = self.create_publisher(
            Image,
            '/camera/birdseye_with_lidar',
            10
        )

        self.line_detection_pub = self.create_publisher(
            Image,
            '/camera/line_detection',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # State variables
        self.frame_count = 0
        self.last_center = 0.0
        self.following_enabled = True  # Can be toggled for testing

        self.get_logger().info('Line Follower Node started!')
        self.get_logger().info('Subscribed to: /camera/image_raw, /scan')
        self.get_logger().info('Publishing: /camera/rectified_image, /camera/birdseye_image, /camera/birdseye_with_lidar, /camera/line_detection, /cmd_vel')

    def scan_callback(self, msg: LaserScan):
        """
        Callback for LiDAR scan data.

        Args:
            msg: ROS LaserScan message
        """
        self.last_scan = msg

    def draw_lidar_points(self, image, scan_msg):
        """
        Draw LiDAR points on bird's-eye view image.

        Args:
            image: Bird's-eye view image
            scan_msg: LaserScan message

        Returns:
            Image with LiDAR points drawn
        """
        if scan_msg is None:
            return image

        output = image.copy()

        # Get LiDAR points in image coordinates
        points = self.lidar_transform.get_front_points(
            scan_msg.ranges,
            scan_msg.angle_min,
            scan_msg.angle_increment,
            fov_degrees=120
        )

        # Draw points
        for img_x, img_y, r in points:
            color = range_to_color(r)
            cv2.circle(output, (int(img_x), int(img_y)), 3, color, -1)

        # Draw robot position marker
        robot_x = self.lidar_transform.robot_image_x
        robot_y = self.lidar_transform.robot_image_y
        cv2.circle(output, (robot_x, robot_y), 8, (255, 255, 0), 2)
        cv2.circle(output, (robot_x, robot_y), 3, (255, 255, 0), -1)

        return output

    def gray_out_blind_spots(self, image):
        """
        Gray out areas outside camera FOV.

        Args:
            image: Bird's-eye view image

        Returns:
            Image with blind spots grayed out
        """
        output = image.copy()
        height, width = output.shape[:2]

        # Create mask for camera FOV (trapezoidal region)
        # This is approximate and should be calibrated
        mask = np.zeros((height, width), dtype=np.uint8)

        # Define FOV polygon (approximate)
        fov_points = np.array([
            [width // 4, 0],
            [3 * width // 4, 0],
            [width, height],
            [0, height]
        ], dtype=np.int32)

        cv2.fillPoly(mask, [fov_points], 255)

        # Convert outside FOV to grayscale
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Blend: use original inside FOV, gray outside
        output = np.where(mask[:, :, np.newaxis] > 0, output, gray_bgr)

        return output

    def image_callback(self, msg: Image):
        """
        Main callback for image processing pipeline.

        Args:
            msg: ROS Image message from camera
        """
        self.frame_count += 1
        current_time = time.time()

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Undistort image
        if self.camera_params is not None:
            undistorted = undistort(cv_image, self.camera_params, crop=False)
        else:
            undistorted = cv_image

        # Publish rectified image
        try:
            rectified_msg = self.bridge.cv2_to_imgmsg(undistorted, 'bgr8')
            rectified_msg.header = msg.header
            self.rectified_pub.publish(rectified_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish rectified image: {e}')

        # Transform to bird's-eye view
        birdseye = transform_to_birdseye(undistorted, self.homography_matrix)

        # Publish bird's-eye view image
        try:
            birdseye_msg = self.bridge.cv2_to_imgmsg(birdseye, 'bgr8')
            birdseye_msg.header = msg.header
            self.birdseye_pub.publish(birdseye_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish birdseye image: {e}')

        # Add LiDAR overlay
        birdseye_with_lidar = self.draw_lidar_points(birdseye, self.last_scan)

        # Publish bird's-eye with LiDAR
        try:
            lidar_msg = self.bridge.cv2_to_imgmsg(birdseye_with_lidar, 'bgr8')
            lidar_msg.header = msg.header
            self.birdseye_lidar_pub.publish(lidar_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish birdseye with lidar: {e}')

        # Detect lines
        detection_result = self.line_detector.process_frame(birdseye, current_time)

        # Draw detected lines
        line_image = self.line_detector.draw_lines(birdseye, detection_result, draw_all=True)

        # Publish line detection image
        try:
            line_msg = self.bridge.cv2_to_imgmsg(line_image, 'bgr8')
            line_msg.header = msg.header
            self.line_detection_pub.publish(line_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish line detection: {e}')

        # Calculate velocity commands
        offset = detection_result['offset']
        curvature = detection_result.get('curvature', float('inf'))
        lines_detected = detection_result.get('lines_found', False)

        if self.following_enabled:
            # Use new curvature-aware velocity calculation
            linear, angular = self.controller.calculate_velocity(
                offset, curvature, lines_detected
            )
        else:
            linear, angular = 0.0, 0.0

        # Publish velocity command (TwistStamped for Gazebo bridge)
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.twist.linear.x = linear
        cmd_msg.twist.angular.z = angular
        self.cmd_vel_pub.publish(cmd_msg)

        # Log status periodically
        if self.frame_count % 30 == 0:
            left_str = "L" if detection_result.get('left_fitx') is not None else "-"
            right_str = "R" if detection_result.get('right_fitx') is not None else "-"
            curv_str = f"curv={curvature:.0f}" if curvature < 10000 else "curv=INF"

            self.get_logger().info(
                f'Frame {self.frame_count}: [{left_str}{right_str}] CTE={offset:.3f}, {curv_str}, '
                f'vel=({linear:.2f}, {angular:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Stop the robot before shutting down
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
