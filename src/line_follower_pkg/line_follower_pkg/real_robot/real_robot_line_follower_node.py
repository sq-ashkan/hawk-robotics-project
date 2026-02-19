"""
Real Robot Line Follower Node

Main ROS2 node for line following on real TurtleBot3 hardware.
Uses continuous proportional control: always moving forward with steering.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import time
import os
import yaml

from .real_line_detector import RealLineDetector
from .real_robot_controller import RealRobotController


class RealRobotLineFollowerNode(Node):
    """Main ROS2 node for real robot line following."""

    def __init__(self):
        super().__init__('real_robot_line_follower_node')

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Load parameters from config file
        self.load_parameters()

        # Initialize 4-point line detector (binary threshold)
        self.line_detector = RealLineDetector()
        self.get_logger().info('4-point line detector initialized (binary threshold)')

        # Initialize continuous proportional controller
        self.controller = RealRobotController()
        self.controller.forward_speed = self.forward_speed
        self.controller.angular_gain = self.angular_gain
        self.controller.max_angular = self.max_angular
        self.controller.max_no_line_count = self.max_no_line_count
        self.get_logger().info(
            f'Continuous controller: speed={self.forward_speed}m/s, '
            f'angular_gain={self.angular_gain}, max_angular={self.max_angular}'
        )

        # QoS profile for sensor topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=self.qos_depth
        )

        # Subscribe to camera image
        self.image_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile
        )

        # Debug image publisher
        if self.publish_debug_images:
            self.line_detection_pub = self.create_publisher(Image, self.topic_detection, 10)

        # Using TwistStamped for real robot
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            10
        )

        # State variables
        self.frame_count = 0
        self.following_enabled = True
        self.decision_count = 0
        self.last_mode = None

        # Statistics
        self.total_frames = 0
        self.detected_frames = 0
        self.start_time = time.time()

        self.get_logger().info('=' * 60)
        self.get_logger().info('Real Robot Line Follower - Continuous Mode')
        self.get_logger().info(f'Camera: {self.camera_topic}')
        self.get_logger().info(f'Cmd_vel: {self.cmd_vel_topic}')
        self.get_logger().info('=' * 60)

    def load_parameters(self):
        """Load parameters from YAML config file."""
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            'config', 'real_robot_params.yaml'
        )

        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Controller parameters
            ctrl = config.get('control', {})
            self.forward_speed = ctrl.get('forward_speed', 0.0075)
            self.angular_gain = ctrl.get('angular_gain', 0.2)
            self.max_angular = ctrl.get('max_angular', 0.3)

            # Safety parameters
            safety = config.get('safety', {})
            self.max_no_line_count = safety.get('max_no_line_count', 20)

            # Debug parameters
            debug = config.get('debug', {})
            self.publish_debug_images = debug.get('publish_debug_images', True)
            self.log_frequency = debug.get('log_frequency', 30)
            self.save_interval = debug.get('save_interval', 30)

            # Camera parameters
            cam = config.get('camera', {})
            self.camera_topic = cam.get('topic', '/camera/image_raw')
            self.qos_depth = cam.get('qos_depth', 20)

            # Topic names
            topics = config.get('topics', {})
            self.cmd_vel_topic = topics.get('cmd_vel', '/cmd_vel')
            self.topic_detection = topics.get('camera_detection', '/camera/line_detection')

            self.get_logger().info(f'Loaded parameters from {config_path}')
        else:
            # Default parameters
            self.forward_speed = 0.02
            self.angular_gain = 0.2
            self.max_angular = 0.3
            self.max_no_line_count = 20
            self.publish_debug_images = True
            self.log_frequency = 30
            self.save_interval = 30
            self.camera_topic = '/camera/image_raw'
            self.qos_depth = 20
            self.cmd_vel_topic = '/cmd_vel'
            self.topic_detection = '/camera/line_detection'
            self.get_logger().warn('Config file not found, using defaults')

    def image_callback(self, msg: Image):
        """Main callback for image processing pipeline."""
        self.frame_count += 1
        self.total_frames += 1
        current_time = time.time()

        # Log first image received
        if self.frame_count == 1:
            self.get_logger().info('First camera image received!')

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # ROTATE IMAGE 180 degrees (camera is upside down)
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Run 4-point line detection (includes crop, binary, noise removal)
        detection_result = self.line_detector.process_frame(cv_image, current_time)

        # Save debug images to Desktop (once at frame 30)
        if self.frame_count == 30:
            raw_path = os.path.expanduser('~/Desktop/camera_raw.jpg')
            cv2.imwrite(raw_path, cv_image)
            self.get_logger().info(f'Saved raw image: {raw_path}')
            binary = detection_result.get('binary')
            if binary is not None:
                save_path = os.path.expanduser('~/Desktop/camera.jpg')
                cv2.imwrite(save_path, binary)
                self.get_logger().info(f'Saved binary image: {save_path}')

        # Update statistics
        if detection_result.get('lines_found', False):
            self.detected_frames += 1

        # Publish debug detection image
        if self.publish_debug_images:
            line_image = self.line_detector.draw_lines(cv_image, detection_result, draw_all=True)
            try:
                line_msg = self.bridge.cv2_to_imgmsg(line_image, 'bgr8')
                line_msg.header = msg.header
                self.line_detection_pub.publish(line_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish detection: {e}')

        # Calculate velocity commands
        offset = detection_result['offset']
        curvature = detection_result.get('curvature', float('inf'))
        lines_detected = detection_result.get('lines_found', False)
        confidence = detection_result.get('confidence', 0.0)

        if self.following_enabled:
            linear, angular = self.controller.calculate_velocity(
                offset, curvature, lines_detected, confidence
            )
        else:
            linear, angular = 0.0, 0.0

        # Save debug image periodically or on mode change
        mode = detection_result.get('mode', '?')
        num_pts = detection_result.get('num_points', 0)
        mode_changed = mode != self.last_mode
        self.last_mode = mode

        if mode_changed or (self.frame_count % self.save_interval == 0):
            self.decision_count += 1
            debug_img = self.line_detector.draw_lines(cv_image, detection_result, draw_all=True)
            label = f"#{self.decision_count} [{mode}] offset={offset:.3f} vel=({linear:.4f}, {angular:.3f})"
            cv2.putText(debug_img, label, (10, debug_img.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            save_path = os.path.expanduser(f'~/Desktop/decision_{self.decision_count:03d}.jpg')
            cv2.imwrite(save_path, debug_img)

        # Publish velocity command using TwistStamped
        cmd_msg = TwistStamped()
        cmd_msg.twist.linear.x = linear
        cmd_msg.twist.angular.z = angular
        self.cmd_vel_pub.publish(cmd_msg)

        # Log status periodically
        if self.frame_count % self.log_frequency == 0:
            detection_rate = 100.0 * self.detected_frames / max(1, self.total_frames)
            elapsed = current_time - self.start_time
            fps = self.total_frames / max(1, elapsed)
            state = self.controller.get_state()

            self.get_logger().info(
                f'[{self.frame_count}] {state}, [{mode}] pts={num_pts}/4, '
                f'offset={offset:.3f}, conf={confidence:.2f}, '
                f'vel=({linear:.4f}, {angular:.3f}), '
                f'detect={detection_rate:.1f}%, fps={fps:.1f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RealRobotLineFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Stop the robot before shutting down
        stop_msg = TwistStamped()
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0
        node.cmd_vel_pub.publish(stop_msg)

        # Log final statistics
        if node.total_frames > 0:
            detection_rate = 100.0 * node.detected_frames / node.total_frames
            elapsed = time.time() - node.start_time
            fps = node.total_frames / max(1, elapsed)
            node.get_logger().info(f'Final stats: {node.total_frames} frames, '
                                  f'{detection_rate:.1f}% detected, {fps:.1f} fps')

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
