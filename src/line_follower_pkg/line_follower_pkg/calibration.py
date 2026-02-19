"""
Camera calibration module

This module provides functions to load camera calibration parameters
and undistort images using OpenCV.
"""

import os
import yaml
import numpy as np
import cv2
from cv_bridge import CvBridge


# Global bridge instance for ROS <-> OpenCV conversion
_bridge = CvBridge()

# Cache for undistortion maps (computed once for efficiency)
_undistort_maps = None
_roi = None
_new_camera_matrix = None


def load_calibration(config_path: str = None) -> dict:
    """
    Load camera calibration parameters from YAML file.

    Args:
        config_path: Path to the calibration YAML file.
                    If None, looks in default config directory.

    Returns:
        Dictionary containing:
            - camera_matrix: 3x3 numpy array (intrinsic matrix K)
            - dist_coeffs: 1x5 numpy array (distortion coefficients)
            - image_width: int
            - image_height: int
        Or None if file not found.
    """
    if config_path is None:
        # Default config path
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config',
            'camera_calibration.yaml'
        )

    if not os.path.exists(config_path):
        print(f"[calibration] Warning: Calibration file not found: {config_path}")
        return None

    try:
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        # Extract camera matrix
        camera_matrix = np.array(
            data['camera_matrix']['data'],
            dtype=np.float64
        ).reshape(3, 3)

        # Extract distortion coefficients
        dist_coeffs = np.array(
            data['distortion_coefficients']['data'],
            dtype=np.float64
        ).reshape(1, -1)

        calibration = {
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs,
            'image_width': data['image_width'],
            'image_height': data['image_height']
        }

        print(f"[calibration] Loaded calibration from: {config_path}")
        return calibration

    except Exception as e:
        print(f"[calibration] Error loading calibration: {e}")
        return None


def init_undistort_maps(camera_params: dict, alpha: float = 1.0):
    """
    Initialize undistortion maps for efficient undistortion.
    Should be called once at startup.

    Args:
        camera_params: Dictionary with camera_matrix, dist_coeffs, image_width, image_height
        alpha: Free scaling parameter (0 = crop all black pixels, 1 = keep all pixels)
    """
    global _undistort_maps, _roi, _new_camera_matrix

    if camera_params is None:
        return

    camera_matrix = camera_params['camera_matrix']
    dist_coeffs = camera_params['dist_coeffs']
    image_size = (camera_params['image_width'], camera_params['image_height'])

    # Get optimal new camera matrix
    _new_camera_matrix, _roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        image_size,
        alpha,
        image_size
    )

    # Compute undistortion maps
    _undistort_maps = cv2.initUndistortRectifyMap(
        camera_matrix,
        dist_coeffs,
        None,
        _new_camera_matrix,
        image_size,
        cv2.CV_16SC2
    )

    print("[calibration] Undistortion maps initialized")


def undistort(image, camera_params: dict, crop: bool = True):
    """
    Remove lens distortion from an image.

    This function can handle both ROS Image messages and OpenCV images (numpy arrays).

    Args:
        image: Input image (ROS Image message or numpy array)
        camera_params: Dictionary with camera calibration parameters
                      (from load_calibration())
        crop: If True, crop to valid ROI; if False, keep full image

    Returns:
        Undistorted image in the same format as input
        (ROS Image if input was ROS Image, numpy array if input was numpy array)
    """
    global _undistort_maps, _roi, _new_camera_matrix

    # Handle case when no calibration is available
    if camera_params is None:
        return image

    # Check if input is ROS Image message
    is_ros_image = hasattr(image, 'encoding')

    if is_ros_image:
        # Convert ROS Image to OpenCV
        try:
            cv_image = _bridge.imgmsg_to_cv2(image, 'bgr8')
        except Exception as e:
            print(f"[calibration] Error converting ROS image: {e}")
            return image
    else:
        cv_image = image

    # Initialize maps if not done yet
    if _undistort_maps is None:
        init_undistort_maps(camera_params)

    # Use precomputed maps for efficient undistortion
    if _undistort_maps is not None:
        undistorted = cv2.remap(
            cv_image,
            _undistort_maps[0],
            _undistort_maps[1],
            cv2.INTER_LINEAR
        )

        # Crop to ROI if requested
        if crop and _roi is not None:
            x, y, w, h = _roi
            if w > 0 and h > 0:
                undistorted = undistorted[y:y+h, x:x+w]
    else:
        # Fallback to direct undistort (slower)
        undistorted = cv2.undistort(
            cv_image,
            camera_params['camera_matrix'],
            camera_params['dist_coeffs']
        )

    # Convert back to ROS Image if input was ROS Image
    if is_ros_image:
        try:
            result = _bridge.cv2_to_imgmsg(undistorted, 'bgr8')
            result.header = image.header
            return result
        except Exception as e:
            print(f"[calibration] Error converting back to ROS image: {e}")
            return image
    else:
        return undistorted


def get_new_camera_matrix() -> np.ndarray:
    """
    Get the optimal new camera matrix computed during undistortion setup.

    Returns:
        3x3 numpy array or None if not initialized
    """
    return _new_camera_matrix


def get_roi() -> tuple:
    """
    Get the valid ROI after undistortion.

    Returns:
        Tuple (x, y, width, height) or None if not initialized
    """
    return _roi


def calibrate():
    """
    Placeholder for camera calibration.
    Use the calibrate_camera node for interactive calibration.
    """
    print("[calibration] Use 'ros2 run line_follower_pkg calibrate_camera' for calibration")
    return None
