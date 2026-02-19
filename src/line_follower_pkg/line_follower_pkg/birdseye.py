"""
Bird's-Eye View Transformation Module

Calculates homography matrix and transforms camera images to
a top-down view for lane detection.
"""

import os
import numpy as np
import cv2
import yaml


# Default image dimensions (TurtleBot3 Waffle camera)
DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 480

# Default output dimensions for bird's-eye view
DEFAULT_OUTPUT_WIDTH = 640
DEFAULT_OUTPUT_HEIGHT = 480

# Cache for homography matrix and inverse
_homography_matrix = None
_homography_inverse = None
_warp_maps = None


def load_homography_points(config_path: str = None, use_real_robot: bool = False) -> dict:
    """
    Load homography source and destination points from YAML file.

    Args:
        config_path: Path to the homography YAML file.
                    If None, looks in default config directory.
        use_real_robot: If True, loads real_robot_homography.yaml instead of homography.yaml

    Returns:
        Dictionary with 'source_points' and 'destination_points' as numpy arrays,
        or None if file not found.
    """
    if config_path is None:
        config_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config'
        )

        # Select config file based on use_real_robot flag
        if use_real_robot:
            config_file = 'real_robot_homography.yaml'
        else:
            config_file = 'homography.yaml'

        config_path = os.path.join(config_dir, config_file)

    if not os.path.exists(config_path):
        # If real robot config not found, try fallback to simulation config
        if use_real_robot:
            print(f"[birdseye] Real robot homography not found at {config_path}")
            fallback_path = os.path.join(
                os.path.dirname(config_path),
                'homography.yaml'
            )
            if os.path.exists(fallback_path):
                print(f"[birdseye] Using fallback simulation homography")
                config_path = fallback_path
            else:
                return None
        else:
            return None

    try:
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        source_points = np.array(data['source_points'], dtype=np.float32)
        destination_points = np.array(data['destination_points'], dtype=np.float32)

        print(f"[birdseye] Loaded homography from: {config_path}")
        return {
            'source_points': source_points,
            'destination_points': destination_points
        }
    except Exception as e:
        print(f"[birdseye] Error loading homography points: {e}")
        return None


def save_homography_points(source_points: np.ndarray, destination_points: np.ndarray,
                           config_path: str = None):
    """
    Save homography points to YAML file.

    Args:
        source_points: 4x2 numpy array of source points
        destination_points: 4x2 numpy array of destination points
        config_path: Output path (default: config/homography.yaml)
    """
    if config_path is None:
        config_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config'
        )
        os.makedirs(config_dir, exist_ok=True)
        config_path = os.path.join(config_dir, 'homography.yaml')

    data = {
        'source_points': source_points.tolist(),
        'destination_points': destination_points.tolist()
    }

    with open(config_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

    print(f"[birdseye] Homography points saved to: {config_path}")


def get_default_source_points(image_width: int = DEFAULT_IMAGE_WIDTH,
                               image_height: int = DEFAULT_IMAGE_HEIGHT) -> np.ndarray:
    """
    Get default source points for perspective transformation.

    These points define a trapezoid in the original image that corresponds
    to the road area visible to the camera.

    The points are ordered: top-left, top-right, bottom-right, bottom-left

    Args:
        image_width: Width of the input image
        image_height: Height of the input image

    Returns:
        4x2 numpy array of source points
    """
    # Default trapezoid points for TurtleBot3 Waffle camera
    # These values work well for the standard camera setup
    # but may need adjustment based on camera angle and height

    # The trapezoid is narrower at the top (far from robot)
    # and wider at the bottom (close to robot)

    top_width_ratio = 0.4  # Width at top as ratio of image width
    bottom_width_ratio = 0.9  # Width at bottom as ratio of image width
    top_y_ratio = 0.5  # Y position of top edge as ratio of image height
    bottom_y_ratio = 0.95  # Y position of bottom edge as ratio of image height

    top_margin = (1.0 - top_width_ratio) / 2
    bottom_margin = (1.0 - bottom_width_ratio) / 2

    source_points = np.array([
        [image_width * top_margin, image_height * top_y_ratio],  # Top-left
        [image_width * (1 - top_margin), image_height * top_y_ratio],  # Top-right
        [image_width * (1 - bottom_margin), image_height * bottom_y_ratio],  # Bottom-right
        [image_width * bottom_margin, image_height * bottom_y_ratio]  # Bottom-left
    ], dtype=np.float32)

    return source_points


def get_default_destination_points(output_width: int = DEFAULT_OUTPUT_WIDTH,
                                    output_height: int = DEFAULT_OUTPUT_HEIGHT) -> np.ndarray:
    """
    Get default destination points for perspective transformation.

    These points define a rectangle in the output (bird's-eye) image.

    Args:
        output_width: Width of the output image
        output_height: Height of the output image

    Returns:
        4x2 numpy array of destination points
    """
    margin_ratio = 0.1  # Margin on sides

    margin_x = output_width * margin_ratio
    margin_y_top = output_height * 0.05
    margin_y_bottom = output_height * 0.05

    destination_points = np.array([
        [margin_x, margin_y_top],  # Top-left
        [output_width - margin_x, margin_y_top],  # Top-right
        [output_width - margin_x, output_height - margin_y_bottom],  # Bottom-right
        [margin_x, output_height - margin_y_bottom]  # Bottom-left
    ], dtype=np.float32)

    return destination_points


def calculate_homography(image_shape: tuple = None,
                         source_points: np.ndarray = None,
                         destination_points: np.ndarray = None,
                         use_real_robot: bool = False) -> np.ndarray:
    """
    Calculate homography matrix for perspective transformation.

    The homography matrix H transforms points from the original image
    to the bird's-eye view using: p' = H * p

    Args:
        image_shape: Tuple (height, width) of input image, or None for defaults
        source_points: 4x2 numpy array of source points, or None for defaults
        destination_points: 4x2 numpy array of destination points, or None for defaults
        use_real_robot: If True, loads real_robot_homography.yaml instead of homography.yaml

    Returns:
        3x3 numpy array representing the homography matrix,
        or None if calculation fails
    """
    global _homography_matrix, _homography_inverse

    # Try to load from config file first
    points = load_homography_points(use_real_robot=use_real_robot)
    if points is not None:
        source_points = points['source_points']
        destination_points = points['destination_points']
        print("[birdseye] Using homography points from config file")
    else:
        # Use defaults
        if image_shape is not None:
            image_height, image_width = image_shape[:2]
        else:
            image_width = DEFAULT_IMAGE_WIDTH
            image_height = DEFAULT_IMAGE_HEIGHT

        if source_points is None:
            source_points = get_default_source_points(image_width, image_height)

        if destination_points is None:
            destination_points = get_default_destination_points()

        print("[birdseye] Using default homography points")

    # Calculate homography matrix
    try:
        _homography_matrix = cv2.getPerspectiveTransform(
            source_points,
            destination_points
        )

        # Also calculate inverse for later use
        _homography_inverse = cv2.getPerspectiveTransform(
            destination_points,
            source_points
        )

        print("[birdseye] Homography matrix calculated successfully")
        return _homography_matrix

    except Exception as e:
        print(f"[birdseye] Error calculating homography: {e}")
        return None


def transform_to_birdseye(image: np.ndarray,
                          homography_matrix: np.ndarray = None,
                          output_size: tuple = None) -> np.ndarray:
    """
    Transform image to bird's-eye view using perspective transformation.

    Args:
        image: Input image (numpy array, BGR format)
        homography_matrix: 3x3 homography matrix, or None to use cached/calculate
        output_size: Tuple (width, height) for output image, or None for defaults

    Returns:
        Transformed bird's-eye view image
    """
    global _homography_matrix

    if image is None:
        return None

    # Use provided matrix or cached matrix
    if homography_matrix is not None:
        H = homography_matrix
    elif _homography_matrix is not None:
        H = _homography_matrix
    else:
        # Calculate homography if not available
        H = calculate_homography(image.shape)
        if H is None:
            print("[birdseye] Warning: Could not calculate homography, returning original image")
            return image

    # Determine output size
    if output_size is None:
        output_size = (DEFAULT_OUTPUT_WIDTH, DEFAULT_OUTPUT_HEIGHT)

    # Apply perspective transformation
    try:
        birdseye = cv2.warpPerspective(
            image,
            H,
            output_size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)  # Black border
        )
        return birdseye

    except Exception as e:
        print(f"[birdseye] Error in perspective transform: {e}")
        return image


def transform_from_birdseye(image: np.ndarray,
                            output_size: tuple = None) -> np.ndarray:
    """
    Transform from bird's-eye view back to original perspective.

    Args:
        image: Bird's-eye view image
        output_size: Tuple (width, height) for output image

    Returns:
        Image in original camera perspective
    """
    global _homography_inverse

    if image is None or _homography_inverse is None:
        return image

    if output_size is None:
        output_size = (DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT)

    try:
        original = cv2.warpPerspective(
            image,
            _homography_inverse,
            output_size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )
        return original

    except Exception as e:
        print(f"[birdseye] Error in inverse transform: {e}")
        return image


def transform_point_to_birdseye(point: tuple,
                                 homography_matrix: np.ndarray = None) -> tuple:
    """
    Transform a single point from original image to bird's-eye view.

    Args:
        point: Tuple (x, y) in original image coordinates
        homography_matrix: 3x3 homography matrix, or None to use cached

    Returns:
        Tuple (x, y) in bird's-eye view coordinates
    """
    global _homography_matrix

    H = homography_matrix if homography_matrix is not None else _homography_matrix
    if H is None:
        return point

    # Convert point to homogeneous coordinates
    pt = np.array([[point[0], point[1]]], dtype=np.float32)
    pt = pt.reshape(-1, 1, 2)

    # Transform point
    transformed = cv2.perspectiveTransform(pt, H)

    return (transformed[0][0][0], transformed[0][0][1])


def transform_point_from_birdseye(point: tuple) -> tuple:
    """
    Transform a single point from bird's-eye view to original image.

    Args:
        point: Tuple (x, y) in bird's-eye view coordinates

    Returns:
        Tuple (x, y) in original image coordinates
    """
    global _homography_inverse

    if _homography_inverse is None:
        return point

    # Convert point to homogeneous coordinates
    pt = np.array([[point[0], point[1]]], dtype=np.float32)
    pt = pt.reshape(-1, 1, 2)

    # Transform point
    transformed = cv2.perspectiveTransform(pt, _homography_inverse)

    return (transformed[0][0][0], transformed[0][0][1])


def get_homography_matrix() -> np.ndarray:
    """Get the cached homography matrix."""
    return _homography_matrix


def get_inverse_homography_matrix() -> np.ndarray:
    """Get the cached inverse homography matrix."""
    return _homography_inverse
