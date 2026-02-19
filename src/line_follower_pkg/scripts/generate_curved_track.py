#!/usr/bin/env python3
"""Generate a curved track texture image."""

import cv2
import numpy as np
import os

def generate_curved_track(output_path, width=2000, height=2000):
    """
    Generate a track image with curved lines.

    Track layout (robot starts at bottom, moving up):
    - Straight section going forward (up in image, which is -Y in Gazebo)
    - Smooth curve to the LEFT
    - Straight section continuing after curve

    Args:
        output_path: Path to save the image
        width: Image width in pixels
        height: Image height in pixels
    """
    # Create gray background (like Gazebo ground)
    img = np.ones((height, width, 3), dtype=np.uint8) * 180

    # Track parameters
    line_color = (255, 255, 255)  # White lines
    line_thickness = 15
    lane_width = 100  # pixels between left and right lines (narrower)

    # Start position (bottom center of image)
    start_x = width // 2
    start_y = height - 100

    # Define path points for center line
    points_center = []

    # 1. Straight section going up (about 600 pixels = 3 meters at 200px/m)
    for y in range(start_y, start_y - 600, -10):
        points_center.append((start_x, y))

    # 2. Smooth curve to the LEFT (90 degrees)
    curve_radius = 400
    curve_center_x = start_x - curve_radius  # Center of curve is to the LEFT
    curve_center_y = start_y - 600

    # Curve from 0 degrees (pointing right) to 90 degrees (pointing up)
    # This makes a left turn
    for angle in range(0, 95, 5):
        rad = np.radians(angle)
        x = int(curve_center_x + curve_radius * np.cos(rad))
        y = int(curve_center_y - curve_radius * np.sin(rad))
        points_center.append((x, y))

    # 3. Straight section after curve (going left)
    last_point = points_center[-1]
    for i in range(1, 60):
        points_center.append((last_point[0] - i * 10, last_point[1]))

    # Convert to numpy array
    points_center = np.array(points_center)

    # Calculate left and right line points
    def offset_points(points, offset):
        """Offset points perpendicular to the path."""
        result = []
        for i in range(len(points)):
            if i == 0:
                dx = points[i+1][0] - points[i][0]
                dy = points[i+1][1] - points[i][1]
            elif i == len(points) - 1:
                dx = points[i][0] - points[i-1][0]
                dy = points[i][1] - points[i-1][1]
            else:
                dx = points[i+1][0] - points[i-1][0]
                dy = points[i+1][1] - points[i-1][1]

            # Normalize
            length = np.sqrt(dx*dx + dy*dy)
            if length > 0:
                dx /= length
                dy /= length

            # Perpendicular vector (rotate 90 degrees)
            px, py = -dy, dx

            result.append((int(points[i][0] + px * offset),
                          int(points[i][1] + py * offset)))
        return np.array(result)

    # Generate left and right lines for LEFT curve track
    left_points = offset_points(points_center, -lane_width // 2)
    right_points = offset_points(points_center, lane_width // 2)

    # Draw smooth lines using polylines (LEFT curve)
    cv2.polylines(img, [left_points], False, line_color, line_thickness, cv2.LINE_AA)
    cv2.polylines(img, [right_points], False, line_color, line_thickness, cv2.LINE_AA)

    # Right curve track (parallel to left track)
    # Start position for right track (to the right of left track)
    start_x_right = width // 2 + 300  # 300 pixels to the right
    start_y_right = height - 100

    # Define path points for center line of RIGHT track
    points_center_right = []

    # 1. Straight section going up
    for y in range(start_y_right, start_y_right - 600, -10):
        points_center_right.append((start_x_right, y))

    # 2. Smooth curve to the RIGHT (90 degrees)
    curve_radius_right = 400
    curve_center_x_right = start_x_right + curve_radius_right  # Center of curve is to the RIGHT
    curve_center_y_right = start_y_right - 600

    # Curve from 180 degrees to 90 degrees (right turn)
    for angle in range(180, 85, -5):
        rad = np.radians(angle)
        x = int(curve_center_x_right + curve_radius_right * np.cos(rad))
        y = int(curve_center_y_right - curve_radius_right * np.sin(rad))
        points_center_right.append((x, y))

    # 3. Straight section after curve (going right)
    last_point_right = points_center_right[-1]
    for i in range(1, 60):
        points_center_right.append((last_point_right[0] + i * 10, last_point_right[1]))

    # Convert to numpy array
    points_center_right = np.array(points_center_right)

    # Generate left and right lines for RIGHT curve track
    left_points_right = offset_points(points_center_right, -lane_width // 2)
    right_points_right = offset_points(points_center_right, lane_width // 2)

    # Draw smooth lines using polylines (RIGHT curve)
    cv2.polylines(img, [left_points_right], False, line_color, line_thickness, cv2.LINE_AA)
    cv2.polylines(img, [right_points_right], False, line_color, line_thickness, cv2.LINE_AA)

    # Save image
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, img)
    print(f"Track image saved to: {output_path}")
    print(f"Image size: {width}x{height}")

    return img

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, '..', 'worlds', 'textures', 'curved_track.png')
    generate_curved_track(output_path)
