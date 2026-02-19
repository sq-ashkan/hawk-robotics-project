"""
Line Detector Module

Detects lane lines in bird's-eye view images using threshold-based
white pixel detection and calculates the lane center offset.
"""

import cv2
import numpy as np


class LineDetector:
    """Robust lane detector using weighted pixel analysis."""

    def __init__(self):
        # White pixel threshold (tuned for bird's-eye view brightness)
        self.white_threshold = 110

        # Gaussian blur
        self.blur_kernel = (5, 5)

        # Smoothing
        self.center_history = []
        self.history_size = 10  # More smoothing for stability

        # Detection state
        self.detected = False
        self.last_center = None

    def preprocess(self, image):
        """Convert to binary image with white lines."""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        blurred = cv2.GaussianBlur(gray, self.blur_kernel, 0)
        _, binary = cv2.threshold(blurred, self.white_threshold, 255, cv2.THRESH_BINARY)

        return binary

    def find_lane_center(self, binary_image):
        """
        Find lane center using row-by-row analysis.

        Scans multiple rows and finds left/right line positions,
        then calculates the center.
        """
        height, width = binary_image.shape
        img_center = width // 2

        left_positions = []
        right_positions = []

        # Scan rows from bottom to middle (closer = more weight)
        for row_idx in range(height - 10, height // 3, -5):
            row = binary_image[row_idx, :]
            white_cols = np.where(row > 105)[0]

            if len(white_cols) == 0:
                continue

            # Find left line (closest white pixels to the left of center)
            left_cols = white_cols[white_cols < img_center]
            right_cols = white_cols[white_cols >= img_center]

            if len(left_cols) > 0:
                # Get the rightmost point of left line cluster
                left_positions.append(np.max(left_cols))

            if len(right_cols) > 0:
                # Get the leftmost point of right line cluster
                right_positions.append(np.min(right_cols))

        # Calculate lane center
        left_x = None
        right_x = None

        if len(left_positions) > 0:
            left_x = np.median(left_positions)

        if len(right_positions) > 0:
            right_x = np.median(right_positions)

        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2
        elif left_x is not None:
            lane_center = left_x + 50  # Offset when only left line visible
        elif right_x is not None:
            lane_center = right_x - 50  # Offset when only right line visible
        else:
            lane_center = None

        return left_x, right_x, lane_center

    def calculate_curvature(self, binary_image, left_x, right_x):
        """
        Estimate curvature by comparing top and bottom line positions.
        """
        height, width = binary_image.shape
        img_center = width // 2

        # Check top portion of image (look-ahead)
        top_left = []
        top_right = []

        for row_idx in range(height // 4, height // 2, 5):
            row = binary_image[row_idx, :]
            white_cols = np.where(row > 105)[0]

            if len(white_cols) == 0:
                continue

            left_cols = white_cols[white_cols < img_center]
            right_cols = white_cols[white_cols >= img_center]

            if len(left_cols) > 0:
                top_left.append(np.max(left_cols))
            if len(right_cols) > 0:
                top_right.append(np.min(right_cols))

        # Calculate curvature based on position difference
        curvature = float('inf')

        if len(top_left) > 2 and left_x is not None:
            top_left_x = np.median(top_left)
            left_diff = top_left_x - left_x
            if abs(left_diff) > 10:
                curvature = min(curvature, abs(500 / left_diff))

        if len(top_right) > 2 and right_x is not None:
            top_right_x = np.median(top_right)
            right_diff = top_right_x - right_x
            if abs(right_diff) > 10:
                curvature = min(curvature, abs(500 / right_diff))

        return curvature

    def process_frame(self, image, current_time=0):
        """
        Process a frame to detect lane and calculate offset.
        """
        height, width = image.shape[:2]
        img_center = width / 2

        # Preprocess
        binary = self.preprocess(image)

        # Find lane positions
        left_x, right_x, lane_center = self.find_lane_center(binary)

        # Smooth center position
        if lane_center is not None:
            self.center_history.append(lane_center)
            if len(self.center_history) > self.history_size:
                self.center_history.pop(0)
            lane_center = np.mean(self.center_history)
            self.last_center = lane_center
        elif self.last_center is not None:
            lane_center = self.last_center

        # Calculate offset (normalized -1 to 1)
        if lane_center is not None:
            offset = (lane_center - img_center) / (width / 2)
            offset = np.clip(offset, -1.0, 1.0)
        else:
            offset = 0.0

        # Calculate curvature
        curvature = self.calculate_curvature(binary, left_x, right_x)

        lines_found = (left_x is not None) or (right_x is not None)
        self.detected = lines_found

        return {
            'offset': offset,
            'curvature': curvature,
            'lane_center': lane_center,
            'left_x': left_x,
            'right_x': right_x,
            'lines_found': lines_found,
            'binary': binary,
            # Legacy compatibility
            'left_fit': None,
            'right_fit': None,
            'left_fitx': None,
            'right_fitx': None,
            'ploty': None,
            'leftx': np.array([]),
            'lefty': np.array([]),
            'rightx': np.array([]),
            'righty': np.array([]),
            'histogram': None,
            'left_line': None,
            'right_line': None,
            'all_lines': [],
            'edges': binary,
            'line_mask': binary
        }

    def draw_lines(self, image, result, draw_all=False):
        """Draw detection results on image."""
        output = image.copy()
        height = image.shape[0]
        width = image.shape[1]

        # Draw left line position
        if result['left_x'] is not None:
            x = int(result['left_x'])
            cv2.line(output, (x, 0), (x, height), (255, 0, 0), 2)

        # Draw right line position
        if result['right_x'] is not None:
            x = int(result['right_x'])
            cv2.line(output, (x, 0), (x, height), (0, 0, 255), 2)

        # Draw lane center
        if result['lane_center'] is not None:
            x = int(result['lane_center'])
            cv2.circle(output, (x, height - 30), 10, (0, 255, 0), -1)

            # Draw line from image center
            img_center = width // 2
            cv2.line(output, (img_center, height - 10), (x, height - 30), (0, 255, 255), 2)

        # Info text
        cv2.putText(output, f"CTE: {result['offset']:.3f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return output


# Legacy functions
def detect_lines(image):
    detector = LineDetector()
    result = detector.process_frame(image)
    return result['all_lines']

def find_center(lines):
    return 0.0
