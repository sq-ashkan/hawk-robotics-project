"""
Line Detector for Real Robot - Multi-Height Scan

Detects white lines on dark floor using binary thresholding.
Scans at 6 heights for robust lane center estimation.
Uses paired centers (both lines visible at same height) for steering.

Floor: dark/blue
Lines: white
"""

import cv2
import numpy as np


# Output dimensions after crop+resize
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480


class RealLineDetector:
    """Line detector using binary threshold and multi-height scanning."""

    def __init__(self):
        # Binary threshold value (high for dark floor + white lines)
        self.threshold = 200

        # Noise removal: minimum connected component area
        self.min_component_area = 1000

        # Multi-height scan ratios (from top of image)
        # 6 heights from far (0.25) to near (0.75)
        self.scan_ratios = [0.25, 0.35, 0.45, 0.55, 0.65, 0.75]
        self.scan_rows = 5  # rows to scan around each height

        # Minimum pixels in a cluster to count as a line
        self.min_cluster_size = 3

        # Gap tolerance when clustering adjacent pixels
        self.cluster_gap = 5

        # Smoothing history
        self.center_history = []
        self.history_size = 5

        # Track lane half-width for single-line fallback
        self.estimated_half_width = 150

        # State
        self.detected = False
        self.last_center = None
        self.last_confidence = 0.0
        self.last_mode = 'no_lines'

        # Legacy compatibility
        self.crop_top = 0
        self.crop_bottom = 0

    def set_crop_region(self, crop_top, crop_bottom):
        """Legacy compatibility."""
        self.crop_top = crop_top
        self.crop_bottom = crop_bottom

    def preprocess(self, image):
        """
        Convert camera image to clean binary image.

        Pipeline: crop bottom 50% -> resize 640x480 ->
                  grayscale -> threshold -> morphology -> noise removal
        """
        h, w = image.shape[:2]

        # Crop: keep bottom 50% of image (floor area)
        cropped = image[int(h * 0.5):, :]

        # Resize to standard dimensions
        resized = cv2.resize(cropped, (OUTPUT_WIDTH, OUTPUT_HEIGHT))

        # Convert to grayscale and apply binary threshold
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)

        # Morphological erosion to remove thin noise streaks
        erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.erode(binary, erode_kernel, iterations=1)
        # Dilate back to restore line thickness
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.dilate(binary, dilate_kernel, iterations=1)

        # Remove small noise using connected components
        inv = cv2.bitwise_not(binary)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(inv)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] < self.min_component_area:
                binary[labels == i] = 255

        return binary

    def find_lines_at_region(self, binary, center_y, num_rows=5):
        """
        Scan multiple rows around center_y and find left/right line positions.

        Returns:
            (left_x, right_x) - averaged positions, or None if not found
        """
        half = num_rows // 2
        all_left = []
        all_right = []
        img_height = binary.shape[0]

        for dy in range(-half, half + 1):
            y = center_y + dy
            if y < 0 or y >= img_height:
                continue

            row = binary[y, :]
            white_cols = np.where(row > 200)[0]

            if len(white_cols) < self.min_cluster_size:
                continue

            # Cluster adjacent white pixels
            clusters = []
            current_cluster = [white_cols[0]]

            for i in range(1, len(white_cols)):
                if white_cols[i] - white_cols[i - 1] <= self.cluster_gap:
                    current_cluster.append(white_cols[i])
                else:
                    if len(current_cluster) >= self.min_cluster_size:
                        clusters.append(np.mean(current_cluster))
                    current_cluster = [white_cols[i]]

            if len(current_cluster) >= self.min_cluster_size:
                clusters.append(np.mean(current_cluster))

            # Assign clusters to left/right
            if len(clusters) >= 2:
                all_left.append(clusters[0])
                all_right.append(clusters[-1])
            elif len(clusters) == 1:
                if clusters[0] < OUTPUT_WIDTH / 2:
                    all_left.append(clusters[0])
                else:
                    all_right.append(clusters[0])

        left_x = float(np.mean(all_left)) if all_left else None
        right_x = float(np.mean(all_right)) if all_right else None
        return left_x, right_x

    def detect_lines(self, binary):
        """
        Scan at multiple heights for robust line detection.

        Returns:
            List of (ratio, y, left_x, right_x) tuples
        """
        h = binary.shape[0]
        scan_results = []

        for ratio in self.scan_ratios:
            y = int(h * ratio)
            left, right = self.find_lines_at_region(binary, y, self.scan_rows)
            scan_results.append((ratio, y, left, right))

        return scan_results

    def determine_steering(self, scan_results):
        """
        Determine steering from multi-height scan results.

        Strategy:
        1. Find all heights where BOTH lines are visible (paired centers)
        2. Use the NEAREST paired center as primary steering signal
        3. Blend with farthest paired center for gentle curve anticipation
        4. If no pairs, estimate center from single line + remembered width

        Returns:
            (offset, mode_string)
            offset: normalized [-1, 1], positive = turn right
        """
        center_x = OUTPUT_WIDTH / 2

        # Separate paired and single detections
        paired = []
        singles_left = []
        singles_right = []
        total_points = 0

        for ratio, y, left, right in scan_results:
            if left is not None:
                total_points += 1
            if right is not None:
                total_points += 1

            if left is not None and right is not None:
                center = (left + right) / 2
                width = right - left
                paired.append((ratio, center, width))
            else:
                if left is not None:
                    side = 'left' if left < center_x else 'right'
                    singles_left.append((ratio, left)) if side == 'left' \
                        else singles_right.append((ratio, left))
                if right is not None:
                    side = 'right' if right >= center_x else 'left'
                    singles_right.append((ratio, right)) if side == 'right' \
                        else singles_left.append((ratio, right))

        if total_points == 0:
            return 0.0, 'no_lines'

        # Update lane half-width from paired detections
        if paired:
            max_width = max(w for _, _, w in paired)
            self.estimated_half_width = max_width / 2

        # CASE 1: Have paired centers (best case)
        if paired:
            # Sort by ratio descending = nearest first
            paired.sort(key=lambda x: x[0], reverse=True)
            nearest_center = paired[0][1]

            if len(paired) >= 2:
                farthest_center = paired[-1][1]
                # 80% nearest (immediate) + 20% farthest (anticipation)
                lane_center = 0.8 * nearest_center + 0.2 * farthest_center
            else:
                lane_center = nearest_center

            offset = (lane_center - center_x) / center_x
            return float(np.clip(offset, -1.0, 1.0)), 'centered'

        # CASE 2: No pairs - points on both sides
        has_left = len(singles_left) > 0
        has_right = len(singles_right) > 0

        if has_left and has_right:
            # Use nearest point from each side
            singles_left.sort(key=lambda x: x[0], reverse=True)
            singles_right.sort(key=lambda x: x[0], reverse=True)
            left_x = singles_left[0][1]
            right_x = singles_right[0][1]
            lane_center = (left_x + right_x) / 2
            offset = (lane_center - center_x) / center_x
            return float(np.clip(offset, -1.0, 1.0)), 'estimated'

        # CASE 3: Single side only - use estimated half-width
        if has_right and not has_left:
            singles_right.sort(key=lambda x: x[0], reverse=True)
            nearest_x = singles_right[0][1]
            est_center = nearest_x - self.estimated_half_width
            offset = (est_center - center_x) / center_x
            return float(np.clip(offset, -0.8, 0.8)), 'follow_right'

        if has_left and not has_right:
            singles_left.sort(key=lambda x: x[0], reverse=True)
            nearest_x = singles_left[0][1]
            est_center = nearest_x + self.estimated_half_width
            offset = (est_center - center_x) / center_x
            return float(np.clip(offset, -0.8, 0.8)), 'follow_left'

        return 0.0, 'no_lines'

    def process_frame(self, image, current_time=0):
        """
        Process a frame: preprocess -> multi-height scan -> determine steering.

        Returns:
            dict with detection results (compatible with controller)
        """
        binary = self.preprocess(image)

        # Multi-height scan
        scan_results = self.detect_lines(binary)

        # Determine steering
        offset, mode = self.determine_steering(scan_results)

        # Count points and pairs
        total_points = 0
        paired_count = 0
        for _, _, left, right in scan_results:
            if left is not None:
                total_points += 1
            if right is not None:
                total_points += 1
            if left is not None and right is not None:
                paired_count += 1

        # Confidence based on paired scan heights
        if paired_count >= 4:
            confidence = 0.95
        elif paired_count >= 2:
            confidence = 0.8
        elif paired_count >= 1:
            confidence = 0.6
        elif total_points >= 2:
            confidence = 0.4
        elif total_points == 1:
            confidence = 0.2
        else:
            confidence = 0.0

        # Smooth offset with history
        if total_points > 0:
            self.center_history.append(offset)
            if len(self.center_history) > self.history_size:
                self.center_history.pop(0)
            smoothed_offset = float(np.mean(self.center_history))
        else:
            smoothed_offset = offset

        # Calculate curvature from paired centers
        curvature = float('inf')
        paired_centers = [(r, (l + ri) / 2) for r, y, l, ri in scan_results
                          if l is not None and ri is not None]
        if len(paired_centers) >= 2:
            paired_centers.sort()
            near_c = paired_centers[-1][1]
            far_c = paired_centers[0][1]
            diff = abs(far_c - near_c)
            if diff > 10:
                curvature = 500.0 / diff

        # Extract near/far points for backward compatibility
        near_left = None
        near_right = None
        far_left = None
        far_right = None

        for ratio, y, left, right in scan_results:
            if ratio >= 0.6:
                if left is not None and near_left is None:
                    near_left = left
                if right is not None and near_right is None:
                    near_right = right
            if ratio <= 0.4:
                if left is not None:
                    far_left = left
                if right is not None:
                    far_right = right

        lines_found = total_points > 0
        self.detected = lines_found
        self.last_confidence = confidence
        self.last_mode = mode

        # Lane center for debug drawing
        lane_center = None
        if near_left is not None and near_right is not None:
            lane_center = (near_left + near_right) / 2
        elif paired_centers:
            lane_center = paired_centers[-1][1]  # nearest paired center
        elif total_points > 0:
            all_x = [x for x in [near_left, near_right, far_left, far_right]
                     if x is not None]
            if all_x:
                lane_center = np.mean(all_x)

        return {
            'offset': smoothed_offset,
            'curvature': curvature,
            'lane_center': lane_center,
            'left_x': near_left,
            'right_x': near_right,
            'far_left_x': far_left,
            'far_right_x': far_right,
            'lines_found': lines_found,
            'confidence': confidence,
            'mode': mode,
            'num_points': total_points,
            'binary': binary,
            'scan_results': scan_results,
        }

    def draw_lines(self, image, result, draw_all=False):
        """
        Draw detection results on image for debugging.
        Shows all scan heights with color gradient (blue=far, red=near).
        Green dots mark paired lane centers at each height.
        """
        binary = result.get('binary')
        if binary is None:
            return image.copy()

        output = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        h, w = output.shape[:2]

        # Draw image center line
        cx = w // 2
        cv2.line(output, (cx, 0), (cx, h), (128, 128, 128), 1)

        # Draw all scan lines with color gradient
        scan_results = result.get('scan_results', [])
        n = len(scan_results)

        for i, (ratio, y, left, right) in enumerate(scan_results):
            # Color gradient: blue (far) -> red (near)
            t = i / max(1, n - 1)
            blue = int(255 * (1 - t))
            red = int(255 * t)
            color = (blue, 0, red)

            cv2.line(output, (0, y), (w, y), color, 1)

            if left is not None:
                cv2.circle(output, (int(left), y), 6, color, -1)
            if right is not None:
                cv2.circle(output, (int(right), y), 6, color, -1)

            # Green dot at paired center
            if left is not None and right is not None:
                center = int((left + right) / 2)
                cv2.circle(output, (center, y), 4, (0, 255, 0), -1)

        # Draw overall lane center
        if result.get('lane_center') is not None:
            cv2.circle(output, (int(result['lane_center']), h // 2),
                       10, (0, 255, 0), -1)

        # Text info
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_text = 20
        cv2.putText(output, f"Mode: {result.get('mode', '?')}",
                    (10, y_text), font, 0.6, (0, 255, 0), 2)
        y_text += 25
        cv2.putText(output, f"Offset: {result['offset']:.3f}",
                    (10, y_text), font, 0.6, (0, 255, 0), 2)
        y_text += 25

        # Count paired heights
        paired = sum(1 for _, _, l, r in scan_results
                     if l is not None and r is not None)
        cv2.putText(output, f"Pairs: {paired}/{n}",
                    (10, y_text), font, 0.6, (0, 255, 0), 2)
        y_text += 25
        cv2.putText(output, f"Conf: {result['confidence']:.2f}",
                    (10, y_text), font, 0.6, (0, 255, 0), 2)

        return output
