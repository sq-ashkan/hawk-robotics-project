#!/usr/bin/env python3
"""
Generate a track texture with white lines on dark background.
This texture will be used as ground plane in Gazebo.
"""

import cv2
import numpy as np
import os

# Track parameters
TEXTURE_SIZE = 2048  # pixels (will cover 10m x 10m in simulation)
METERS_PER_PIXEL = 10.0 / TEXTURE_SIZE  # 10 meters total

# Line parameters (in meters)
ROAD_WIDTH = 0.5  # distance between two white lines
LINE_WIDTH = 0.05  # white line width (5cm)

# Colors (BGR)
GROUND_COLOR = (40, 40, 40)  # dark gray/asphalt
LINE_COLOR = (255, 255, 255)  # white

def meters_to_pixels(meters):
    return int(meters / METERS_PER_PIXEL)

def create_track_texture():
    # Create dark ground
    img = np.full((TEXTURE_SIZE, TEXTURE_SIZE, 3), GROUND_COLOR, dtype=np.uint8)

    center = TEXTURE_SIZE // 2

    # Track dimensions in pixels
    road_width_px = meters_to_pixels(ROAD_WIDTH)
    line_width_px = max(meters_to_pixels(LINE_WIDTH), 3)  # minimum 3 pixels

    # Rectangular track parameters
    track_length = meters_to_pixels(6.0)  # 6 meters long
    track_width = meters_to_pixels(3.0)   # 3 meters wide

    # Calculate track corners
    left = center - track_length // 2
    right = center + track_length // 2
    top = center - track_width // 2
    bottom = center + track_width // 2

    # Inner and outer line offsets
    inner_offset = road_width_px // 2
    outer_offset = road_width_px // 2

    # Draw outer rectangle (outer white line)
    # Top line
    cv2.rectangle(img,
                  (left - outer_offset, top - outer_offset - line_width_px),
                  (right + outer_offset, top - outer_offset),
                  LINE_COLOR, -1)
    # Bottom line
    cv2.rectangle(img,
                  (left - outer_offset, bottom + outer_offset),
                  (right + outer_offset, bottom + outer_offset + line_width_px),
                  LINE_COLOR, -1)
    # Left line
    cv2.rectangle(img,
                  (left - outer_offset - line_width_px, top - outer_offset),
                  (left - outer_offset, bottom + outer_offset),
                  LINE_COLOR, -1)
    # Right line
    cv2.rectangle(img,
                  (right + outer_offset, top - outer_offset),
                  (right + outer_offset + line_width_px, bottom + outer_offset),
                  LINE_COLOR, -1)

    # Draw inner rectangle (inner white line)
    # Top line
    cv2.rectangle(img,
                  (left + inner_offset, top + inner_offset),
                  (right - inner_offset, top + inner_offset + line_width_px),
                  LINE_COLOR, -1)
    # Bottom line
    cv2.rectangle(img,
                  (left + inner_offset, bottom - inner_offset - line_width_px),
                  (right - inner_offset, bottom - inner_offset),
                  LINE_COLOR, -1)
    # Left line
    cv2.rectangle(img,
                  (left + inner_offset, top + inner_offset),
                  (left + inner_offset + line_width_px, bottom - inner_offset),
                  LINE_COLOR, -1)
    # Right line
    cv2.rectangle(img,
                  (right - inner_offset - line_width_px, top + inner_offset),
                  (right - inner_offset, bottom - inner_offset),
                  LINE_COLOR, -1)

    # Add start/finish marker (yellow line)
    start_x = center
    cv2.rectangle(img,
                  (start_x - line_width_px, bottom - inner_offset - line_width_px),
                  (start_x + line_width_px, bottom + outer_offset + line_width_px),
                  (0, 200, 255), -1)  # Yellow/orange

    return img

def create_simple_straight_track():
    """Create a simple straight track for initial testing."""
    img = np.full((TEXTURE_SIZE, TEXTURE_SIZE, 3), GROUND_COLOR, dtype=np.uint8)

    center = TEXTURE_SIZE // 2
    road_width_px = meters_to_pixels(ROAD_WIDTH)
    line_width_px = max(meters_to_pixels(LINE_WIDTH), 5)

    # Two parallel white lines going from bottom to top
    left_line_x = center - road_width_px // 2
    right_line_x = center + road_width_px // 2

    # Draw left line
    cv2.rectangle(img,
                  (left_line_x - line_width_px // 2, 0),
                  (left_line_x + line_width_px // 2, TEXTURE_SIZE),
                  LINE_COLOR, -1)

    # Draw right line
    cv2.rectangle(img,
                  (right_line_x - line_width_px // 2, 0),
                  (right_line_x + line_width_px // 2, TEXTURE_SIZE),
                  LINE_COLOR, -1)

    return img

if __name__ == '__main__':
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    textures_dir = os.path.join(script_dir, '..', 'worlds', 'textures')
    os.makedirs(textures_dir, exist_ok=True)

    # Generate rectangular track
    track_img = create_track_texture()
    track_path = os.path.join(textures_dir, 'track_texture.png')
    cv2.imwrite(track_path, track_img)
    print(f"Track texture saved to: {track_path}")

    # Generate simple straight track
    straight_img = create_simple_straight_track()
    straight_path = os.path.join(textures_dir, 'straight_track.png')
    cv2.imwrite(straight_path, straight_img)
    print(f"Straight track texture saved to: {straight_path}")

    print("\nTexture size: {}x{} pixels".format(TEXTURE_SIZE, TEXTURE_SIZE))
    print("Covers: 10m x 10m in simulation")
