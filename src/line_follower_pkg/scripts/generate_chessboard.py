#!/usr/bin/env python3
"""
Generate a chessboard calibration pattern for camera calibration.
This script creates a chessboard image that can be printed and used
for camera calibration with OpenCV.

Usage:
    python3 generate_chessboard.py [--rows 6] [--cols 8] [--square_size 100] [--output chessboard.png]
"""

import argparse
import numpy as np
import cv2


def generate_chessboard(rows: int = 6, cols: int = 8, square_size: int = 100,
                        margin: int = 50) -> np.ndarray:
    """
    Generate a chessboard pattern image.

    Args:
        rows: Number of inner corners in the vertical direction (rows - 1 squares)
        cols: Number of inner corners in the horizontal direction (cols - 1 squares)
        square_size: Size of each square in pixels
        margin: White margin around the chessboard in pixels

    Returns:
        numpy array containing the chessboard image
    """
    # Total squares (not inner corners, but actual squares)
    num_rows = rows + 1
    num_cols = cols + 1

    # Calculate image dimensions
    board_width = num_cols * square_size
    board_height = num_rows * square_size
    img_width = board_width + 2 * margin
    img_height = board_height + 2 * margin

    # Create white image
    image = np.ones((img_height, img_width), dtype=np.uint8) * 255

    # Draw black squares
    for row in range(num_rows):
        for col in range(num_cols):
            # Alternate colors like a chessboard
            if (row + col) % 2 == 0:
                x1 = margin + col * square_size
                y1 = margin + row * square_size
                x2 = x1 + square_size
                y2 = y1 + square_size
                image[y1:y2, x1:x2] = 0  # Black

    return image


def main():
    parser = argparse.ArgumentParser(
        description='Generate a chessboard calibration pattern'
    )
    parser.add_argument(
        '--rows', type=int, default=6,
        help='Number of inner corners vertically (default: 6)'
    )
    parser.add_argument(
        '--cols', type=int, default=8,
        help='Number of inner corners horizontally (default: 8)'
    )
    parser.add_argument(
        '--square_size', type=int, default=100,
        help='Size of each square in pixels (default: 100)'
    )
    parser.add_argument(
        '--margin', type=int, default=50,
        help='White margin around the pattern in pixels (default: 50)'
    )
    parser.add_argument(
        '--output', type=str, default='chessboard.png',
        help='Output filename (default: chessboard.png)'
    )

    args = parser.parse_args()

    print(f"Generating chessboard pattern...")
    print(f"  Inner corners: {args.cols} x {args.rows}")
    print(f"  Squares: {args.cols + 1} x {args.rows + 1}")
    print(f"  Square size: {args.square_size} pixels")
    print(f"  Margin: {args.margin} pixels")

    # Generate the chessboard
    chessboard = generate_chessboard(
        rows=args.rows,
        cols=args.cols,
        square_size=args.square_size,
        margin=args.margin
    )

    # Save the image
    cv2.imwrite(args.output, chessboard)
    print(f"  Saved to: {args.output}")

    # Print calibration info
    print(f"\nCalibration info:")
    print(f"  Use CHESSBOARD_SIZE = ({args.cols}, {args.rows}) in OpenCV")
    print(f"  When printing, measure the actual square size in meters")
    print(f"  Example: if printed square is 2.5cm, use SQUARE_SIZE = 0.025")


if __name__ == '__main__':
    main()
