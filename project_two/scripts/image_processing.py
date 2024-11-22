#!/usr/bin/env python3

import cv2
import numpy as np
import csv
import os
from ament_index_python.packages import get_package_share_directory

def extract_and_scale_contours_yz(package_name, relative_csv_path, relative_image_path, plot_width_m=0.3, plot_height_m=0.3, x_value=0.5):

    try:
        # Get Package Path
        package_path = get_package_share_directory(package_name)
    except KeyError:
        # Error Handling
        print(f"Error: Could not find ROS 2 package '{package_name}'.")
        return

    # Construct Full Paths for Image and CSV
    image_path = os.path.join(package_path, relative_image_path)
    output_csv_path = os.path.join(package_path, relative_csv_path)

    # Ensure CSV Path Exists
    os.makedirs(os.path.dirname(output_csv_path), exist_ok=True)

    # Load Image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Error: Could not load the image at {image_path}.")
        return

    # Threshold Image
    _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)

    # Find Contours
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Error Handling
    if not contours:
        print("No contours found in the image.")
        return

    # Combine Contours into Array for Bounding Box
    all_points = np.vstack([contour[:, 0, :] for contour in contours])

    # Calculate Bounding Box of Image
    y_min, z_min = np.min(all_points, axis=0)
    y_max, z_max = np.max(all_points, axis=0)
    width = y_max - y_min
    height = z_max - z_min

    # Error Handling
    if width == 0 or height == 0:
        print("Contours have no area. Scaling skipped.")
        return

    # Determine Image Scale
    scale_y = plot_width_m / width
    scale_z = plot_height_m / height
    scale = min(scale_y, scale_z)

    # Normalize and Scale Contours
    scaled_contours = []
    for contour in contours:
        for point in contour:
            y, z = point[0]
            y_normalized = (y - y_min) * scale
            z_normalized = (z - z_min) * scale + 1.0
            scaled_contours.append((x_value, y_normalized, z_normalized))

    # Write Contours to CSV
    with open(output_csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["X", "Y_mm", "Z_mm"])  # Header
        writer.writerows(scaled_contours)

    print(f"Scaled contour points saved to {output_csv_path}")

if __name__ == "__main__":
    package_name = "project_two"
    relative_csv_path = "csv/contours.csv"
    relative_image_path = "images/fish2.png"
    extract_and_scale_contours_yz(package_name, relative_csv_path, relative_image_path)
