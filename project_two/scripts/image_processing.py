#!/usr/bin/env python3

import cv2
import numpy as np
import csv
import os
from ament_index_python.packages import get_package_share_directory

def extract_and_scale_contours_yz(package_name, relative_csv_path, relative_image_path, plot_width_m=0.3, plot_height_m=0.3, x_value=0.5):
    """
    Extracts the outline of objects in an image, scales the contours to fit within a specified
    plot size in the YZ plane, and saves the scaled contour points to a CSV file inside a ROS 2 package.

    Parameters:
        package_name (str): Name of the ROS 2 package where the image and CSV will be accessed/saved.
        relative_csv_path (str): Relative path (from the package share directory) to save the CSV file.
        relative_image_path (str): Relative path (from the package share directory) to the input image.
        plot_width_m (float): Width of the plot in mm (Y-axis range).
        plot_height_m (float): Height of the plot in mm (Z-axis range).
        x_value (float): Constant X-value for all points in the YZ plane.
    """
    try:
        # Get the package path
        package_path = get_package_share_directory(package_name)
    except KeyError:
        print(f"Error: Could not find ROS 2 package '{package_name}'.")
        return

    # Construct the full paths for the image and CSV
    image_path = os.path.join(package_path, relative_image_path)
    output_csv_path = os.path.join(package_path, relative_csv_path)

    # Ensure the directory for the CSV exists
    os.makedirs(os.path.dirname(output_csv_path), exist_ok=True)

    # Load the image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Error: Could not load the image at {image_path}.")
        return

    # Threshold the image to create a binary image
    _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)

    # Find contours
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        print("No contours found in the image.")
        return

    # Combine all contours into one array for bounding box calculation
    all_points = np.vstack([contour[:, 0, :] for contour in contours])

    # Calculate the bounding box of all points
    y_min, z_min = np.min(all_points, axis=0)
    y_max, z_max = np.max(all_points, axis=0)
    width = y_max - y_min
    height = z_max - z_min

    if width == 0 or height == 0:
        print("Contours have no area. Scaling skipped.")
        return

    # Scale factor for normalization and to fit within the plot
    scale_y = plot_width_m / width
    scale_z = plot_height_m / height
    scale = min(scale_y, scale_z)  # Maintain aspect ratio

    # Normalize and scale the contours
    scaled_contours = []
    for contour in contours:
        for point in contour:
            y, z = point[0]
            y_normalized = (y - y_min) * scale
            z_normalized = (z - z_min) * scale + 0.5
            scaled_contours.append((x_value, y_normalized, z_normalized))

    # Write scaled contour points to CSV
    with open(output_csv_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["X", "Y_mm", "Z_mm"])  # Header
        writer.writerows(scaled_contours)

    print(f"Scaled contour points saved to {output_csv_path}")


# Example usage
if __name__ == "__main__":
    package_name = "project_two"  # Replace with the actual ROS 2 package name
    relative_csv_path = "csv/contours.csv"  # Relative path within the package to save the CSV
    relative_image_path = "images/fish2.png"  # Relative path within the package to the image
    extract_and_scale_contours_yz(package_name, relative_csv_path, relative_image_path)
