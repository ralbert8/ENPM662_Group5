#!/usr/bin/env python3

import os
import csv
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

package_share_dir = get_package_share_directory('project_two')

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')

        # Locate the CSV file
        try:
            csv_path = input_file = os.path.join(package_share_dir, 'csv', 'joint_angles.csv')
        except KeyError:
            raise FileNotFoundError("The 'project_two' package could not be found.")

        # Read joint angles from the CSV
        self.joint_angles = []
        try:
            with open(csv_path, mode='r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip the header row
                for row in reader:
                    # Convert row data to floats and append to joint_angles
                    self.joint_angles.append([float(value) for value in row])
        except Exception as e:
            self.get_logger().error(f"Error reading CSV file: {e}")
            rclpy.shutdown()
            return

        if not self.joint_angles:
            self.get_logger().error("No joint angles to publish. Shutting down.")
            rclpy.shutdown()
            return

        self.get_logger().info("Node initialized. Publishing joint angles every 0.1 seconds.")

        # Publisher setup
        self.publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Timer setup
        self.timer = self.create_timer(0.01, self.publish_joint_angles)

        self.current_index = 0

    def publish_joint_angles(self):
        if self.current_index >= len(self.joint_angles):
            self.get_logger().info("All joint angles published. Shutting down.")
            rclpy.shutdown()
            return

        # Modify columns 4 and 5 (index 3 and 4 in zero-based indexing) to be negative
        angles = self.joint_angles[self.current_index]

        # Create and publish the message
        msg = Float64MultiArray()
        msg.layout.data_offset = 0  # Ensure the data_offset is 0 (default)
        msg.data = angles
        self.publisher.publish(msg)

        self.get_logger().info(f"Published joint angles: {msg.data}")

        self.current_index += 1

def main():
    rclpy.init()
    node = JointAnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
