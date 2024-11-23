#!/usr/bin/env python3

import os
import csv
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time 

package_share_dir = get_package_share_directory('project_two')

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')        
            
        # Locate Package Share directory
        try:
            csv_path = input_file = os.path.join(package_share_dir, 'csv', 'joint_angles.csv')
        except KeyError:
            raise FileNotFoundError("The 'project_two' package could not be found.")
        
        # Wait for csv to appear in directory
        while not os.path.isfile(input_file):
            time.sleep(1)
            print("waiting for joint_angles.csv to appear in directory...")
            
        # Read CSV
        self.joint_angles = []
        try:
            with open(csv_path, mode='r') as file:
                reader = csv.reader(file)

                # Skip Header
                next(reader)
                for row in reader:

                    # Convert Rows to Float and Append to Joint Angle Storage
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

        # Create Publisher
        self.publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Create Timer
        self.timer = self.create_timer(0.01, self.publish_joint_angles)

        self.current_index = 0

    def publish_joint_angles(self):
        if self.current_index >= len(self.joint_angles):
            self.get_logger().info("All joint angles published. Shutting down.")
            rclpy.shutdown()
            return

        # Pull Current Angles from Storage
        angles = self.joint_angles[self.current_index]
        angles[3] = -angles[3]
        angles[4] = -angles[4]

        # Publish Data
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
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
