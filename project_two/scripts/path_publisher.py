#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as n

class FKSubscriber(Node):
    def __init__(self):
        super().__init__('fk_subscriber')

        # Subscriber to the joint angles command topic
        self.joint_angles_subscriber = self.create_subscription(
            Float64MultiArray,
            '/position_controller/commands',
            self.joint_angles_callback,
            10
        )

        # Publisher for the end-effector path
        self.path_publisher = self.create_publisher(Path, '/end_effector_path', 100)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.get_logger().info("FKSubscriber node initialized and listening for joint angles.")

    def forward_kinematics(self, joint_angles):
        joint_angles[3] = -joint_angles[3]
        joint_angles[4] = -joint_angles[4]

        # Define DH Table
        dh_parameters = [
            [joint_angles[0],          0.58365, 0,       -n.pi/2],
            [joint_angles[1] - n.pi/2, 0,       0.425,   0],
            [joint_angles[2],          0,       0.39225, 0],
            [joint_angles[3] + n.pi/2, 0.1099,  0,       n.pi/2],
            [joint_angles[4],          0.09065, 0,       -n.pi/2],
            [joint_angles[5],          0.2002,  0,       0]
        ]

        # Convert DH_i Parameters to A_i Matrix
        def dh_to_transform(theta, d, a, alpha):
            A = n.matrix([
                [n.cos(theta), -n.sin(theta) * n.cos(alpha), n.sin(theta) * n.sin(alpha),  a * n.cos(theta)],
                [n.sin(theta), n.cos(theta) * n.cos(alpha),  -n.cos(theta) * n.sin(alpha), a * n.sin(theta)],
                [0,            n.sin(alpha),                 n.cos(alpha),                 d],
                [0,            0,                            0,                            1]
            ])
            return A

        # Initialize Storage for Matrices
        A_matrices = []
        T_base = []

        # Compute A_i Matrices
        for params in dh_parameters:
            A_i = dh_to_transform(*params)
            A_matrices.append(A_i)

        T_prev = n.eye(4)
        for A in A_matrices:
            T_prev = T_prev * A
            T_base.append(T_prev)

        final_transformation = T_base[-1]

        x = final_transformation[0, 3]
        y = final_transformation[1, 3]
        z = final_transformation[2, 3]

        return x, y, z

    def joint_angles_callback(self, msg):
        """
        Callback for processing joint angles and publishing the end-effector position.
        """
        joint_angles = msg.data
        self.get_logger().info(f"Received joint angles: {joint_angles}")

        # Perform forward kinematics
        x, y, z = self.forward_kinematics(joint_angles)

        # Create a PoseStamped message for the end-effector position
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # Default orientation

        # Add pose to the path message and publish
        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        self.path_publisher.publish(self.path_msg)

        self.get_logger().info(f"Published end-effector position: x={x}, y={y}, z={z}")


def main():
    rclpy.init()
    node = FKSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
