#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
from datetime import datetime
from std_msgs.msg import String


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber_node')
        
        # Initialize robot parameters
        self.actual_vel_theta = 0
        self.actual_vel_x     = 0.0
        self.actual_vel_y     = 0.0
        
        #Subscribers
        self._mutex_cbg           = MutuallyExclusiveCallbackGroup()
        self.joint_position_sub   = self.create_subscription(Float64MultiArray, '/position_controller/commands', self.joint_position_cb,   10, callback_group=self._mutex_cbg)
        self.wheel_velocities_sub = self.create_subscription(Float64MultiArray, '/velocity_controller/commands', self.wheel_velocities_cb, 10, callback_group=self._mutex_cbg)
        self.goal_sub             = self.create_subscription(String,            '/goal_flag',                    self.goal_cb,             10, callback_group=self._mutex_cbg)
        
        # Initialize command methods and arrays to track robot    
        self.theta_vels = []
        self.right_vels = []
        self.left_vels  = []
        self.t_theta    = []
        self.t_vel      = []
        
    def joint_position_cb(self, msg: Float64MultiArray):
        """
        Subscriber c/b to steering commands
        """
        self.get_logger().info(f'Received Steering Command: {msg.data[0]}')
        self.theta_vels.append(msg.data[0])
        self.t_theta.append(datetime.now())
        
    def wheel_velocities_cb(self, msg: Float64MultiArray):
        """
        Subscriber c/b to wheel velocity commands
        """
        self.get_logger().info(f'Received Velocity Command [L, R]: {msg.data[0]}, {msg.data[1]}')
        self.left_vels.append(msg.data[0])
        self.right_vels.append(msg.data[1])
        self.t_vel.append(datetime.now())
        
    def goal_cb(self, msg: String):
        """
        Subscriber c/b to goal completion
        """
        self.get_logger().info(f'Received Goal Flag: {msg.data}')
        
        if "Control Loop Complete" in msg.data:
            self.plot_pose()
                    
    def plot_pose(self):
        """
        plot data regarding robot control commands
        """     
        # Plot commanded positions / velocities over time
        plt.figure()
        plt.subplot(1, 2, 1)
        plt.plot(self.t_theta, self.theta_vels, label='Commanded Steering Angle')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Angle (degrees)')
        plt.title('Commanded Steering Angle over Time')
    
        
        plt.subplot(1, 2, 2)
        plt.plot(self.t_vel, self.right_vels, label='Commanded RIGHT Wheel Velocity')
        plt.plot(self.t_vel, self.left_vels,  label='Commanded LEFT Wheel Velocity')    
        plt.xlabel('Time (seconds)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Commanded Wheel Velocities over Time')
        
        plt.legend()
        plt.grid()
        plt.savefig("part2_subsciber_pose_plot.png")
        plt.show()

       
def main(args=None):
    """
    Node operation - startup and shutdown, and plotting
    
    """
    # Start Node and Controller
    print("******************************************")
    print("Pose Subscriber Node Starting...")
    print("******************************************")

    rclpy.init(args=args)
    node = PoseSubscriber()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        
if __name__ == '__main__':
    main()