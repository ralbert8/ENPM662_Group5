#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import sys
import select
import tty
import termios
from pynput import keyboard
import numpy as np
import sympy as sp
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import matplotlib.pyplot as plt
import math


class ProportionalControlNode(Node):

    def __init__(self):
        super().__init__('proportional_control_node')
        
        #Publishers
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        #Subscriber for ODOM data
        self._mutex_cbg = MutuallyExclusiveCallbackGroup()
        self.odom_sub = self.create_subscription(PoseStamped, '/odom', self.odom_callback, 10, callback_group=self._mutex_cbg)
        self.odom_sub = self.create_subscription(Twist, '/velocity', self.velocity_callback, 10, callback_group=self._mutex_cbg)
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_theta = 0.0 
        self.actual_vel_x = 0.0
        self.actual_vel_y = 0.0
        
        #timer
        self._goal_x       = 10.0 * 39.37 
        self._goal_y       = 10.0 * 39.37
        self._goal_theta   = math.atan2(self._goal_x, self._goal_y)
        
        self._goal_reached = False
        self._timer = self.create_timer(0.3, self.timer_callback)
        

    def odom_callback(self, msg: PoseStamped):
        # self.get_logger().info(f"Received ODOM data: {msg.data}")
        print("callback x position", msg.pose.position.x)
        self.actual_x = msg.pose.position.x
        self.actual_y = msg.pose.position.y
        ort = msg.pose.orientation   # orentation message
        #quaternion conversion to get yaw angle of car
        self.actual_theta = math.atan2(2 * (ort.w * ort.z + ort.x * ort.y), 1 - 2 * (ort.y**2 + ort.z**2)) 
        
    def velocity_callback(self, msg: Twist):
        self.actual_vel_x = msg.linear.x
        self.actual_vel_y = msg.linear.y
    
    def timer_callback(self):
        """
        Send a goal to the action server every X seconds
        """       
        print("timer")
        if self._goal_reached:
            self.get_logger().info("Goal already reached.")
            return 

        self.get_logger().info(f"Executing goal: ({self._goal_x},{self._goal_y})")

    
        distance_to_goal = self.compute_distance_to_goal()
        print("Distance to goal: ",distance_to_goal)
        if distance_to_goal < 39.37:
            self.get_logger().info("Goal reached!")
            self.stop_robot()
            self._goal_reached = True
            return 

        self.adjust_robot_motion(distance_to_goal)
        
    def compute_distance_to_goal(self):
        """
        Compute the distance to the goal

        Returns:
            float: Distance to the goal
        """
        return math.sqrt((self._goal_x - self.actual_x) ** 2 + (self._goal_y - self.actual_y) ** 2)
        
    def adjust_robot_motion(self, distance_to_goal):
        steer_limit = 75 # degress
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        # Car params [inches]
        L       = 38  
        r_wheel = 8/2
        k_p = .2
        # ODOM Postion error calculations
        print("odom x: ", self.actual_x)
        x_err = self._goal_x - self.actual_x
        print("odom y: ", self.actual_y)
        y_err = self._goal_y - self.actual_y
        # lin_vel_err_mag = math.sqrt(x_err**2 + y_err**2)
        print("odom theta: ", self.actual_theta)
        theta_err = math.atan2(y_err,x_err) - self.actual_theta
        
        prop_control_lin_vel = k_p * distance_to_goal 
        prop_control_ang_vel = .6 * theta_err
        
        # lin_vel_right_wheel = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) + (L / 2) * theta_dot_goal[i]))
        # lin_vel_left_wheel  = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) - (L / 2) * theta_dot_goal[i]))
        # replaced the derivatives with the proptional gain control terms
        lin_vel_right_wheel = float((1 / r_wheel) * (prop_control_lin_vel + (L / 2) * prop_control_ang_vel))
        lin_vel_left_wheel  = float((1 / r_wheel) * (prop_control_lin_vel - (L / 2) * prop_control_ang_vel))
                    
        # linear_vel  = (1/2) * (  x_dot_goal[i]  / np.cos(theta_goal[i]) / (r_wheel/2)  )
        # steer_angle = np.arctan( L * theta_dot_goal[i] / math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) )
        steer_angle = math.atan2(2*L*math.sin(theta_err), distance_to_goal)
        if steer_angle> math.radians(steer_limit):
            steer_angle=math.radians(steer_limit)
        if steer_angle<-math.radians(steer_limit):
            steer_angle=-math.radians(steer_limit)
            
        # steer_angles.append(steer_angle)
        # linear_vels1.append(lin_vel_right_wheel)
        # linear_vels2.append(lin_vel_left_wheel)
        # print("Steer Angle",steer_angle)
        # print("Linear Velocity",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
        # Publish the twist message
        wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel] 
        joint_positions.data = [steer_angle,steer_angle] 

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        
        
    def stop_robot(self):
        """
        Stop the robot by setting velocities and angles to zero
        """
        self.joint_position_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        self.wheel_velocities_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        
def main(args=None):
    # Start Node and Controller
    print("hi")
    rclpy.init(args=args)
    node = ProportionalControlNode()
    # steer_angles, right_vels, left_vels, x_goal, y_goal, t = node.prop_control()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        # clean up the node
        rclpy.shutdown()
       
    # rclpy.init(args=args)
    # node = ProportionalControlNode()
    # steer_angles, linear_vels1, linear_vels2, x_goal, y_goal, t = node.prop_control()
    
    # plt.figure()
    # plt.subplot(2, 2, 1)
    # plt.plot(t, steer_angles, label='commanded steer_angle')
    # plt.xlabel('Time (seconds)')
    # plt.ylabel('Angle (radians)')
   
    
    # plt.subplot(2, 2, 2)
    # plt.scatter(x_goal, y_goal, label='car goal position')
    # plt.xlabel('X-coordinate (inches)')
    # plt.ylabel('Y-coordinate (inches)')
    
    # plt.subplot(2, 2, 3)    
    # plt.plot(t, linear_vels1, label='commanded RIGHT linear_vel')
    # plt.xlabel('Time (seconds)')
    # plt.ylabel('Velocity (in/s)')     
    
    # plt.subplot(2, 2, 4)    
    # plt.plot(t, linear_vels2, label='commanded LEFT linear_vel')
    # plt.xlabel('Time (seconds)')
    # plt.ylabel('Velocity (in/s)')  
        
    # plt.legend()
    # plt.grid()
    # plt.show()
    
    # node.destroy_node()
    # rclpy.shutdown()
    
   

if __name__ == '__main__':
    main()
    

