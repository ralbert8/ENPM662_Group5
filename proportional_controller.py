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

        while rclpy.ok():  # While ROS is running
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
        return math.sqrt(
            (self._goal_x - self._robot_x) ** 2 + (self._goal_y - self._robot_y) ** 2
        )
        
    def adjust_robot_motion(self):
        
        
        
        # ODOM Postion error calculations
        print("odom x: ", self.actual_x)
        x_err = y_goal[i] - self.actual_x
        print("odom y: ", self.actual_y)
        y_err = y_goal[i] - self.actual_y
        lin_vel_err_mag = math.sqrt(x_err**2 + y_err**2)
        print("odom theta: ", self.actual_theta)
        theta_err = theta_goal[i] - self.actual_theta
        
        prop_control_lin_vel = k_p * lin_vel_err_mag 
        prop_control_ang_vel = k_p * theta_err
        
        # lin_vel_right_wheel = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) + (L / 2) * theta_dot_goal[i]))
        # lin_vel_left_wheel  = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) - (L / 2) * theta_dot_goal[i]))
        # replaced the derivatives with the proptional gain control terms
        lin_vel_right_wheel = float((1 / r_wheel) * (prop_control_lin_vel + (L / 2) * prop_control_ang_vel))
        lin_vel_left_wheel  = float((1 / r_wheel) * (prop_control_lin_vel - (L / 2) * prop_control_ang_vel))
                    
        linear_vel  = (1/2) * (  x_dot_goal[i]  / np.cos(theta_goal[i]) / (r_wheel/2)  )
        steer_angle = np.arctan( L * theta_dot_goal[i] / math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) )
        
        steer_angles.append(steer_angle)
        linear_vels1.append(lin_vel_right_wheel)
        linear_vels2.append(lin_vel_left_wheel)

        print("Steer Angle",steer_angle)
        print("Linear Velocity",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
        # Publish the twist message
        wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel] 
        joint_positions.data = [steer_angle,steer_angle] 

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        
        #car stops when within one car length of 10,10 gridspace
        # if abs(x_err) < L and abs(y_err) < L:
        #     break
        
    def prop_control(self):

        # Circular trajectory constraints
        r        = 10 * 39.37   # radius of circle [cm]
        duration = 30   # duration of motion [sec]
        dt       = 1    # sample interval [sec]

        # Calculate goal points to make circular trajectory
        # using polar coordinates to define eq of circle
        t              = [0]
        x_goal         = [0]
        y_goal         = [0]
        theta_goal     = [0]
        x_dot_goal     = [0]
        y_dot_goal     = [0]
        theta_dot_goal = [0]                
        i              = 0

        # gather goal points along trajectoy
        while i < duration:
            i += dt
            print("i: ",i)
            
            # Assume car will travel 1/4 circle --> 2pi = full circle --> 1/4 circle = pi/2
            theta      = (-(np.pi / 2)) + (np.pi / 2) * ( i     / duration) #integrated chunk of circle based on time - RADIANS
            theta_prev = (-(np.pi / 2)) + (np.pi / 2) * ((i-dt) / duration)
            
            t.append(i)
            theta_goal.append(theta)
            
            x = r * np.cos(theta)       #polar coords for eq of circle
            y = r * np.sin(theta) + r   #offset to start at 0,0 and travel to 10,10
            x_goal.append(x)   
            y_goal.append(y)
            
            theta_dot = (theta - theta_prev) / dt            
            theta_dot_goal.append(theta_dot)
            
            x_dot_goal.append(-y * theta_dot)
            y_dot_goal.append( x * theta_dot)
                    
        # Command the car    
        self.msg = "Closed Loop Proportion Controller"
        self.get_logger().info(self.msg)
        joint_positions  = Float64MultiArray()  #front-axle steering
        wheel_velocities = Float64MultiArray() #rear-wheel drive
        linear_vel       = 0.0 #initial value
        steer_angle      = 0.0 #initial value
        steer_angles     = [0]
        linear_vels1      = [0]
        linear_vels2      = [0]
        k_p = .2 #proportional gain
        x_err_list = [0]
        y_err_list = [0]
        theta_err_list = [0]
        # Car params [inches]
        L       = 38  
        r_wheel = 8/2
        
        i = 0
        while i < (duration / dt):
            

        print("Duration Complete. Stopping Robot")
        steer_angle=0.0
        linear_vel=0.0
        print("Steer Angle",steer_angle)
        print("Linear Velocity",linear_vel)
        
        # Publish the twist message
        wheel_velocities.data = [-linear_vel,linear_vel] # need to update with new variable names
        joint_positions.data = [steer_angle,steer_angle] # need to update with new variable names

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        
        return steer_angles, linear_vels1, linear_vels2, x_goal, y_goal, t
            
def main(args=None):
    # Start Node and Controller
    print("hi")
    rclpy.init(args=args)
    node = ProportionalControlNode()
    steer_angles, right_vels, left_vels, x_goal, y_goal, t = node.prop_control()
    
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
    

# #equations used from lecture
# x_dot = (r_wheel/2) * (lin_vel_left_wheel + lin_vel_right_wheel) * math.cos(theta) 
# y_dot = (r_wheel/2) * (lin_vel_left_wheel + lin_vel_right_wheel) * math.sin(theta)
# theta_dot = (r_wheel/L) * (lin_vel_right_wheel - lin_vel_left_wheel)