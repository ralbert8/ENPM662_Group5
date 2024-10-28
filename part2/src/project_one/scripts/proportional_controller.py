#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

import sys
import select
import tty
import termios
from pynput import keyboard
import numpy as np
import sympy as sp

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
        
        #Subscriber for IMU data, need to sort out position data for closed loop control
        self.imu_sub = self.create_subscription(Imu, '/imu/out', self.subscriber_callback, 10)

        # self.settings = termios.tcgetattr(sys.stdin)

    def subscriber_callback(self, msg: Imu):
        # process incoming messages
        self.get_logger().info(f"Received IMU data: {msg.data}")
        
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
  
        # Car params [inches]
        L       = 38  
        r_wheel = 8/2
        
        i = 0
        while i < (duration / dt):
            
            time.sleep(dt)
            i += 1
            
            
            lin_vel_right_wheel = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) + (L / 2) * theta_dot_goal[i]))
            lin_vel_left_wheel  = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) - (L / 2) * theta_dot_goal[i]))
            
                        
            linear_vel  = (1/2) * (  x_dot_goal[i]  / np.cos(theta_goal[i]) / (r_wheel/2)  )
            steer_angle = np.arctan( L * theta_dot_goal[i] / math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2)   )
            
            steer_angles.append(steer_angle)
            linear_vels1.append(lin_vel_right_wheel)
            linear_vels2.append(lin_vel_left_wheel)
            
            #kinematic equations
            # x_dot = (r_wheel/2) * (lin_vel_left_wheel + lin_vel_right_wheel) * math.cos(theta) 
            # y_dot = (r_wheel/2) * (lin_vel_left_wheel + lin_vel_right_wheel) * math.sin(theta)
            # theta_dot = (r_wheel/L) * (lin_vel_right_wheel - lin_vel_left_wheel)

            # K_p = .5 proportional gain constant
            # x_pos_error = current_x_desired - imu_x_Pos
            # y_pos_error = current_y_desired - imu_y_Pos
            
            # I think we need solve kinematic equations for wheel velocties and theta, then publish as commands, incorperate gain



            print("Steer Angle",steer_angle)
            print("Linear Velocity",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
            # Publish the twist message
            wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel] # need to update with new variable names
            joint_positions.data = [steer_angle,steer_angle] # need to update with new variable names

            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)





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
    rclpy.init(args=args)
    node = ProportionalControlNode()
    steer_angles, linear_vels1, linear_vels2, x_goal, y_goal, t = node.prop_control()
    
    plt.figure()
    plt.subplot(2, 2, 1)
    plt.plot(t, steer_angles, label='commanded steer_angle')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Angle (radians)')
   
    
    plt.subplot(2, 2, 2)
    plt.scatter(x_goal, y_goal, label='car goal position')
    plt.xlabel('X-coordinate (inches)')
    plt.ylabel('Y-coordinate (inches)')
    
    plt.subplot(2, 2, 3)    
    plt.plot(t, linear_vels1, label='commanded RIGHT linear_vel')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity (in/s)')     
    
    plt.subplot(2, 2, 4)    
    plt.plot(t, linear_vels2, label='commanded LEFT linear_vel')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity (in/s)')  
        
    plt.legend()
    plt.grid()
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()
    

# #equations used from lecture
# x_dot = (r_wheel/2) * (lin_vel_left_wheel + lin_vel_right_wheel) * math.cos(theta) 
# y_dot = (r_wheel/2) * (lin_vel_left_wheel + lin_vel_right_wheel) * math.sin(theta)
# theta_dot = (r_wheel/L) * (lin_vel_right_wheel - lin_vel_left_wheel)
