#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import math
from datetime import datetime

class ProportionalControlNode(Node):

    def __init__(self):
        super().__init__('proportional_control_node')
        
        # Initialize robot parameters
        self.actual_x     = 0.0
        self.actual_y     = 0.0
        self.actual_theta = 0
        self.actual_vel_x = 0.0
        self.actual_vel_y = 0.0
        self.k_p          = 0.5
        self.k_t          = 1 
        self._goal_x      = 10.0 
        self._goal_y      = 10.0
        self._goal_reached = False
        
        #Publishers
        self.joint_position_pub   = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub      = self.create_publisher(JointState,        '/joint_states', 10)
        
        #Subscriber for odom data
        self._mutex_cbg = MutuallyExclusiveCallbackGroup()
        self.odom_sub   = self.create_subscription(PoseStamped, '/odom', self.odom_callback, 10, callback_group=self._mutex_cbg)
        self.odom_sub   = self.create_subscription(Twist,   '/velocity', self.velocity_callback, 10, callback_group=self._mutex_cbg)

        # Initialize command methods and arrays to track robot    
        self.joint_positions  = Float64MultiArray() #front-axle steering
        self.wheel_velocities = Float64MultiArray() #rear-wheel drive
        self.steer_angles     = []
        self.right_vels       = []
        self.left_vels        = []  
        self.x_vals           = []
        self.y_vals           = []
        self.t                = []      
        
        #timer
        self._timer = self.create_timer(1, self.timer_callback)

    def odom_callback(self, msg: PoseStamped):
        
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
        Send a linear/steering velocity to the robot every X seconds
        """       
        print("\n")
        
        if self._goal_reached:
            self.get_logger().info("Goal already reached. Press ctrl+c to exit and plot.")
            return

        self.get_logger().info(f"Executing goal: ({self._goal_x},{self._goal_y})")

        while rclpy.ok():  # While ROS is running
            
            distance_to_goal = math.sqrt((self._goal_x - self.actual_x) ** 2 + (self._goal_y - self.actual_y) ** 2)
            print("Distance to goal: ",distance_to_goal)
            
            if distance_to_goal < 2:
                self.stop_robot()
                return
            elif self.actual_x > self._goal_x + 2 or \
                 self.actual_y > self._goal_y + 2:
                self.stop_robot()
                return
            else:
                self.x_vals.append(self.actual_x)
                self.y_vals.append(self.actual_y)
                self.adjust_robot_motion()
        
    def compute_distance_to_goal(self):
        """
        Compute the distance to the goal

        Returns:
            float: Distance to the goal
        """        
        dist = math.sqrt(
            (self._goal_x - self.actual_x) ** 2 + (self._goal_y - self.actual_y) ** 2
        )
        max_wheel_velocity = 10
        
        return min(dist, max_wheel_velocity)
        
    def calculate_angular_velocity(self, angle_to_goal):
        """
        Calculate the angular velocity based on the angle to the goal

        Args:
            angle_to_goal (float): Angle to the goal

        Returns:
            float: Angular velocity
        """
        max_steering_angle = np.radians(30)
        
        # Adjust angle to ensure proper rotation direction
        if angle_to_goal < 0:
            angle_to_goal += 2 * math.pi

        # Compute relative orientation between robot and goal
        w = angle_to_goal - self.actual_theta
        
        if w > math.pi:  # Ensure shortest rotation
            w -= 2 * math.pi
        elif w < -math.pi:  # Ensure shortest rotation
            w += 2 * math.pi

        # Proportional control for angular velocity, with limit for turtlebot max angular velocity
        angular_velocity = self.k_p * w
        
        if angular_velocity > 0 and angular_velocity > max_steering_angle:
            angular_velocity = max_steering_angle
        elif angular_velocity < 0 and abs(angular_velocity) > max_steering_angle:
            angular_velocity = -max_steering_angle
            
        return angular_velocity
    
    def stop_robot(self):
        """
        stop the robot
        """
        
        self._goal_reached=True
        
        print("******************************************")
        print("Control Loop Complete. Stopping Robot...")
        print("******************************************")                        
        steer_angle         = 0.0
        lin_vel_right_wheel = 0.0
        lin_vel_left_wheel  = 0.0
        self.t.append(datetime.now())
        self.steer_angles.append(np.degrees(steer_angle))
        self.right_vels.append(lin_vel_right_wheel)
        self.left_vels.append(lin_vel_left_wheel)                
        print("Commanded Steering Angle: ",steer_angle)
        print("Commanded Right & Left Wheel Velocity: ",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
        
        # Publish the stop commands
        self.wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel]
        self.joint_positions.data  = [steer_angle,steer_angle]
        self.joint_position_pub.publish(self.joint_positions)
        self.wheel_velocities_pub.publish(self.wheel_velocities)
                     
    def adjust_robot_motion(self):  
        """
        make incremental adjustments to robot velocity and heading
        """              
                
        # ODOM Postion error calculations
        print("Robot Position X: ", self.actual_x)
        print("Robot Position Y: ", self.actual_y)
        lin_vel_err_mag = self.compute_distance_to_goal()
        
        print("Robot Heading Angle [degrees]: ", np.degrees(self.actual_theta))
        dy= self._goal_y - self.actual_y
        dx= self._goal_x - self.actual_x
        angle_to_goal = math.atan2(dx, dy)
        print("Angle to Goal from Current Position [degrees]: ",np.degrees(angle_to_goal))
        theta_err = angle_to_goal- self.actual_theta
        print("Heading Error [degrees]: ",np.degrees(theta_err))
        steer_angle_vel = self.calculate_angular_velocity(theta_err)
        
        prop_control_lin_vel = self.k_p * lin_vel_err_mag 
        prop_control_ang_vel = self.k_t * steer_angle_vel
                
        self.t.append(datetime.now())
        self.steer_angles.append(np.degrees(prop_control_ang_vel))
        self.right_vels.append(prop_control_lin_vel)
        self.left_vels.append(prop_control_lin_vel)

        print("Commanded Steering Angle: ",np.degrees(prop_control_ang_vel))
        print("Commanded Right & Left Wheel Velocity: ",-prop_control_lin_vel,", ",prop_control_lin_vel)
        # Publish the command
        self.wheel_velocities.data = [-prop_control_lin_vel,prop_control_lin_vel] 
        self.joint_positions.data  = [prop_control_ang_vel,prop_control_ang_vel] 

        self.joint_position_pub.publish(self.joint_positions)
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        
    def plot_pose(self):
                    
        # Plot commanded positions / velocities over time
        plt.figure()
        plt.subplot(2, 2, 1)
        plt.plot(self.t, self.steer_angles, label='Commanded Steering Angle')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Angle (degrees)')
        plt.title('Commanded Steering Angle over Time')
    
        
        plt.subplot(2, 2, 2)
        plt.scatter(self.x_vals, self.y_vals, label='Car X-Y Positions')
        plt.xlabel('X-coordinate (meters)')
        plt.ylabel('Y-coordinate (meters)')
        plt.title('Robot Position over Time')
        
        plt.subplot(2, 2, 3)    
        plt.plot(self.t, self.right_vels, label='Commanded RIGHT Wheel Velocity')
        plt.plot(self.t, self.left_vels,  label='Commanded LEFT Wheel Velocity')    
        plt.xlabel('Time (seconds)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Commanded Wheel Velocities over Time')  
            
        plt.legend()
        plt.grid()
        plt.show()
        plt.savefig("./part2_pose_plot.png")
            
       
def main(args=None):
    
    # Start Node and Controller
    print("******************************************")
    print("Proportional Controller Node Starting...")
    print("******************************************")

    rclpy.init(args=args)
    node = ProportionalControlNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
        node.plot_pose()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()