#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import numpy as np

import time
import matplotlib.pyplot as plt
import math
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion



class ProportionalControlNode(Node):

    def __init__(self):
        
        super().__init__('proportional_control_node')
        
        # Timer to send goals every 2 seconds (if list is not empty)
        self._timer = self.create_timer(1.0, self.timer_callback)
                            
        #Publishers
        self.joint_position_pub   = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub      = self.create_publisher(JointState, '/joint_states', 10) # TODO what is this for?
        
        # Subscriber to get odometry data
        # We are using a MutuallyExclusiveCallbackGroup to ensure
        # that the callback is executed in a separate thread from the main thread
        # qos_profile = QoSProfile(depth=10)
        self._odometry_cbg = MutuallyExclusiveCallbackGroup()
        self._odometry_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odometry_cb,
            10,
            callback_group=self._odometry_cbg,
        )

        # Initialize variables
        self._goal_x       = 10.0 * 39.37 
        self._goal_y       = 10.0 * 39.37 
        self._robot_x      = 0.0
        self._robot_y      = 0.0
        self._robot_yaw    = 0.0
        self._goal_reached = False
        
        # Initialize command params    
        self.joint_positions  = Float64MultiArray() #front-axle steering
        self.wheel_velocities = Float64MultiArray() #rear-wheel drive
        self.steer_angles     = []
        self.right_vels       = []
        self.left_vels        = []                

        # Proportional control gains
        self._kl = 0.5  # Linear velocity gain
        self._ka = 0.8  # Angular velocity gain
        
    def odometry_cb(self, msg: Odometry):
        """
        Callback to handle odometry data

        Args:
            msg (Odometry): Odometry message
        """
        
        # position
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        
        # orientation
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        
        # Convert quaternion to Euler angles
        _, _, self._robot_yaw = euler_from_quaternion(quaternion)

    def timer_callback(self):
        """
        Send a goal to the action server every 2 seconds
        """       
        print("timer")
        if self._goal_reached:
            self.get_logger().info("Goal already reached.")
            return 

        self.get_logger().info(f"Executing goal: ({self._goal_x},{self._goal_y})")
        # Create a rate object to control the loop rate
        rate = self.create_rate(10)

        while rclpy.ok():  # While ROS is running
            distance_to_goal = self.compute_distance_to_goal()

            if distance_to_goal < 0.02:
                self.get_logger().info("Goal reached!")
                self.stop_robot()
                self._goal_reached = True
                return 

            self.adjust_robot_motion(distance_to_goal)
            rate.sleep()
                        
    def compute_distance_to_goal(self):
        """
        Compute the distance to the goal

        Returns:
            float: Distance to the goal
        """
        return math.sqrt(
            (self._goal_x - self._robot_x) ** 2 + (self._goal_y - self._robot_y) ** 2
        )
                
    def calculate_angular_velocity(self, angle_to_goal):
        """
        Calculate the angular velocity based on the angle to the goal

        Args:
            angle_to_goal (float): Angle to the goal

        Returns:
            float: Angular velocity
        """
        # Adjust angle to ensure proper rotation direction
        if angle_to_goal < 0:
            angle_to_goal += 2 * math.pi

        # Compute relative orientation between robot and goal
        w = angle_to_goal - self._robot_yaw
        if w > math.pi:  # Ensure shortest rotation
            w -= 2 * math.pi
        elif w < -math.pi:  # Ensure shortest rotation
            w += 2 * math.pi

        # Proportional control for angular velocity, with limit for turtlebot max angular velocity
        angular_velocity = self._ka * w
        return max(min(angular_velocity, 1.82), -1.82)
        
    def stop_robot(self):
        """
        stop the robot
        """
        
        print("******************************************")
        print("Duration Complete. Stopping Robot...")
        print("******************************************")                        
        steer_angle         = 0.0
        lin_vel_right_wheel = 0.0
        lin_vel_left_wheel  = 0.0
        self.steer_angles.append(steer_angle)
        self.right_vels.append(lin_vel_right_wheel)
        self.left_vels.append(lin_vel_left_wheel)                
        print("Commanded Steering Angle: ",steer_angle)
        print("Commanded Right & Left Wheel Linear Velocity: ",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
        # Publish the twist message
        self.wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel]
        self.joint_positions.data  = [steer_angle,steer_angle]
        self.joint_position_pub.publish(self.joint_positions)
        self.wheel_velocities_pub.publish(self.wheel_velocities) 

    def adjust_robot_motion(self, distance_to_goal):
        """
        Adjust the robot motion based on the distance to the goal

        Args:
            distance_to_goal (float): Distance to the goal

        """
        dy = self._goal_y - self._robot_y
        dx = self._goal_x - self._robot_x
        # Compute angle to goal
        angle_to_goal = math.atan2(dy, dx)
        # Compute angular velocity
        angular_velocity = self.calculate_angular_velocity(angle_to_goal)
        # Proportional control for linear velocity
        linear_velocity = min(self._kl * distance_to_goal, 0.6)
        
        # Set car params [inches]
        L       = 38
        r_wheel = 8/2
        
        # kinematic equations
        # lin_vel_right_wheel = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) + (L / 2) * theta_dot_goal[i]))
        # lin_vel_left_wheel  = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) - (L / 2) * theta_dot_goal[i]))
        # steer_angle = np.arctan( L * theta_dot_goal[i] / math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2)   )
        
        # track commanded velocity & steering angle
        self.steer_angles.append(angular_velocity)  #(steer_angle)
        self.right_vels.append(linear_velocity)  #(lin_vel_right_wheel)
        self.left_vels.append(linear_velocity)  #(lin_vel_left_wheel)

        # print("Commanded Steering Angle: ",steer_angle)
        # print("Commanded Right & Left Wheel Linear Velocity: ",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
        # # Publish the twist message
        # wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel]
        # joint_positions.data  = [steer_angle,steer_angle]
        # self.joint_position_pub.publish(joint_positions)
        # self.wheel_velocities_pub.publish(wheel_velocities)
        
        print("Commanded Steering Angle: ",angular_velocity)
        print("Commanded Right & Left Wheel Linear Velocity: ",-linear_velocity,", ",linear_velocity)
        # Publish the twist message
        self.wheel_velocities.data = [-linear_velocity,linear_velocity]
        self.joint_positions.data  = [angular_velocity,angular_velocity]
        self.joint_position_pub.publish(self.joint_positions)
        self.wheel_velocities_pub.publish(self.wheel_velocities)        


        
    
    
    def prop_control(self):

        # Circular trajectory constraints
        r        = 10 * 39.37   # radius of circle [inches to meters conversion]
        duration = 30           # duration of motion [sec]
        dt       = 0.1          # sample interval [sec]

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
                    
        # Initialize command params    
        joint_positions  = Float64MultiArray() #front-axle steering
        wheel_velocities = Float64MultiArray() #rear-wheel drive
        steer_angles     = []
        right_vels       = []
        left_vels        = []
  
        # Set car params [inches]
        L       = 38
        r_wheel = 8/2
        
        i = 0
        while i < (duration / dt):
            
            time.sleep(dt)
            i += 1
            
            # kinematic equations
            lin_vel_right_wheel = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) + (L / 2) * theta_dot_goal[i]))
            lin_vel_left_wheel  = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) - (L / 2) * theta_dot_goal[i]))
            steer_angle = np.arctan( L * theta_dot_goal[i] / math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2)   )
            
            # track commanded velocity & steering angle
            steer_angles.append(steer_angle)
            right_vels.append(lin_vel_right_wheel)
            left_vels.append(lin_vel_left_wheel)

            # K_p = .5 proportional gain constant
            # x_pos_error = current_x_desired - imu_x_Pos
            # y_pos_error = current_y_desired - imu_y_Pos

            print("Commanded Steering Angle: ",steer_angle)
            print("Commanded Right & Left Wheel Linear Velocity: ",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
            # Publish the twist message
            wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel]
            joint_positions.data  = [steer_angle,steer_angle]
            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)

            if i >= (duration / dt):
                print("******************************************")
                print("Duration Complete. Stopping Robot...")
                print("******************************************")                
                steer_angle         = 0.0
                lin_vel_right_wheel = 0.0
                lin_vel_left_wheel  = 0.0
                steer_angles.append(steer_angle)
                right_vels.append(lin_vel_right_wheel)
                left_vels.append(lin_vel_left_wheel)                
                print("Commanded Steering Angle: ",steer_angle)
                print("Commanded Right & Left Wheel Linear Velocity: ",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
                # Publish the twist message
                wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel]
                joint_positions.data  = [steer_angle,steer_angle]
                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)            
        
        return steer_angles, right_vels, left_vels, x_goal, y_goal, t
            
def main(args=None):
    
    # Start Node and Controller
    print("hi")
    rclpy.init(args=args)
    node = ProportionalControlNode()
    #steer_angles, right_vels, left_vels, x_goal, y_goal, t = node.prop_control()
    
    # # Plot commanded positions / velocities over time
    # plt.figure()
    # plt.subplot(2, 2, 1)
    # plt.plot(t, steer_angles, label='Commanded Steering Angle')
    # plt.xlabel('Time (seconds)')
    # plt.ylabel('Angle (radians)')
   
    
    # plt.subplot(2, 2, 2)
    # plt.scatter(x_goal, y_goal, label='Car Goal Position')
    # plt.xlabel('X-coordinate (inches)')
    # plt.ylabel('Y-coordinate (inches)')
    
    # plt.subplot(2, 2, 3)    
    # plt.plot(t, right_vels, label='Commanded RIGHT Wheel Velocity')
    # plt.plot(t, left_vels, label='Commanded LEFT Wheel Velocity')    
    # plt.xlabel('Time (seconds)')
    # plt.ylabel('Linear Velocity (in/s)')     
        
    # plt.legend()
    # plt.grid()
    # plt.show()
    
    try:
        rclpy.spin(node) # spin the node
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()