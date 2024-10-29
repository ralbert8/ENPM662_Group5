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
from tf_transformations import euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

class ProportionalControlNode(Node):

    def __init__(self):
        
        super().__init__('proportional_control_node')
        
        # Initialize variables
        self._goal_x       = 10.0 * 39.37 
        self._goal_y       = 10.0 * 39.37 
        self._robot_x      = 0.0
        self._robot_y      = 0.0
        self._robot_xdot   = 0.0
        self._robot_ydot   = 0.0        
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

        # Timer to send goals every 2 seconds (if list is not empty)
        self._timer = self.create_timer(0.3, self.timer_callback)
          
        # Subscriber to get position data        
        self._mutex_cbg = MutuallyExclusiveCallbackGroup() # MutuallyExclusiveCallbackGroup to ensure callback in separate thread
        self._pose_sub = self.create_subscription(
            PoseStamped,
            "odom",
            self.pose_cb,
            10,
            callback_group=self._mutex_cbg,
        )
        self._vel_sub = self.create_subscription(
            Twist,
            "velocity",
            self.vel_cb,
            10,
            callback_group=self._mutex_cbg,
        )        
                          
        #Publishers
        self.joint_position_pub   = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub      = self.create_publisher(JointState, '/joint_states', 10) # TODO what is this for?
                        
        
    def pose_cb(self, msg: PoseStamped):
        """
        Callback to handle pose data

        Args:
            msg (PoseStamped): PoseStamped message
        """
        # position
        self._robot_x = msg.pose.position.x
        self._robot_y = msg.pose.position.y        
        
        self.get_logger().info(f"Received data on robot pose-x and pose-y: {self._robot_x}, {self._robot_y}")
        
        # orientation
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        
        # Convert quaternion to Euler angles
        _, _, self._robot_yaw = euler_from_quaternion(quaternion)
        
    def vel_cb(self, msg: Twist):
        """
        Callback to handle velocity data

        Args:
            msg (Twist): Twist message
        """
        # velocity
        self._robot_xdot = msg.linear.x
        self._robot_ydot = msg.linear.y
        
        self.get_logger().info(f"Received data on robot velocity-x and velocity-y: {self._robot_xdot}, {self._robot_ydot}")        
        

    def timer_callback(self):
        """
        Send a goal to the action server every 2 seconds
        """       
        print("timer")
        if self._goal_reached:
            self.get_logger().info("Goal already reached.")
            return 

        self.get_logger().info(f"Executing goal: ({self._goal_x},{self._goal_y})")

        while rclpy.ok():  # While ROS is running
            distance_to_goal = self.compute_distance_to_goal()
            print("Distance to goal: ",distance_to_goal)

            if distance_to_goal < 0.02:
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

        # Compute angle to goal
        x             = self._robot_x    
        y             = self._robot_y
        x_dot         = self._robot_xdot
        y_dot         = self._robot_ydot       
        dy            = self._goal_y - y
        dx            = self._goal_x - x       
        angle_to_goal = math.atan2(dy, dx)
        
        print("Robot angle to goal : ",angle_to_goal)
        print("Robot position X: ",x)
        print("Robot position Y: ",y)
        print("Robot velocity X: ",x_dot)
        print("Robot velocity Y: ",y_dot)        
                
        # Compute angular velocity
        angular_velocity = self.calculate_angular_velocity(angle_to_goal)
        print("Desired steering angular velocity: ",angular_velocity)
        
        # Set car params [inches]
        L       = 38
        r_wheel = 8/2        
                                
        # kinematic equations
        lin_vel_right_wheel = float( (1 / r_wheel) * (math.sqrt(x_dot**2 + y_dot**2) + (L / 2) * angular_velocity)   ) #- (self._kl * distance_to_goal)
        lin_vel_left_wheel  = float( (1 / r_wheel) * (math.sqrt(x_dot**2 + y_dot**2) - (L / 2) * angular_velocity)   ) #- (self._kl * distance_to_goal)
        if x_dot == 0 and y_dot == 0:
            steer_angle = 0
        else:
            steer_angle = np.arctan( L * angular_velocity / math.sqrt(x_dot**2 + y_dot**2)   )
        
        # track commanded velocity & steering angle
        # self.steer_angles.append(steer_angle)
        # self.right_vels.append(lin_vel_right_wheel)
        # self.left_vels.append(lin_vel_left_wheel)        

        print("Commanded Steering Angle: ",steer_angle)
        print("Commanded Right & Left Wheel Linear Velocity: ",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
        # Publish the twist message
        self.wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel]
        self.joint_positions.data  = [steer_angle,steer_angle]
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
        
 # class ProportionalControlNode(Node):

 #    def __init__(self):
 #        super().__init__('proportional_control_node')
        
 #        #Publishers
 #        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
 #        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
 #        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
 #        #Subscriber for ODOM data
 #        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
 #        self.actual_x = 0.0
 #        self.actual_y = 0.0
 #        self.actual_theta = 0.0 

 #    def odom_callback(self, msg):
 #        self.get_logger().info(f"Received ODOM data: {msg.data}")
 #        self.actual_x = msg.pose.pose.position.x
 #        self.actual_y = msg.pose.pose.position.y
 #        ort = msg.pose.pose.orentation   # orentation message
 #        #quaternion conversion to get yaw angle of car
 #        self.actual_theta = math.atan2(2 * (ort.w * ort.z + ort.x * ort.y), 1 - 2 * (ort.y**2 + ort.z**2)) 
        
        
 #    def prop_control(self):

 #        # Circular trajectory constraints
 #        r        = 10 * 39.37   # radius of circle [cm]
 #        duration = 30   # duration of motion [sec]
 #        dt       = 1    # sample interval [sec]

 #        # Calculate goal points to make circular trajectory
 #        # using polar coordinates to define eq of circle
 #        t              = [0]
 #        x_goal         = [0]
 #        y_goal         = [0]
 #        theta_goal     = [0]
 #        x_dot_goal     = [0]
 #        y_dot_goal     = [0]
 #        theta_dot_goal = [0]                
 #        i              = 0

 #        # gather goal points along trajectoy
 #        while i < duration:
 #            i += dt
 #            print("i: ",i)
            
 #            # Assume car will travel 1/4 circle --> 2pi = full circle --> 1/4 circle = pi/2
 #            theta      = (-(np.pi / 2)) + (np.pi / 2) * ( i     / duration) #integrated chunk of circle based on time - RADIANS
 #            theta_prev = (-(np.pi / 2)) + (np.pi / 2) * ((i-dt) / duration)
            
 #            t.append(i)
 #            theta_goal.append(theta)
            
 #            x = r * np.cos(theta)       #polar coords for eq of circle
 #            y = r * np.sin(theta) + r   #offset to start at 0,0 and travel to 10,10
 #            x_goal.append(x)   
 #            y_goal.append(y)
            
 #            theta_dot = (theta - theta_prev) / dt            
 #            theta_dot_goal.append(theta_dot)
            
 #            x_dot_goal.append(-y * theta_dot)
 #            y_dot_goal.append( x * theta_dot)
                    
 #        # Command the car    
 #        self.msg = "Closed Loop Proportion Controller"
 #        self.get_logger().info(self.msg)
 #        joint_positions  = Float64MultiArray()  #front-axle steering
 #        wheel_velocities = Float64MultiArray() #rear-wheel drive
 #        linear_vel       = 0.0 #initial value
 #        steer_angle      = 0.0 #initial value
 #        steer_angles     = [0]
 #        linear_vels1      = [0]
 #        linear_vels2      = [0]
 #        k_p = .2 #proportional gain
 #        x_err_list = [0]
 #        y_err_list = [0]
 #        theta_err_list = [0]
 #        # Car params [inches]
 #        L       = 38  
 #        r_wheel = 8/2
        
 #        i = 0
 #        while i < (duration / dt):
            
 #            time.sleep(dt)
 #            i += 1
            
 #            # ODOM Postion error calculations
 #            x_err = x_goal[i] - self.actual_x
 #            y_err = y_goal[i] - self.actual_y
 #            lin_vel_err_mag = math.sqrt(x_err**2 + y_err**2)
 #            theta_err = theta_goal[i] - self.actual_theta
            
 #            prop_control_lin_vel = k_p * lin_vel_err_mag 
 #            prop_control_ang_vel = k_p * theta_err
            
 #            # lin_vel_right_wheel = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) + (L / 2) * theta_dot_goal[i]))
 #            # lin_vel_left_wheel  = float((1 / r_wheel) * (math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) - (L / 2) * theta_dot_goal[i]))
 #            # replaced the derivatives with the proptional gain control terms
 #            lin_vel_right_wheel = float((1 / r_wheel) * (prop_control_lin_vel + (L / 2) * prop_control_ang_vel))
 #            lin_vel_left_wheel  = float((1 / r_wheel) * (prop_control_lin_vel - (L / 2) * prop_control_ang_vel))
                        
 #            linear_vel  = (1/2) * (  x_dot_goal[i]  / np.cos(theta_goal[i]) / (r_wheel/2)  )
 #            steer_angle = np.arctan( L * theta_dot_goal[i] / math.sqrt(x_dot_goal[i]**2 + y_dot_goal[i]**2) )
            
 #            steer_angles.append(steer_angle)
 #            linear_vels1.append(lin_vel_right_wheel)
 #            linear_vels2.append(lin_vel_left_wheel)

 #            print("Steer Angle",steer_angle)
 #            print("Linear Velocity",-lin_vel_right_wheel,", ",lin_vel_left_wheel)
 #            # Publish the twist message
 #            wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel] 
 #            joint_positions.data = [steer_angle,steer_angle] 

 #            self.joint_position_pub.publish(joint_positions)
 #            self.wheel_velocities_pub.publish(wheel_velocities)
            
 #            #car stops when within one car length of 10,10 gridspace
 #            # if abs(x_err) < L and abs(y_err) < L:
 #            #     break

 #        print("Duration Complete. Stopping Robot")
 #        steer_angle=0.0
 #        linear_vel=0.0
 #        print("Steer Angle",steer_angle)
 #        print("Linear Velocity",linear_vel)
        
 #        # Publish the twist message
 #        wheel_velocities.data = [-linear_vel,linear_vel] # need to update with new variable names
 #        joint_positions.data = [steer_angle,steer_angle] # need to update with new variable names

 #        self.joint_position_pub.publish(joint_positions)
 #        self.wheel_velocities_pub.publish(wheel_velocities)
        
 #        return steer_angles, linear_vels1, linear_vels2, x_goal, y_goal, t           
def main(args=None):
    
    # Start Node and Controller
    print("hi")
    rclpy.init(args=args)
    node = ProportionalControlNode()
    #steer_angles, right_vels, left_vels, x_goal, y_goal, t = node.prop_control()
    
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
    
      
    
    

if __name__ == '__main__':
    main()
