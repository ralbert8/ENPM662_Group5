#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import math
from datetime import datetime
from std_msgs.msg import String

class ProportionalControlNode(Node):

    """
    Proportional controller for a front-steer / read-drive robot in ROS2.

    Publishes steering angles to `/position_controller/commands` and wheel
    linear velocities to `/velocity_controller/commands` while driving toward a fixed
    goal (``_goal_x``, ``_goal_y``). It logs state and can plot commands/trajectory.

    Attributes:
        actual_x (float): Current x-position from odometry [m].
        actual_y (float): Current y-position from odometry [m].
        actual_theta (float): Current yaw/heading [rad].
        actual_vel_x (float): Current x linear velocity [m/s].
        actual_vel_y (float): Current y linear velocity [m/s].
        k_v (float): Proportional gain for linear velocity.
        k_t (float): Proportional gain for angular control.
        _goal_x (float): Goal x-position [m].
        _goal_y (float): Goal y-position [m].
        _goal_reached (bool): Flag indicating goal completion.
        L (float): Wheelbase [m].
        r_wheel (float): Wheel radius [m].
        joint_position_pub (Publisher): Publishes front steering angles to `/position_controller/commands`.
        wheel_velocities_pub (Publisher): Publishes wheel linear velocities to `/velocity_controller/commands`.
        goal_pub (Publisher): Publishes status to `/goal_flag`.
        _mutex_cbg (MutuallyExclusiveCallbackGroup): Callback group for thread safety.
        odom_sub (Subscription): Subscribes to `/odom` (PoseStamped).
        vel_sub (Subscription): Subscribes to `/velocity` (Twist).
        joint_positions (Float64MultiArray): Outgoing steering command buffer.
        wheel_velocities (Float64MultiArray): Outgoing wheel velocity buffer.
        steer_angles (list[float]): Commanded steering angles history [deg].
        right_vels (list[float]): Right wheel velocities history [m/s].
        left_vels (list[float]): Left wheel velocities history [m/s].
        x_vals (list[float]): Logged x positions [m].
        y_vals (list[float]): Logged y positions [m].
        theta_vals (list[float]): Logged headings [deg].
        t (list[datetime]): Timestamps for logged samples.
        _timer (Timer): Control loop timer.
    """


    
    def __init__(self):

        """
        Initializes publishers, subscribers, timers, controller gains, and state.
        """

        # Inherit node capabilities
        super().__init__('proportional_control_node')
        
        # Initialize robot state
        self.actual_x     = 0.0
        self.actual_y     = 0.0
        self.actual_theta = 0
        self.actual_vel_x = 0.0
        self.actual_vel_y = 0.0

        # Proportional control gains
        self.k_v          = 0.8
        self.k_t          = 2.8

        # Goal coordinates
        self._goal_x      = 10.0
        self._goal_y      = 10.0
        self._goal_reached = False

        # Robot parameters
        self.L       = 38 * .0254 # Wheelbase in meters
        self.r_wheel = (8/2) * .0254 # Wheel radius in meters
        
        
        # Publishers
        self.joint_position_pub   = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.goal_pub             = self.create_publisher(String,            '/goal_flag', 10)
        
        # Subscribers (Mutually exclusive callback group for thread safety)
        self._mutex_cbg = MutuallyExclusiveCallbackGroup()
        self.odom_sub   = self.create_subscription(PoseStamped, '/odom',     self.odom_callback,     10, callback_group=self._mutex_cbg)
        self.vel_sub    = self.create_subscription(Twist,       '/velocity', self.velocity_callback, 10, callback_group=self._mutex_cbg)

        # Data for plotting  
        self.joint_positions  = Float64MultiArray()
        self.wheel_velocities = Float64MultiArray()
        self.steer_angles     = []
        self.right_vels       = []
        self.left_vels        = []
        self.x_vals           = []
        self.y_vals           = []
        self.theta_vals       = []
        self.t                = []  
        
        # Timer for regular control loop
        self._timer = self.create_timer(0.1, self.timer_callback)


    
    def odom_callback(self, msg: PoseStamped):

        """
        Update the robot's actual position and orientation from `PoseStamped` message.

        Parameters:
            msg (PoseStamped): Pose message with position and orientation (quaternion).
        """
        
        self.actual_x = msg.pose.position.x
        self.actual_y = msg.pose.position.y
        ort = msg.pose.orientation

        # Convert quaternion to yaw angle
        self.actual_theta = math.atan2(2 * (ort.w * ort.z + ort.x * ort.y), 1 - 2 * (ort.y**2 + ort.z**2)) 


    
    def velocity_callback(self, msg: Twist):

        """
        Update robot linear velocity from a `Twist` message.

        Parameters:
            msg (Twist): Velocity message. Uses `linear.x` and `linear.y` [m/s]/
        """
        
        self.actual_vel_x = msg.linear.x
        self.actual_vel_y = msg.linear.y


    
    def timer_callback(self):
        
        """
        Periodic control loop that decides whether to stop or adjust motion
        based on distance to goal and overshoot bounds.
        """ 
        
        print("\n")

        # If goal is reached
        if self._goal_reached:
            self.get_logger().info("Goal already reached. Press ctrl+c to exit and plot.")
            return

        # While ROS is running
        while rclpy.ok():
            
            distance_to_goal = math.sqrt((self._goal_x - self.actual_x) ** 2 + (self._goal_y - self.actual_y) ** 2)

            # Stop if within tolerance or overshoot margin
            if distance_to_goal < .5:
                self.stop_robot()
                return
            elif self.actual_x > self._goal_x + 2 or \
                 self.actual_y > self._goal_y + 2:
                self.stop_robot()
                return
            else:
                self.adjust_robot_motion()


    
    def compute_distance_to_goal(self) -> float:
        
        """
        Compute Euclidean distance from current position to goal.

        Returns:
            float: Distance to the goal. Capped at 10 to avoid runaway commands.
        """

        # Compute Euclidean distance to goal
        dist = math.sqrt(
            (self._goal_x - self.actual_x) ** 2 + (self._goal_y - self.actual_y) ** 2
        )
        max_wheel_velocity = 10
        
        return min(dist, max_wheel_velocity)


    
    def calculate_angular_velocity(self, angle_to_goal):
        
        """
        Compute bounded angular command toward target heading

        Parameters:
            angle_to_goal (float): Absolute angle from robot pose to goal.

        Returns:
            float: Angular error bounded by +/- 45 degrees to avoid unrealistic steering.
        """
        
        max_steering_angle = np.radians(45)
        
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
        angular_velocity = w
        
        if angular_velocity > 0 and angular_velocity > max_steering_angle:
            angular_velocity = max_steering_angle
        elif angular_velocity < 0 and abs(angular_velocity) > max_steering_angle:
            angular_velocity = -max_steering_angle
            
        return angular_velocity


    
    def stop_robot(self):
        
        """
        stop the robot and publish final commands and goal-status message.
        """

        # Set goal reached flag
        self._goal_reached=True

        # Stop and reset all robot motion
        steer_angle         = 0.0
        lin_vel_right_wheel = 0.0
        lin_vel_left_wheel  = 0.0

        # Store current timestamp
        self.t.append(datetime.now())

        # Store current robot pose information
        self.theta_vals.append(np.degrees(self.actual_theta))
        self.x_vals.append(self.actual_x)
        self.y_vals.append(self.actual_y)        
        self.steer_angles.append(np.degrees(steer_angle))
        self.right_vels.append(lin_vel_right_wheel)
        self.left_vels.append(-lin_vel_left_wheel)                
        
        # Publish the stop commands
        self.wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel]
        self.joint_positions.data  = [steer_angle,steer_angle]
        self.joint_position_pub.publish(self.joint_positions)
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        self.get_logger().info(f'Publishing Velocity Command: {list(self.wheel_velocities.data)}')
        self.get_logger().info(f'Publishing Steering Command: {list(self.joint_positions.data)}')
                     
                     
        msg = String()
        msg.data = "******************************************"
        self.goal_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        msg.data = "Control Loop Complete. Stopping Robot..."
        self.goal_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        msg.data = "******************************************"
        self.goal_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    
        
    def adjust_robot_motion(self):  
        
        """
        Compute and publish steering and velocity commands using proportional control.
        """              
                
        # Compute linear error
        lin_vel_err_mag = self.compute_distance_to_goal()

        # Compute components of linear error
        dy= self._goal_y - self.actual_y
        dx= self._goal_x - self.actual_x

        # Compute angle to goal
        angle_to_goal = math.atan2(dy, dx)

        # Compute angular error
        theta_err = angle_to_goal- self.actual_theta

        # Compute required angular velocity
        steer_angle_vel = self.calculate_angular_velocity(theta_err)

        # Perform proportional control
        prop_control_lin_vel = self.k_v * lin_vel_err_mag 
        prop_control_ang_vel = self.k_t * steer_angle_vel

        # Adjust wheel velocities
        lin_vel_right_wheel = float((1 / self.r_wheel) * (prop_control_lin_vel + (self.L / 2) * prop_control_ang_vel))
        lin_vel_left_wheel  = float((1 / self.r_wheel) * (prop_control_lin_vel - (self.L / 2) * prop_control_ang_vel))
        
        # Store current timestamp        
        self.t.append(datetime.now())

        # Store current robot pose information
        self.theta_vals.append(np.degrees(self.actual_theta))
        self.x_vals.append(self.actual_x)
        self.y_vals.append(self.actual_y)
        self.steer_angles.append(np.degrees(prop_control_ang_vel))
        self.right_vels.append(lin_vel_right_wheel)
        self.left_vels.append(-lin_vel_left_wheel)

        # Store wheel velocities and steering angles
        self.wheel_velocities.data = [-lin_vel_left_wheel,lin_vel_right_wheel] 
        self.joint_positions.data  = [prop_control_ang_vel,prop_control_ang_vel] 

        # Publish commands
        self.joint_position_pub.publish(self.joint_positions)
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        self.get_logger().info(f'Publishing Velocity Command: {list(self.wheel_velocities.data)}')
        self.get_logger().info(f'Publishing Steering Command: {list(self.joint_positions.data)}')


    
    def plot_pose(self):
        
        """
        Plot steering commands, orientation history, wheel velocities, and X-Y path.
        """
                    
        # Plot commanded positions / velocities over time
        plt.figure()
        plt.subplot(2, 2, 1)
        plt.plot(self.t, self.steer_angles, label='Commanded Steering Angle')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Angle (degrees)')
        plt.title('Commanded Steering Angle over Time')
    
        
        plt.subplot(2, 2, 2)
        plt.plot(self.t, self.theta_vals, label='Car Steering Angle Positions')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Steering Angle (degrees)')
        plt.title('Robot Steering Angle over Time')
        
        plt.subplot(2, 2, 3)    
        plt.plot(self.t, self.right_vels, label='Commanded RIGHT Wheel Velocity')
        plt.plot(self.t, self.left_vels,  label='Commanded LEFT Wheel Velocity')    
        plt.xlabel('Time (seconds)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Commanded Wheel Velocities over Time')  
        
        plt.subplot(2, 2, 4)
        plt.scatter(self.x_vals, self.y_vals, label='Car X-Y Positions')
        plt.xlabel('X-coordinate (meters)')
        plt.ylabel('Y-coordinate (meters)')
        plt.title('Robot Position over Time')
        
        plt.legend()
        plt.grid()
        plt.savefig("part2_pose_plot.png")
        plt.show()



def main(args=None):
    
    """
    Entry point: Create and spin the node with a MultiThreadedExecutor.

    Parameters:
        args (list[str] | None, optional): Command line arguments to rclpy.init.
    """
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



if __name__ == '__main__':
    main()
