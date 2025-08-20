#!/usr/bin/env python3

"""
Keyboard teleoperation for a front-wheel steer, rear-wheel drive robot in ROS2.

Use WASD to command:
    - w: Increase linear velocity
    - s: Decrease linear velocity
    - a: Increment steer left
    - d: Increment steer right
    - q: Force stop
    - ESC: quit

Publishes steering angles to `/position_controller/commands` and wheel linear
velocities to `/velocity_controller/commands`. Also advertises `/joint_states`
for completeness.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Step size for incremental changes
LIN_VEL_STEP_SIZE = 1
ANG_VEL_STEP_SIZE = 0.1



class KeyboardControlNode(Node):

    """
    A ROS2 node that reads keybpard input from a POSIX terminal and publishes
    steering angles and wheel velocities for manual teleoperation.

    Attributes:
        joint_position_pub (Publisher): Publishes steering angles (Float64MultiArray)
            to `/position_controller/commands` as `[left_steer, right_steer]`.
        wheel_velocities_pub (Publisher): Publishes wheel velocities 
            (Float64MultiArray) to `/velocity_controller/commands` as `[left, right]`.
        joint_state_pub (Publisher): Publishes to `/joint_states`.
        settings (list): Original terminal settings captured via `termios.tcgetattr
            to restore cooked mode after raw input reads.
    """


    
    def __init__(self):

        """
        Initialize publishers and capture terminal settings for raw key reads.
        """

        # Inherit node capabilities
        super().__init__('keyboard_control_node')

        # Create publishers
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Get terminal settings
        self.settings = termios.tcgetattr(sys.stdin)


    
    def getKey(self):

        """
        Read a single, non-blocking, key press from stdin using raw terminal mode.

        Returns:
            str: The key pressed, or '' if no key was pressed before timeout.
        """
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    
    def run_keyboard_control(self):

        """
        Main loop for keyboard teleoperation.

        Attributes:
            msg (str): Help string printed at startup showing keybinds.
        """
        
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_angle=0.0

        # Handle keystrokes
        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE

                # Clamp steering to reasonable range
                if steer_angle>1.0:
                        steer_angle=1.0
                if steer_angle<-1.0:
                    steer_angle=-1.0

                print("Steer Angle",steer_angle)
                print("Linear Velocity",linear_vel)
                # Publish the twist message
                wheel_velocities.data = [-linear_vel,linear_vel]
                joint_positions.data = [steer_angle,steer_angle]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)



def main(args=None):

    """
    Initialize ROS2, run the keyboard teleop loop, and cleanly shut down.

    Parameters:
        args (list[str] | None, optional): Command ling arguments passed to rclpy.init.
    """
    
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    main()
