# 🏫 ENPM662-Group 5 Riley Albert, Alex Hall and Carissa Arillo

This ROS2 simulation demonstrates the use of a Raspberry Pi camera module to capture the contours of objects in its frame, where it will be drawn on a canvas by a UR manipulator.

---

*Note: If you encounter issues running with the following instructions, scroll down to the next set of instructions with work-arounds in place. Some team members needed the work-around due to computer memory / processing speed limitations.*

---

##  🤖 Rviz and Gazebo

Utilize the Following Instructions to Properly Load Rviz and Gazebo:

1. Source ROS using:

```bash
source /opt/ros/{ROS_DISTRO}/setup.bash
```

2. In the root workspace, build the workspace using:

```bash
colcon build
source install/setup.bash
```

3. Launch the Rviz and Gazebo software:
```bash
ros2 launch project_two rviz_and_gazebo.launch.py
```

This will open Gazebo and Rviz. In Gazebo, the robot spawns with a camera sensor. The camera sensor field of view faces a scene of objects. The Gazebo world contains a drawing canvas and a scene for the robot to sketch. 
In Rviz, the camera feed can be seen in the bottom left corner.

---

## 💬 Publisher and Subscriber

Utilize the Following Instructions to Process the Image, Run Inverse Kinematics, and Control the Robot to Sketch the Image

1. Node 1 - Camera Image Subscriber. 
    - Open a new terminal window.

    - ```bash
      source install/setup.bash
      ros2 run project_two node_img_subscriber.py
      ```
      
    - Inverse Kinematics will be printed to the terminal window. Wait for the completion message: "Finished running inverse kinematics on planned end effector trajectory!"

3. Node 2 - End Effector (Actual) Path Publisher.  
    - Open a new terminal window.

      ```bash
      source install/setup.bash 
      ros2 run project_two path_publisher.py
      ```

    - The terminal will show a message indicating it is listening for the actual joint angles: "FKSubscriber node initialized and listening for joint angles."

4. Node 3 - Joint Angle Command Publisher.
    - Open a new terminal window.

      ```bash
      source install/setup.bash
      ros2 run project_two joint_angle_publisher.py
      ```
      
    - The commanded joint angles will be printed to the terminal window.
View the RViz and Gazebo windows to watch the robot arm maneuver to sketch the Gazebo scene. The RViz window will also plot the path of the end effector in realtime. 




## WORK-AROUND INSTRUCTIONS
## Rviz and Gazebo
### Utilize the Following Instructions to Properly Load Rviz and Gazebo:
1. Source ROS using source /opt/ros/{ROS_DISTRO}/setup.bash
2. In the root workspace, build the workspace using: colcon build, source install/setup.bash
3. Launch the Rviz and Gazebo software: ros2 launch project_two rviz_and_gazebo.launch.py

This will open Gazebo and Rviz. In Gazebo, the robot spawns with a camera sensor. The camera sensor field of view faces a scene of objects. The Gazebo world contains a drawing canvas and a scene for the robot to sketch. 
In Rviz, the camera feed can be seen in the bottom left corner.

## Publisher and Subscriber
### Utilize the Following Instructions to Process the Image, Run Inverse Kinematics, and Control the Robot to Sketch the Image
1. Once Gazebo and Rviz are launched, source ROS and build the package in a new terminal window: source /opt/ros/{ROS_DISTRO}/setup.bash, colcon build, source install/setup.bash
2. Run the image subscriber: ros2 run project_two node_img_subscriber.py
3. Once terminal states that inverse kinematics has begun, close Gazebo and Rviz using ctrl+c in the original terminal window to speed up computation.
4. Once terminal states IK is complete, relaunch Gazebo and Rviz in the original terminal: colcon build, source install/setup.bash, ros2 launch project_two rviz_and_gazebo.launch.py
5. In the terminal where the image subscriber was run, rebuild and run path publisher: colcon build, source install/setup.bash, ros2 run project_two path_publisher.py
6. In a new terminal window source ROS and build package: source /opt/ros/{ROS_DISTRO}/setup.bash, colcon build, source install/setup.bash
7. Run joint publisher: ros2 run project_two joint_angle_publisher.py
