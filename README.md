# Project 2 ENPM662-Group 5 Riley Albert, Alex Hall and Carissa Arillo
## Rviz and Gazebo
### Utilize the Following Instructions to Properly Load Rviz and Gazebo:
1. Source ROS using source /opt/ros/{ROS_DISTRO}/setup.bash
2. In the root workspace, build the workspace using: colcon build, source install/setup.bash
3. Launch the Rviz and Gazebo software: ros2 launch project_two rviz_and_gazebo.launch.py

This will open Gazebo and Rviz. In Gazebo, the robot spawns with a camera sensor. The camera sensor field of view faces a scene of objects.
In Rviz, the camera feed can be seen in the bottom left corner.

## Publisher and Subscriber
### Utilize the Following Instructions to Process Image, Run IK, and Control Robot
1. Once Gazebo and Rviz are launched, source ROS and build the package in a new terminal window: source /opt/ros/{ROS_DISTRO}/setup.bash, colcon build, source install/setup.bash
2. Run the image subscriber: ros2 run project_two node_img_subscriber.py
3. Once terminal states that inverse kinematics has begun, close Gazebo and Rviz using ctrl+c in the original terminal window to speed up computation.
4. Once terminal states IK is complete, relaunch Gazebo and Rviz in the original terminal: colcon build, source install/setup.bash, ros2 launch project_two rviz_and_gazebo.launch.py
5. In the terminal where the image subscriber was run, rebuild and run path publisher: colcon build, source install/setup.bash, ros2 run project_two path_publisher.py
6. In a new terminal window source ROS and build package: source /opt/ros/{ROS_DISTRO}/setup.bash, colcon build, source install/setup.bash
7. Run joint publisher: ros2 run project_two joint_angle_publisher.py
