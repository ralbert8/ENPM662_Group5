# Project 2 ENPM662-Group 5 Riley Albert, Alex Hall and Carissa Arillo
## Part 1 Rviz and Gazebo initialization
### To run part 1, do the following:
1. In a terminal window in the root workspace, build the workspace using: colcon build, source install/setup.bash
2. Launch the Rviz and Gazebo software: ros2 launch project_two rviz_and_gazebo.launch.py. 
This will open Gazebo and Rviz. In Gazebo, the robot spawns with a camera sensor. The camera sensor field of view faces a scene of objects.
In Rviz, the camera feed can be seen in the bottom left corner.

## Part 2 Publisher and Subscriber
### To run part 2, do the following:
1. Launch the camera subscriber and joint angle publisher: ros2 launch project_two pub_and_sub.launch.py. 
The subscriber performs the following:
 - Subscribes to the camera raw_image topic, processes it to extract the scene contours, and outputs to <root workspace>/install/project_two/share/project_two/csv/contours.csv.
 - Then the inverse kinematics will be determined to plan the robot arm trajectory path to sketch the contours, print the pose to the terminal, and output to <root workspace>/install/project_two/share/project_two/csv/joint_angles.csv.
The publisher performs the following: 
 - Waits for the joint_angles.csv to generate and begins commanding the robot arm joints according to the planned trajectory. 
