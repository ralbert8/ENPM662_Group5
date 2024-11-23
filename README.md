# Project 1 ENPM662-Group 5 Riley Albert, Alex Hall and Carissa Arillo
%% Part 1 Teleop:
If you place the config file in the <root workspace>/install/project_one/share/project_one/rviz/ folder, then the LaserScan will be setup correctly upon starting up Rviz with the display_part1.launch.py file. 

To run part 1, do the following:
1. In a terminal window in the root workspace, build the workspace:
colcon build

2. In the same terminal window in the root workspace, start RViz and Gazebo:
source install/setup.bash
ros2 launch project_one debug_competition.launch.py

3. In another terminal in the root workspace, start the teleop node: 
source install/setup.bash
ros2 run project_one teleop.py

note: teleop will allow the user to operate the car in the arena using w,s,a,d, and q keys. 

%% Part 2 Proportional Control:
To run part 2, do the following: 
1. In a terminal window in the root workspace, build the workspace:
colcon build

2. In the same terminal window in the root workspace:
source install/setup.bash
ros2 launch project_one debug_empty.launch.py

3. In another terminal window in the root workspace, start the pose_subscriber node:
source install/setup.bash
ros2 run project_one pose_subscriber.py

4. In another terminal in the root workspace, start the closed loop control node: 
source install/setup.bash
ros2 run project_one proportional_controller.py


observe the movement of the car, when you ctrl + c in the terminal the plots will appear.

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
