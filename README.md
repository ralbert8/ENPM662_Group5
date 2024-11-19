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
## Part 1 Image Processing
### To run part 1, do the following:
1. In a terminal window in the root workspace, build the workspace using: colcon build, source install/setup.bash
2. Run the image processing node using: ros2 run project_two image_proccessing.py. This will output a .csv to <root workspace>/install/project_two/share/project_two/csv/contours.csv
## Part 2 Inverse Kinematics
### To run part 2, do the following:
1. Run the inverse kinematics node using: ros2 run project_two inverse_kinematics.py. This will plot the end effector path, output the pose to the terminal, and output a .csv to <root workspace>/install/project_two/share/project_two/csv/joint_angles.csv ** NOTE: Avoid running this node with Gazebo open. It severely slows down the process.
## Part 3 Gazebo Environment
### To launch Gazebo, do the following:
1. Launch Gazebo using: ros2 launch project_two gazebo.launch.py
## Part 4 Joint Angle Publisher
### To run part 4, do the following
1. Run joint angle publisher node using: ros2 run project_two joint_angle_publisher.py. This will move the robot arm according to the joint angles in the .csv generated in part 2.
