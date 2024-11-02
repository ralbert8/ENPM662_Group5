# ENPM662-Group 5 Riley Albert, Alex Hall and Carissa Arillo

%% Part 1:
If you place the config file in the <root workspace>/install/project_one/share/project_one/rviz/ folder, then the LaserScan will be setup correctly upon starting up Rviz with the display_part1.launch.py file. 

To run part 1, do the following:
1. In a terminal window in the root workspace, build the workspace:
colcon build

2. In the same terminal window in the root workspace, start RViz and Gazebo:
source install/setup.bash
ros2 launch project_one debug.launch.py

3. In another terminal in the root workspace, start the teleop node: 
source install/setup.bash
ros2 run project_one teleop.py

note: teleop will allow the user to operate the car in the arena using w,s,a,d, and q keys. 

%% Part 2
To run part 2, do the following: 
1. In a terminal window in the root workspace, build the workspace:
colcon build

2. In the same terminal window in the root workspace:
source install/setup.bash
ros2 launch project_one gazebo.launch.py

3. In another terminal window in the root workspace, start the pose_subscriber node:
source install/setup.bash
ros2 run project_one pose_subscriber.py

4. In another terminal in the root workspace, start the closed loop control node: 
source install/setup.bash
ros2 run project_one proportional_controller.py

observe the movement of the car, when you ctrl + c in the terminal the plots will appear.
