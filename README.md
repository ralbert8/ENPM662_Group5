# ENPM662-Group

%% Part 1:
If you place the config file in the <root workspace>/install/project_one/share/project_one/rviz/ folder, then the LaserScan will be setup correctly upon starting up Rviz with the display_part1.launch.py file. 

To run part 1, do the following:
1. In a terminal window in the root workspace, build the workspace:
colcon build

1. In the same terminal window in the root workspace, start RViz:
source install/setup.bash
ros2 launch project_one display_part1.launch.py

2. In another terminal window in the root workspace, start Gazebo:
source install/setup.bash
ros2 launch project_one gazebo_part1.launch.py

3. In another terminal in the root workspace, start the teleop node: 
source install/setup.bash
ros2 run project_one teleop.py

note: teleop will allow the user to operate the car in the arena using w,s,a,d, and q keys. 
