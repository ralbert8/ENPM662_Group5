# ENPM662-Group

%% Part 1:
If you place the config file in the <root workspace>/install/project_one/share/project_one/rviz/ folder, then the LaserScan will be setup correctly upon starting up Rviz with the display_part1.launch.py file. 

To run part 1, do the following:
1. In a terminal window in the root workspace:
colcon build
source install/setup.bash
ros2 launch project_one display_part1.launch.py

2. In another terminal window in the root workspace:
source install/setup.bash
ros2 launch project_one gazebo_part1.launch.py
