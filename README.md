# 🏫 ENPM662-Group 5 Riley Albert, Alex Hall and Carissa Arillo

This ROS2 package demonstrates **teleoperation and proportional control** of a **custom-designed** toy car with a trailer designed in **SolidWorks**

---

## 🕹️ Teleoperation

If the config file is placed in the <root workspace>/install/project_one/share/project_one/rviz/ folder, the LaserScan will be setup correctly upon starting up Rviz with the display_part1.launch.py file. 

Execution:

1. In a terminal window in the root workspace, build the workspace:
   
```bash
colcon build
```

2. In the same terminal window in the root workspace, start RViz and Gazebo:
   
```bash
source install/setup.bash
ros2 launch project_one debug_competition.launch.py
```

3. In another terminal in the root workspace, start the teleop node:
```bash
source install/setup.bash
ros2 run project_one teleop.py
```

*Note: Teleop will allow the user to operate the car in the arena using w,s,a,d, and q keys.*

---

## 🧠 Proportional Control:

Execution:

1. In a terminal window in the root workspace, build the workspace:

```bash
colcon build
```

2. In the same terminal window in the root workspace:

```bash
source install/setup.bash
ros2 launch project_one debug_empty.launch.py
```

3. In another terminal window in the root workspace, start the `pose_subscriber` node:

```bash
source install/setup.bash
ros2 run project_one pose_subscriber.py
```

4. In another terminal in the root workspace, start the closed loop control node:

```bash
source install/setup.bash
ros2 run project_one proportional_controller.py
```

Observe the movement of the car. When ctrl + c is entered in the terminal, the plots will appear.

---

## 🎥 Execution

- [Teleoperation](https://drive.google.com/file/d/1D8-hF6ZDiEbGhcGJF3loK9Fkj2LDi-zo/view?usp=sharing)
- [Proportional Control](https://drive.google.com/file/d/1pltz6T_K9bOQBhY81XqEgg7sT0FY-u-M/view)
