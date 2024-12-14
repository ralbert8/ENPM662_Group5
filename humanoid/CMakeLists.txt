cmake_minimum_required(VERSION 3.8)
project(humanoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  urdf
  meshes
  launch
  worlds
  config
  src
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/humanoid
  DESTINATION include
)

install(
  PROGRAMS
  launch/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

add_library(odometry_plugin SHARED
  src/odometry_plugin.cpp
)

target_include_directories(odometry_plugin PUBLIC include)

ament_target_dependencies(odometry_plugin
  "gazebo_ros"
  "nav_msgs"
  "tf2_ros"
)

ament_export_libraries(odometry_plugin)

install(TARGETS
  odometry_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
