cmake_minimum_required(VERSION 3.8)
project(pyroki)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
# Install examples directory to make pyroki_snippets available
install(DIRECTORY examples/
  DESTINATION lib/python3.10/site-packages/
  FILES_MATCHING PATTERN "*.py")

# Install Python executables in the ROS2 expected location
install(PROGRAMS src/pyroki/ros2_nodes/ik_service_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pyroki_ik_service)
install(PROGRAMS src/pyroki/ros2_nodes/controller_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pyroki_controller_interface)
install(PROGRAMS src/pyroki/ros2_nodes/trajectory_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pyroki_trajectory_planner)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install examples
install(DIRECTORY examples
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()