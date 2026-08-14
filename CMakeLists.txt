cmake_minimum_required(VERSION 3.8)
project(jaguar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Install Python scripts as executable nodes
install(PROGRAMS
  scripts/nxp_jaguar_controller.py
  scripts/check_states.py
  scripts/check_joints.py
  scripts/jaguar_main.py
  scripts/jaguar_utils.py
  scripts/parameters.py
  scripts/xiaomimotor_lib.py
  scripts/bringup_canbus.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (config, launch, models)
install(DIRECTORY
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
