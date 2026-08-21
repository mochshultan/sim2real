cmake_minimum_required(VERSION 3.8)
project(jaguar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
)

# Build RobStride Hard Real-Time C++ Node
add_executable(robstride_can_node
  src/robstride_can_node.cpp
)

target_compile_features(robstride_can_node PUBLIC cxx_std_17)

target_include_directories(robstride_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(robstride_can_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(robstride_can_node
  Threads::Threads
)

# Install C++ Executable
install(TARGETS
  robstride_can_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts as executable nodes
install(PROGRAMS
  scripts/nxp_jaguar_controller.py
  scripts/keyboard_teleop.py
  scripts/can_hardware_node.py
  scripts/robstride_motor_lib.py
  scripts/robstride_motor_test.py
  scripts/test_sit_stand.py
  scripts/check_states.py
  scripts/check_joints.py
  scripts/jaguar_main.py
  scripts/jaguar_utils.py
  scripts/parameters.py
  scripts/scan_robostride_ids.py
  scripts/set_robostride_zero.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (config, launch, models, urdf, meshes, sim2sim)
install(DIRECTORY
  config
  launch
  models
  urdf
  meshes
  sim2sim
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
