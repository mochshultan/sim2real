cmake_minimum_required(VERSION 3.8)
project(jaguar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
# yaml-cpp versions export either a namespaced target or the older plain target.
if(TARGET yaml-cpp::yaml-cpp)
  set(JAGUAR_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(JAGUAR_YAML_CPP_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp was found but exports no supported CMake target")
endif()
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

include(CTest)

include_directories(
  include
)

add_library(jaguar_ros2_control SHARED src/ros2_control_system.cpp)
target_compile_features(jaguar_ros2_control PUBLIC cxx_std_17)
target_include_directories(jaguar_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(jaguar_ros2_control hardware_interface pluginlib rclcpp)
target_link_libraries(jaguar_ros2_control ${JAGUAR_YAML_CPP_TARGET} Threads::Threads)
pluginlib_export_plugin_description_file(hardware_interface jaguar_hardware.xml)

add_library(jaguar_impedance_controller SHARED src/impedance_controller.cpp)
target_compile_features(jaguar_impedance_controller PUBLIC cxx_std_17)
target_include_directories(jaguar_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(jaguar_impedance_controller
  controller_interface pluginlib realtime_tools rclcpp sensor_msgs std_msgs std_srvs)
pluginlib_export_plugin_description_file(controller_interface jaguar_controllers.xml)

install(TARGETS jaguar_ros2_control jaguar_impedance_controller
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

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
  ament_index_cpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_ros2_control_system tests/test_ros2_control_system.cpp)
  target_link_libraries(test_ros2_control_system jaguar_ros2_control)
  ament_target_dependencies(test_ros2_control_system hardware_interface rclcpp)
  add_test(NAME ros2_control_system COMMAND test_ros2_control_system
    ${CMAKE_CURRENT_SOURCE_DIR}/config/cybergear_motor_params.yaml)
  add_executable(test_can_protocol tests/test_can_protocol.cpp)
  target_include_directories(test_can_protocol PRIVATE include)
  target_compile_features(test_can_protocol PRIVATE cxx_std_17)
  add_test(NAME can_protocol COMMAND test_can_protocol)
  add_executable(test_motion_safety tests/test_motion_safety.cpp)
  target_include_directories(test_motion_safety PRIVATE include)
  target_compile_features(test_motion_safety PRIVATE cxx_std_17)
  add_test(NAME motion_safety COMMAND test_motion_safety)
  add_executable(test_serial_parser tests/test_serial_parser.c)
  target_compile_features(test_serial_parser PRIVATE c_std_99)
  add_test(NAME serial_parser COMMAND test_serial_parser)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_can_protocol PRIVATE -UNDEBUG)
    target_compile_options(test_motion_safety PRIVATE -UNDEBUG)
    target_compile_options(test_ros2_control_system PRIVATE -UNDEBUG)
  endif()
  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_serial_parser PRIVATE -UNDEBUG)
  endif()
  add_test(NAME python_tests COMMAND ${Python3_EXECUTABLE} -m pytest
    ${CMAKE_CURRENT_SOURCE_DIR}/tests -q -p no:cacheprovider)
endif()

target_link_libraries(robstride_can_node
  Threads::Threads
  ${JAGUAR_YAML_CPP_TARGET}
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
  scripts/keyboard_cmd_vel.py
  scripts/jaguar_cmd_vel_mux.py
  scripts/can_hardware_node.py
  scripts/robstride_motor_test.py
  scripts/test_sit_stand.py
  scripts/check_states.py
  scripts/check_joints.py
  scripts/remote_xbox_forwarder.py
  scripts/gamepad_reader.py
  scripts/scan_robostride_ids.py
  scripts/set_robostride_zero.py
  scripts/calibrate_sit_zero.py
  scripts/calibrate_stand_pose.py
  scripts/calibrate_parallel_link.py
  scripts/calibrate_xbox.py
  scripts/monitor_joints.py
  scripts/record_sim2real_benchmark.py
  scripts/pace/run_jaguar_chirp.py
  DESTINATION lib/${PROJECT_NAME}
)

# Importable modules live beside the executable scripts, without extra ROS nodes.
install(FILES
  scripts/robstride_motor_lib.py
  scripts/parameters.py
  scripts/motion_limits.py
  scripts/joint_layout.py
  scripts/project_paths.py
  scripts/policy_observation.py
  scripts/session_recorder.py
  scripts/parallel_link_mapping.py
  DESTINATION lib/${PROJECT_NAME}
)

# Deploy only runtime assets. Training checkpoints and offline tools stay in source.
install(DIRECTORY config launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES models/policy.pt
  models/policy_adaboot_l2c2_nominal32v_clearance5cm_39996_20261112.pt
  models/deployment_adaboot_l2c2_nominal32v_clearance5cm_39996_20261112.json
  models/policy_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.pt
  models/deployment_adaboot_ideal_l2c2_kp45kd45_clear8cm_29996_20261022.json
  models/policy_adaboot_ideal_l2c2_zero_reference_5999_20261103.pt
  models/deployment_adaboot_ideal_l2c2_zero_reference_5999_20261103.json
  models/policy_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.pt
  models/deployment_adaboot_l2c2_zero_reference_clearance5cm_19999_20261111.json
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
