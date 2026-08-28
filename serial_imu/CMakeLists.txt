cmake_minimum_required(VERSION 3.8)
project(serial_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(talker src/serial_port.cpp src/ch_serial.c)
target_compile_features(talker PUBLIC c_std_99 cxx_std_17)
target_include_directories(talker PRIVATE src)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)

add_executable(listener src/sub_spec.cpp)
target_compile_features(listener PUBLIC cxx_std_17)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

