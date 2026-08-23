cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mobile_manipulator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(mobile_manipulator_control_node
  src/mobile_manipulator_control_node.cpp
)

ament_target_dependencies(mobile_manipulator_control_node
  geometry_msgs
  mobile_manipulator_msgs
  rclcpp
  rclcpp_action
)

target_compile_features(mobile_manipulator_control_node PUBLIC cxx_std_17)

install(TARGETS mobile_manipulator_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()