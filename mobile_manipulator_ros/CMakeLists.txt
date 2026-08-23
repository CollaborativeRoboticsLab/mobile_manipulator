cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mobile_manipulator_base REQUIRED)
find_package(mobile_manipulator_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(
  include
)

add_executable(mobile_manipulator_server
  src/mobile_manipulator_server.cpp
)

ament_target_dependencies(mobile_manipulator_server
  mobile_manipulator_base
  mobile_manipulator_msgs
  pluginlib
  rclcpp
  rclcpp_action
)

install(TARGETS mobile_manipulator_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(mobile_manipulator_base mobile_manipulator_msgs pluginlib)

ament_package()