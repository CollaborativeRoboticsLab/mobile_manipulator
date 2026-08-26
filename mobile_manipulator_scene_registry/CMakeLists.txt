cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_scene_registry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(mobile_manipulator_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(scene_registry_node
  src/scene_registry_node.cpp
)

target_include_directories(scene_registry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_registry_node
  ament_index_cpp
  grasping_msgs
  mobile_manipulator_base
  pluginlib
  rclcpp
  rclcpp_action
)

target_link_libraries(scene_registry_node yaml-cpp)

install(TARGETS scene_registry_node
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
ament_export_dependencies(ament_index_cpp grasping_msgs mobile_manipulator_base pluginlib rclcpp rclcpp_action)

ament_package()