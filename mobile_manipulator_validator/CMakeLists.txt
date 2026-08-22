cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_validator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(mobile_manipulator_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

add_executable(task_feasibility_service_node
  src/task_feasibility_service_node.cpp
)

ament_target_dependencies(task_feasibility_service_node
  geometry_msgs
  grasping_msgs
  mobile_manipulator_msgs
  nav2_msgs
  rclcpp
  tf2
)

rosidl_get_typesupport_target(cpp_typesupport_target mobile_manipulator_msgs "rosidl_typesupport_cpp")
target_link_libraries(task_feasibility_service_node ${cpp_typesupport_target})

install(TARGETS task_feasibility_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(mobile_manipulator_msgs)

ament_package()