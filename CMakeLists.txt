cmake_minimum_required(VERSION 3.8)
project(franka_moveit_compliant_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(cartesian_trajectory_executor src/cartesian_trajectory_executor.cpp)
ament_target_dependencies(cartesian_trajectory_executor
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
)

install(TARGETS cartesian_trajectory_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
