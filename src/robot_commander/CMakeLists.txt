cmake_minimum_required(VERSION 3.8)

project(robot_commander)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(
  moveit_ros_planning_interface
  REQUIRED
)


add_executable(
  commander
  src/commander.cpp
)


ament_target_dependencies(
  commander

  rclcpp
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
)


target_compile_features(
  commander
  PUBLIC
  cxx_std_17
)


install(
  TARGETS
  commander

  DESTINATION
  lib/${PROJECT_NAME}
)


install(
  DIRECTORY
  launch

  DESTINATION
  share/${PROJECT_NAME}

  OPTIONAL
)


ament_package()
