cmake_minimum_required(VERSION 3.8)
project(robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Find Modbus library
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

add_library(robot_hardware_arm SHARED
  src/arm_hardware_interface.cpp
  src/ICLStepper.cpp
)

target_include_directories(robot_hardware_arm PUBLIC include ${LIBMODBUS_INCLUDE_DIRS})
target_link_libraries(robot_hardware_arm ${LIBMODBUS_LIBRARIES})

ament_target_dependencies(robot_hardware_arm
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware_plugin.xml)

install(TARGETS robot_hardware_arm
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(FILES robot_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
