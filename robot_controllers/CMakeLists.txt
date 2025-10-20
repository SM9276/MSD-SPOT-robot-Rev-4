cmake_minimum_required(VERSION 3.8)
project(robot_controllers)

set(CMAKE_CXX_STANDARD 17) # Set the desired C++ standard (e.g., 11, 14, 17, 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Compile
add_library(
  robot_controllers
  SHARED
  src/ArmSystem.cpp
  src/Motor.cpp
  src/ICLStepper.cpp
  src/Communication/RS485.cpp
  src/Communication/ModbusDevice.cpp
)
target_compile_features(robot_controllers PUBLIC cxx_std_17)
target_include_directories(robot_controllers PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/robot_controllers>
)
ament_target_dependencies(
  robot_controllers PUBLIC
  ${PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robot_controllers.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/robot_controllers
)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS robot_controllers
  EXPORT export_robot_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_robot_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_INCLUDE_DEPENDS})
ament_package()