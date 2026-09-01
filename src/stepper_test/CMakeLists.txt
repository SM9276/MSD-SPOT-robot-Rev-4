cmake_minimum_required(VERSION 3.8)
project(stepper_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(motor_key src/motor_key.cpp src/ICLStepper.cpp)
target_include_directories(motor_key PUBLIC include)
ament_target_dependencies(motor_key rclcpp)
target_link_libraries(motor_key modbus)

install(TARGETS
  motor_key
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
