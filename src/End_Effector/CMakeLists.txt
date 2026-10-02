cmake_minimum_required(VERSION 3.8)
project(end_effector)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/pwm_output_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pwm_output_node
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
