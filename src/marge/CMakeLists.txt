cmake_minimum_required(VERSION 3.8)
project(marge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_path(MODBUS_INCLUDE_DIR modbus/modbus.h)
find_library(MODBUS_LIBRARY modbus)

if(NOT MODBUS_INCLUDE_DIR)
  message(FATAL_ERROR "modbus/modbus.h not found. Please install libmodbus development headers.")
endif()

if(NOT MODBUS_LIBRARY)
  message(FATAL_ERROR "libmodbus not found. Please install the libmodbus runtime library.")
endif()

add_library(icl_stepper SHARED src/ICLStepper.cpp)
target_include_directories(icl_stepper
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MODBUS_INCLUDE_DIR}
)
target_link_libraries(icl_stepper
  PUBLIC
    ${MODBUS_LIBRARY}
)
target_compile_features(icl_stepper PUBLIC cxx_std_17)

add_executable(homer src/homer.cpp)
target_link_libraries(homer
  PRIVATE
    icl_stepper
)
target_compile_features(homer PRIVATE cxx_std_17)

install(TARGETS icl_stepper homer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(icl_stepper)

ament_package()
