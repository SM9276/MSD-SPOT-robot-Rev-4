# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_end_effector_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED end_effector_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(end_effector_FOUND FALSE)
  elseif(NOT end_effector_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(end_effector_FOUND FALSE)
  endif()
  return()
endif()
set(_end_effector_CONFIG_INCLUDED TRUE)

# output package information
if(NOT end_effector_FIND_QUIETLY)
  message(STATUS "Found end_effector: 0.0.0 (${end_effector_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'end_effector' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${end_effector_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(end_effector_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${end_effector_DIR}/${_extra}")
endforeach()
