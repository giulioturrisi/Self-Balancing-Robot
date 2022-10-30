# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ros2_bubble_rob_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ros2_bubble_rob_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ros2_bubble_rob_FOUND FALSE)
  elseif(NOT ros2_bubble_rob_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ros2_bubble_rob_FOUND FALSE)
  endif()
  return()
endif()
set(_ros2_bubble_rob_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ros2_bubble_rob_FIND_QUIETLY)
  message(STATUS "Found ros2_bubble_rob: 0.0.0 (${ros2_bubble_rob_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ros2_bubble_rob' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ros2_bubble_rob_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ros2_bubble_rob_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ros2_bubble_rob_DIR}/${_extra}")
endforeach()
