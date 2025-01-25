# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_graph_waypoints_creation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED graph_waypoints_creation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(graph_waypoints_creation_FOUND FALSE)
  elseif(NOT graph_waypoints_creation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(graph_waypoints_creation_FOUND FALSE)
  endif()
  return()
endif()
set(_graph_waypoints_creation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT graph_waypoints_creation_FIND_QUIETLY)
  message(STATUS "Found graph_waypoints_creation: 0.0.0 (${graph_waypoints_creation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'graph_waypoints_creation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${graph_waypoints_creation_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(graph_waypoints_creation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${graph_waypoints_creation_DIR}/${_extra}")
endforeach()
