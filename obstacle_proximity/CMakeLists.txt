cmake_minimum_required(VERSION 3.8)
project(obstacle_proximity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(obstacles_information_msgs REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)

add_executable(obstacle_proximity_node
               src/obstacle_proximity_node.cpp
               src/sat_collision_checker.cc
               )

# Link libraries
ament_target_dependencies(obstacle_proximity_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  obstacles_information_msgs
  nav_msgs
)

target_include_directories(obstacle_proximity_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  obstacle_proximity_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(obstacle_proximity_node
  Eigen3::Eigen
)

# Install other files (e.g., launch files)
install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON for trajectory_obstacle_checker_node.")
message(=============================================================)

ament_package()
