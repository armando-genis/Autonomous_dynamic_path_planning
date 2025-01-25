cmake_minimum_required(VERSION 3.8)
project(graph_waypoints_creation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(graph_waypoints_creation src/graph_waypoints_creation.cpp)

# Link the executable with the libraries
ament_target_dependencies(graph_waypoints_creation 
                          rclcpp 
                          geometry_msgs 
                          std_msgs 
                          visualization_msgs 
                          nav_msgs 
                          tf2 
                          tf2_ros 
                          tf2_geometry_msgs 
                          Eigen3)

target_include_directories(graph_waypoints_creation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  graph_waypoints_creation
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(graph_waypoints_creation
  Eigen3::Eigen
)

install(
  DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON for graph_waypoints_creation.")
message(=============================================================)

ament_package()
