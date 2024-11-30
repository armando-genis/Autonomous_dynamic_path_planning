cmake_minimum_required(VERSION 3.8)
project(point_cloud_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED) 

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_node
                src/point_cloud.cpp
              )

ament_target_dependencies(point_cloud_node
                std_msgs
                rclcpp
                sensor_msgs
                tf2
                tf2_ros
                tf2_geometry_msgs
                pcl_conversions
)


target_link_libraries(point_cloud_node ${PCL_LIBRARIES})

target_link_libraries(point_cloud_node
  Eigen3::Eigen
)

target_include_directories(point_cloud_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
point_cloud_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (e.g., launch files)
install(
  DIRECTORY src 
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON for point_cloud_node.")
message(=============================================================)

ament_package()
