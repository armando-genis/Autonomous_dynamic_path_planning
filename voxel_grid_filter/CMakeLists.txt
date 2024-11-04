cmake_minimum_required(VERSION 3.5)
project(voxel_grid_filter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# External dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED) 

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(voxel_grid_filter src/VoxelGrid.cpp)

ament_target_dependencies(voxel_grid_filter rclcpp std_msgs pcl_conversions sensor_msgs)
target_link_libraries(voxel_grid_filter Eigen3::Eigen)
target_link_libraries(voxel_grid_filter ${OpenCV_LIBS})

target_link_libraries(voxel_grid_filter ${PCL_LIBRARIES})

install(TARGETS
  voxel_grid_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
