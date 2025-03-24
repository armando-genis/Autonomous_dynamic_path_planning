# Autonomous_dynamic_path_planning
 

## → 📥 Install dependencies
Before installing the necessary dependencies, remember to source the appropriate ROS2 environment for your ROS2 version. This ensures the correct packages are installed for your distribution.

```bash
#fundamental libraries
sudo apt update
sudo apt-get install libeigen3-dev
sudo apt install libpcl-dev
sudo apt-get install libpcap-dev
sudo apt install libopencv-dev
#ros2 packages
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt-get install ros-$ROS_DISTRO-pcl-ros
sudo apt install ros-$ROS_DISTRO-vision-msgs
sudo apt install ros-$ROS_DISTRO-perception-pcl
sudo apt install ros-$ROS_DISTRO-pcl-msgs
sudo apt install ros-$ROS_DISTRO-vision-opencv
sudo apt install ros-$ROS_DISTRO-grid-map-ros
sudo apt install ros-$ROS_DISTRO-grid-map-cv
sudo apt install ros-$ROS_DISTRO-cv-bridge
```

## → 📥 Building

<img height="50" src="https://user-images.githubusercontent.com/25181517/192158606-7c2ef6bd-6e04-47cf-b5bc-da2797cb5bda.png">

If it is the fist time you build the workspace follow the next commands to do not crash your computer. 
 ```bash
colcon build --packages-select obstacles_information_msgs
source install/setup.bash
colcon build --packages-select global_dynamic_launcher
colcon build --packages-select lidar_ground_getter
colcon build --packages-select voxel_grid_filter
colcon build --packages-select pointcloud_clustering
colcon build --packages-select trajectory_obstacle_checker
colcon build --packages-select dynamic_hybrid_path_planning
```

## → 💡 Launch 

# Point Cloud and Obstacle Processing
```bash 
ros2 launch global_dynamic_launcher rio_voxel_ground_cluster.launch.py
```
# Local Planning

```bash 
ros2 launch local_path_planning local_planning.launch.py 
```

## Simulating Steering Input

For path planning, the system expects a steering angle input in radians. You can simulate this by publishing a `Float64` message to the `/sdv/steering/position` topic. Use the following command in your terminal:

```bash
ros2 topic pub /sdv/steering/position std_msgs/msg/Float64 "{data: 0.5}"
```

*Note:* Adjust the value (e.g., `0.5`) as needed to simulate different steering angles.

## Modifying Local Path Planning Behavior

To make the car choose a different path, you can modify the behavior of the local path planning module by changing a specific variable. In the file `local_path_planning_node.h` located in the `local_path_planning` package, update the following variable:

```cpp
bool mode_obstacle = 1;
```

Changing the value of `mode_obstacle` creates a white box in the grid map, which instructs the system to ignore obstacles near the car. Adjust this setting as needed to test different planning scenarios.

## Tinyspline Installation

To install the tinyspline library (required for certain spline interpolation functionalities), run:

```bash 
git clone https://github.com/msteinbeck/tinyspline.git tinyspline
cd tinyspline
mkdir build
cd build
cmake -DTINYSPLINE_ENABLE_CPP=True -DTINYSPLINE_ENABLE_PYTHON=False -DTINYSPLINE_ENABLE_ALL_INTERFACES=False ..
cmake --build . --target install

./test/c/tinyspline_tests
./test/cxx/tinysplinecxx_tests
```



