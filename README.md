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

## → 🏅 lidar config 
`sudo ifconfig enp2s0 10.66.171.101`

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
```bash
ros2 launch pointcloud_clustering pointcloud_clustering.launch.py
ros2 launch trajectory_obstacle_checker trajectory_obstacle_checker.launch.py
ros2 launch dynamic_hybrid_path_planning dynamic_planning.launch.py
```

new launcher 

```bash 
ros2 launch global_dynamic_launcher rio_voxel_ground_cluster.launch.py
```