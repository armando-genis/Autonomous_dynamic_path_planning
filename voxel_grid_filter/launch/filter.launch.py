from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    voxel_params = os.path.join(get_package_share_directory('voxel_grid_filter'),'config','param.yaml')

    return LaunchDescription([
        Node(
            package='voxel_grid_filter',
            executable='voxel_grid_filter',
            name='voxel_grid_filter',
            output='screen',
            parameters=[voxel_params]
        )
    ])



