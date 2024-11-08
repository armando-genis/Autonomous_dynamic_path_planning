
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    paramsConfig = os.path.join(get_package_share_directory('global_dynamic_launcher'),'config','rio_voxel_ground.yaml')


    publisher_node_ground_lidar = launch_ros.actions.Node(
        package='lidar_ground_getter',
        executable='lidar_ground_node',
        name='lidar_ground_node',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen'
    )

    publisher_node_riovoxel_lidar = launch_ros.actions.Node(
        package='voxel_grid_filter',
        executable='voxel_grid_filter',
        name='voxel_grid_filter',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen'
    )

    publisher_node_pointcloud_clustering_node = launch_ros.actions.Node(
        package='pointcloud_clustering',
        executable='pointcloud_clustering_node',
        name='pointcloud_clustering_node',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
        parameters=[paramsConfig]
    )
    
    return launch.LaunchDescription([
        publisher_node_ground_lidar,
        publisher_node_riovoxel_lidar,
        publisher_node_pointcloud_clustering_node
    ])