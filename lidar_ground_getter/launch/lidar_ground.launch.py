import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ground_params = os.path.join(get_package_share_directory('lidar_ground_getter'),'config','param.yaml')


    publisher_node_ground_lidar = launch_ros.actions.Node(
        package='lidar_ground_getter',
        executable='lidar_ground_node',
        name='lidar_ground_node',
        output='screen',
        parameters=[ground_params]
    )
    
    return launch.LaunchDescription([
        publisher_node_ground_lidar
    ])


