import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    local = launch_ros.actions.Node(
        package='local_path_planning',
        executable='local_path_planning_node',
        name='local_path_planning_node',
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )
    
    return launch.LaunchDescription([
        local
    ])