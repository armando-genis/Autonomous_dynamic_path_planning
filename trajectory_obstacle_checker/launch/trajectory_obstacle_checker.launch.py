import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # paramsConfig = os.path.join(get_package_share_directory('pointcloud_rotation'),'config','params.yaml')


    trajectory_obstacle_checker_node = launch_ros.actions.Node(
        package='trajectory_obstacle_checker',
        executable='trajectory_obstacle_checker_node',
        name='trajectory_obstacle_checker_node',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen'
        # parameters=[paramsConfig]
    )
    
    return launch.LaunchDescription([
        trajectory_obstacle_checker_node
    ])