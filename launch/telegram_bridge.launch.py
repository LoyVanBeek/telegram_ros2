from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    parameters_file_path = Path(get_package_share_directory('telegram_ros2'), 'config', 'example_param.yaml')
    return LaunchDescription([
        Node(
            package='telegram_ros2',
            node_executable='telegram_ros2_bridge',
            node_name='telegram_bridge',
            parameters=[parameters_file_path]
        )
    ])
