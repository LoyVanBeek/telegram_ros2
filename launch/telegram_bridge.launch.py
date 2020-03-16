import pathlib
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = pathlib.Path(__file__).parents[0] / 'config' / 'example_param.yaml'

    return LaunchDescription([
        Node(
            package='telegram_ros2',
            node_executable='telegram_ros2_bridge',
            node_name='telegram_bridge',
            parameters=[parameters_file_path]
        )
    ])
