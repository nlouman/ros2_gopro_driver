from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_gopro_driver',
            executable='gopro_node',
            name='gopro_node',
            parameters=['config/gopro_config.yaml'],
            output='screen'
        )
    ])