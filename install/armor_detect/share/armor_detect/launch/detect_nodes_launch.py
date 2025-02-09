from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='armor_detect',
            executable='armor_detect_node',
            name='armor_detection',
            output='screen',
        ),
        Node(
            package='armor_detect',
            executable='video_reader_node',
            name='video_reader',
            output='screen',
        ),
    ])
