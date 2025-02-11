import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('auto_aim') 

    return LaunchDescription([
        Node(
            package='video_reader',
            executable='video_reader_node', 
            name='video_reader_node',  
            output='screen'
        ),
        
        Node(
            package='armor_detect',  
            executable='armor_detect_node',
            name='armor_detect_node',
            output='screen'
        ),

        Node(
            package='armor_tracker',  
            executable='armor_tracker_node', 
            name='armor_tracker_node', 
            output='screen',
            parameters=[
                os.path.join(pkg_path, "config/tracker.yaml")
            ] 
        ),
    ])
