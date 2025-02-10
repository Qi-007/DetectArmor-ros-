import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file = os.path.join(
        
        get_package_share_directory('auto_aim'), 'config', 'auto_aim_params.yaml'
    )

    print(f"Using parameter file: {param_file}") 

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
            parameters=[param_file]
        ),
    ])
