import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='video_reader',
            executable='video_reader_node', 
            name='video_reader_node',  
            output='screen'
        ),
        
        launch_ros.actions.Node(
            package='armor_detect',  
            executable='armor_detect_node',
            name='armor_detect_node',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='armor_tracker',  
            executable='armor_tracker_node', 
            name='armor_traker',
            output='screen'
        ),

    ])

