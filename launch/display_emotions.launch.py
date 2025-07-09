from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Display Emotions Node
        Node(
            package='display_emotions',
            executable='display_emotions_node',
            name='display_emotions_node',
            output='screen',
            parameters=[
                {'faces_cycle': True},
                {'faces_cycle_delay': 0.25},
                {'reset_to_idle': True}
            ]
        ),

        # RQT Image View
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            arguments=[],
            remappings=[
                ('/image', '/utbots/display_emotions/image')
            ]
        )
    ])
