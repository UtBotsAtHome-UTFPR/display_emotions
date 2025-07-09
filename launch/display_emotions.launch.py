from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Publish Emotions Node
        Node(
            package='display_emotions',
            executable='display_emotions_node',
            name='display_emotions_node',
            output='screen',
            parameters=[
                {'faces_cycle': True},
                {'faces_cycle_delay': 0.25},
                {'reset_to_idle': True},
                {'aspect_ratio': '1024_600'},  # 1024_600 ou '4_3'
                {'image_topic': '/utbots/display_emotions/image'}
            ]
        ),

        # Show Emotino Images
        Node(
            package='display_emotions',
            executable='cv_image_view',
            name='cv_image_view',
            parameters=[{'image_topic': '/utbots/display_emotions/image'}],
            output='screen',
        ),

    ])
