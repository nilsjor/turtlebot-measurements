from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='signal_measure',
            executable='get_signal',
            name='get_signal',
            output='screen'
        ),
        Node(
            package='signal_measure',
            executable='get_pos',
            name='get_pos',
            output='screen'
        ),
        Node(
            package='signal_measure',
            executable='visualize_signal',
            name='visualize_signal',
            output='screen'
        ),
    ])
