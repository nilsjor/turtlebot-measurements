from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
            executable='visualize_signal',
            name='visualize_signal',
            output='screen'
        ),

        # Run the tf2_transform_node (from the turtlebot3_support package)
        ExecuteProcess(
            cmd=['ros2', 'run', 'turtlebot3_support', 'tf2_transform_node', 'channel_info', 'sensor_msgs/msg/PointCloud2', 'map'],
            name='tf2_transform_node',
            output='screen'
        ),
    ])
