from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detection',
            executable='positive_obstacle_detection',
            name='positive_obstacle_detection',
            output='screen'
        ),
        Node(
            package='obstacle_detection',
            executable='negative_obstacle_detection',
            name='negative_obstacle_detection',
            output='screen'
        )
    ])
