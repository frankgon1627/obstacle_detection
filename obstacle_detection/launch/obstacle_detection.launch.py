from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container_name = LaunchConfiguration('container_name', default='/ouster/os_container')

    declare_container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value=container_name,
        description='Name of the container to load nodes into'
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="obstacle_detection",
                plugin="obstacle_detection::PositiveObstacleDetectionNode",
                name="positive_obstacle_detection",
            ),
            ComposableNode(
                package="obstacle_detection",
                plugin="obstacle_detection::NegativeObstacleDetectionNode",
                name="negative_obstacle_detection",
            ),
            ComposableNode(
                package="obstacle_detection",
                plugin="obstacle_detection::CombinerNode",
                name="combiner_node",
            )
        ]
    )

    return LaunchDescription([
        declare_container_name_arg,
        load_composable_nodes
    ])