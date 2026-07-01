import launch
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = '/auto/line'

    matcher_package_dir = FindPackageShare('nav_line_matcher')
    follower_package_dir = FindPackageShare('nav_path_follow')
    arbitration_package_dir = FindPackageShare('nav_arbitration')

    matcher_yaml_path = PathJoinSubstitution(
        [matcher_package_dir, 'config', 'default.yaml'])
    follower_yaml_path = PathJoinSubstitution(
        [follower_package_dir, 'config', 'default.yaml'])
    arbitration_yaml_path = PathJoinSubstitution(
        [arbitration_package_dir, 'config', 'default.yaml'])

    container = ComposableNodeContainer(
        name='nav_line_matcher_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='nav_line_matcher',
                plugin='nav_line_matcher::LineMatcherServer',
                name='matcher',
                parameters=[matcher_yaml_path]),
            ComposableNode(
                package='nav_path_follow',
                plugin='nav_path_follow::PathFollower',
                name='follower',
                parameters=[follower_yaml_path])
        ],
        output='screen',
    )

    container_with_ns = GroupAction([
        PushRosNamespace(ns),
        container
    ])

    arbitration = Node(
        package='nav_arbitration',
        namespace='',
        executable='nav_arbitration',
        name='arbitration',
        parameters=[arbitration_yaml_path],
        output='screen'
    )

    return launch.LaunchDescription([container_with_ns, arbitration])
