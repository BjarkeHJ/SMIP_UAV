from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('smip_uav')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'smip_uav_params.yaml']),
        description='Path to the SMIP UAV parameter YAML file',
    )

    smip_node = Node(
        package='smip_uav',
        executable='smip_uav_node',
        name='smip_uav_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_arg,
        smip_node,
    ])
