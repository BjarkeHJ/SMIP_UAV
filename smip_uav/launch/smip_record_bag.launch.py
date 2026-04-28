from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

DATA_LOG_DIR = '/data_log'

TOPICS = [
    '/tf',
    '/tf_static',
    '/smip/map_surfel',
    '/smip/drone_path',
]

def generate_launch_description():
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='smip_bag_' + datetime.now().strftime('%Y%m%d_%H%M%S'),
        description='Name of the output bag (placed in /data_log/)',
    )

    record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', [DATA_LOG_DIR + '/', LaunchConfiguration('bag_name')],
            '--storage', 'sqlite3',
        ] + TOPICS,
        output='screen',
    )

    return LaunchDescription([
        bag_name_arg,
        record,
    ])
