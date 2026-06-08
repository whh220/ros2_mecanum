from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='delta2a_lidar',
            executable='lidar_node',
            name='delta2a_lidar',
            output='screen',
            parameters=[{
                'port':     '/dev/ttyUSB1',
                'baud':     230400,
                'frame_id': 'lidar_link',
                'topic':    'scan',
            }],
        )
    ])
