import launch
import launch_ros
from ament_index_python import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():

    sam_bot_odom_path=get_package_share_directory('sam_bot_odom_py')
    urdf_path=os.path.join(sam_bot_odom_path,'urdf','sam_bot.urdf')
    rviz_config_path=os.path.join(sam_bot_odom_path,'config','rviz.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config_path]
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])