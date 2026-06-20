from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import launch
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    sam_bot_odom_py_path = get_package_share_directory('sam_bot_odom_py')
    nav2_bringup_path=get_package_share_directory('nav2_bringup')
    ekf_config_path = os.path.join(sam_bot_odom_py_path, 'config', 'ekf.yaml')
    rviz_config_path=os.path.join(nav2_bringup_path,'rviz','nav2_default_view.rviz')

    sam_bot_nav2_path=get_package_share_directory('sam_bot_nav')
    nav2_param_path=launch.substitutions.LaunchConfiguration(
        'params_file',default=os.path.join(sam_bot_nav2_path,'config','nav2_params.yaml')
    )
    map_yaml_path=launch.substitutions.LaunchConfiguration(
        'map',default=os.path.join(sam_bot_nav2_path,'map','room.yaml')
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
    )

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config_path],
        parameters=[{'use_sim_time':False}],
        output='screen'
    )
    odometry_node = Node(
        package= 'sam_bot_odom_py',
        executable='OdometryNode',
        name='odometry_node',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }]
    )
    stm32_node= Node(
        package= 'stm32',
        executable= 'communication',
        name= 'stm32_node',
        output='screen',
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )
    lidar_node=Node(
        package='lidar',
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
    
    return  LaunchDescription([
        stm32_node,
        lidar_node,
        odometry_node,
        # robot_localization_node,
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_path,'/launch','bringup_launch.py']
            ),
            launch_arguments={
                'map': map_yaml_path,
                'params_file': nav2_param_path,
                'use_sim_time': 'false' 
            }.items(),
        ),
        joint_state_publisher_node,
        rviz_node
    ])
           