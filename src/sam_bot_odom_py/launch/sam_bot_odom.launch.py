from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    sam_bot_odom_py_path = get_package_share_directory('sam_bot_odom_py')
    ekf_config_path = os.path.join(sam_bot_odom_py_path, 'config', 'ekf.yaml')
    urdf_path=os.path.join(sam_bot_odom_py_path,'urdf','sam_bot.urdf')
    rviz_config_path=os.path.join(sam_bot_odom_py_path,'config','rviz.rviz')
    

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
    wheel_odom = Node(
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
    laser_odom=Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/laser_odom',
            'publish_tf' : True,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 20.0}],
        )
    
    return  LaunchDescription([
        stm32_node,
        lidar_node,
        wheel_odom,
        # laser_odom,
        # robot_localization_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
           