from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import launch
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    
    nav2_bringup_path=get_package_share_directory('nav2_bringup')
    rviz_config_path=os.path.join(nav2_bringup_path,'rviz','nav2_default_view.rviz')
    sam_bot_nav2_path=get_package_share_directory('sam_bot_nav')
    nav2_param_path=launch.substitutions.LaunchConfiguration(
        'params_file',default=os.path.join(sam_bot_nav2_path,'config','nav2_params.yaml')
    )
    map_yaml_path=launch.substitutions.LaunchConfiguration(
        'map',default=os.path.join(sam_bot_nav2_path,'map','room2.yaml')
    )

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config_path],
        parameters=[{'use_sim_time':False}],
        output='screen'
    )
  
    
    
    return  LaunchDescription([
        
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_path,'/launch','/bringup_launch.py']
            ),
            launch_arguments={
                'map': map_yaml_path,
                'params_file': nav2_param_path,
                'use_sim_time': 'false' 
            }.items(),
        ),
        rviz_node
    ])
           