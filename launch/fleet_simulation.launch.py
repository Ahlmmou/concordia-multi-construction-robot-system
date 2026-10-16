"""Exactly five robots; disable legacy controls and route Nav2 through the gate."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_directory('crs_multibot')
    return LaunchDescription([
        DeclareLaunchArgument('mission_config',default_value=os.path.join(share,'config','fleet_mission.yaml')),
        DeclareLaunchArgument('world',default_value=os.path.join(share,'worlds','ev_9.world')),
        DeclareLaunchArgument('map_yaml',default_value=os.path.join(share,'config','EVfloor9_formation_aligned.yaml')),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('gazebo_gui',default_value='false'),
        DeclareLaunchArgument('simulation_localization',default_value='true'),
        DeclareLaunchArgument('controller',default_value='true'),
        DeclareLaunchArgument('manual_navigation',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','s_gazebo_multi_nav2_world.launch.py')),
            launch_arguments={'world':LaunchConfiguration('world'),'map_yaml':LaunchConfiguration('map_yaml'),'simulation_localization':LaunchConfiguration('simulation_localization'),'gazebo_gui':LaunchConfiguration('gazebo_gui'),'enable_drive':'false','enable_rviz':'false','enable_legacy_formation':'false','velocity_output_topic':'cmd_vel_uncoordinated'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','fleet_controller.launch.py')),
            condition=IfCondition(LaunchConfiguration('controller')),
            launch_arguments={'config':LaunchConfiguration('mission_config'),'manual_navigation':LaunchConfiguration('manual_navigation')}.items()),
        Node(package='crs_multibot',executable='fleet_rviz_relay.py',parameters=[{'use_sim_time':True}],output='screen',condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2',executable='rviz2',arguments=['-d',os.path.join(share,'rviz','fleet_simulation.rviz')],
             parameters=[{'use_sim_time':True}],remappings=[('/tf','/fleet_visualization/tf'),('/tf_static','/fleet_visualization/tf_static')],
             output='screen',condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='crs_multibot',executable='fleet_live_status.py',
             output='screen',condition=IfCondition(LaunchConfiguration('rviz')))
    ])
