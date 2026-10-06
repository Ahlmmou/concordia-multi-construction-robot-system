"""Display all five existing robots in one map frame, without motion control."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('crs_multibot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'rviz', 'fleet_spawn.rviz')),
        Node(package='crs_multibot', executable='fleet_rviz_relay.py',
             parameters=[{'use_sim_time': use_sim_time}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz_fleet_spawn',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[('/tf', '/fleet_visualization/tf'),
                         ('/tf_static', '/fleet_visualization/tf_static')],
             output='screen'),
    ])
