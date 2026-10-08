"""Verified world/map identity for the existing five-robot Gazebo session.
Disable AMCL tf_broadcast on every robot before using this runtime launch.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='tf2_ros',executable='static_transform_publisher',
        name='simulation_map_to_odom',namespace=r,
        arguments=['--frame-id','map','--child-frame-id','odom'],
        parameters=[{'use_sim_time':True}],
        remappings=[('/tf','tf'),('/tf_static','tf_static')])
        for r in ('tb1','tb2','tb3','tb4','tb5')])
