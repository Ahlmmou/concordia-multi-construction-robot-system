#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    robot = {'name': 'tb1', 'x_pose': '0', 'y_pose': '0', 'z_pose': 0.01}

    TURTLEBOT3_MODEL = 'waffle'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')

    package_dir = get_package_share_directory('turtlebot3_multi_robot')

    gazebo_ros = get_package_share_directory('gazebo_ros')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    world = os.path.join(
        get_package_share_directory('turtlebot3_multi_robot'),
        'worlds', 'ev9_floor_plan_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    
     
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # map_server=Node(package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'),
    #                  },],
    #     remappings=remappings)

    
    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    # remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Spawn turtlebot3 instances in gazebo
        # Create state publisher node for that instance
    turtlebot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'publish_frequency': 10.0,}],
        arguments=[urdf],
    )

        # Create spawn call
    spawn_turtlebot3_burger = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(turtlebot3_multi_robot,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
            '-entity', robot['name'],
            '-x', robot['x_pose'], '-y', robot['y_pose'],
            '-z', '0.01', '-Y', '0.0',
            '-unpause',
        ],
        output='screen',
    )

    navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )
    #changed turtlebot3_multi_robot -> nav2_bringup
    nav2_params = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params',
        'nav2_params.yaml'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'use_sim_time' : 'true',
            'params_file' : nav2_params
        }.items()
    )

    delayed_nav = TimerAction(
        period=15.0,
        actions=[nav2_bringup]
    )
    
    async_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # , 'slam_params_file': params_file_path
    slam_tool_box = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(async_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    delayed_slam = TimerAction(
        period=15.0,
        actions=[slam_tool_box]
    )


    rviz_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    rviz_delay = TimerAction(
        period=10.0,
        actions=[rviz_node]
    )



            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
    ld.add_action(turtlebot_state_publisher)
    ld.add_action(spawn_turtlebot3_burger)
    ld.add_action(delayed_nav)
    ld.add_action(delayed_slam)
    ld.add_action(rviz_delay)
    ######################
    return ld
