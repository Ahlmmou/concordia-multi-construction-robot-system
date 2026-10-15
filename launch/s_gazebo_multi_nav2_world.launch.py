#!/usr/bin/env python3
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
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
import math
import yaml
from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()

    # One source for Gazebo world poses and AMCL map initial poses.
    # Their planar axes coincide in this world; odometry uses world coordinates.
    spawn_file = os.path.join(get_package_share_directory('crs_multibot'),
                              'config', 'fleet_spawn.yaml')
    with open(spawn_file) as stream:
        spawn_config = yaml.safe_load(stream)
    robots = spawn_config['robots']
    if [robot['name'] for robot in robots] != ['tb1', 'tb2', 'tb3', 'tb4', 'tb5']:
        raise ValueError('fleet_spawn.yaml must define exactly tb1 through tb5')
    for robot in robots:
        for key in ('x_pose', 'y_pose', 'z_pose', 'yaw'):
            robot[key] = str(robot[key])

    TURTLEBOT3_MODEL = 'burger'

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

    
    s_turtlebot3_multi_robot = get_package_share_directory('crs_multibot')

    nav_launch_dir = os.path.join(s_turtlebot3_multi_robot, 'launch', 'nav2_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            s_turtlebot3_multi_robot, 'rviz', 'fleet_spawn.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        s_turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    world = os.path.join(
        s_turtlebot3_multi_robot,
        'worlds', 'ev_9.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gazebo_gui', default='true')),
    )
    
     
    ld.add_action(DeclareLaunchArgument('simulation_localization', default_value='false'))
    ld.add_action(DeclareLaunchArgument('gazebo_gui', default_value='false'))
    ld.add_action(DeclareLaunchArgument('world', default_value=world))
    ld.add_action(DeclareLaunchArgument('map_yaml', default_value=os.path.join(s_turtlebot3_multi_robot,'config','EVfloor9_outer_only.yaml')))
    ld.add_action(DeclareLaunchArgument('enable_legacy_formation', default_value='false'))
    ld.add_action(DeclareLaunchArgument('velocity_output_topic', default_value='cmd_vel'))
    ld.add_action(DeclareLaunchArgument('auxiliary_navigation', default_value='true'))
    ld.add_action(DeclareLaunchArgument('leader_params', default_value=os.path.join(
        s_turtlebot3_multi_robot, 'params', 'nav2_params_hilti.yaml')))
    ld.add_action(DeclareLaunchArgument('auxiliary_model', default_value=os.path.join(
        s_turtlebot3_multi_robot, 'models', 'turtlebot3_burger', 'model_scaled_x2.sdf')))
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            s_turtlebot3_multi_robot, 'launch', 'fleet_spawn_view.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'rviz_config': rviz_config_file}.items(),
        condition=IfCondition(enable_rviz)))
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    # map_server=Node(package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'),
    #                  },],
    #     remappings=remappings)
    goal_listener_and_sender = Node(
            package='crs_multibot',
            executable='goal_listener_and_sender.py',
            condition=IfCondition(LaunchConfiguration('enable_legacy_formation', default='false')),
            output='screen',
        )
    
    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': LaunchConfiguration('map_yaml')}],
            remappings=remappings
    )
        #map_file = os.path.join(get_package_share_directory('nav2_project'), 'config', 'turtlebot3_house.yaml')

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)
    # Mission supervision owns formation goals; legacy forwarding is opt-in.
    ld.add_action(goal_listener_and_sender)

    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = [ '/' + robot['name'] ]
        print ("namespace = ", namespace)

        if namespace == [ '/tb1' ]:
            print ("process conducted")
            
            urdf = os.path.join(
                # s_turtlebot3_multi_robot, 'urdf', 'turtlebot3_tb1_' + TURTLEBOT3_MODEL + '.urdf'
                # s_turtlebot3_multi_robot, 'urdf', 'wajueji_3.urdf'     
                s_turtlebot3_multi_robot, 'urdf', 'Hilti_4.urdf'   
                )
            
            model_sdf = os.path.join(
                # s_turtlebot3_multi_robot, 'models', 'wajueji_3', 'wajueji_3.sdf'
                s_turtlebot3_multi_robot, 'models', 'Hilti_4', 'Hilti_4.sdf'
                )

            params_file = LaunchConfiguration('leader_params')

        else :
            urdf = os.path.join(
                # s_turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
                s_turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '_scaled_x2.urdf'
                )
            
            model_sdf = LaunchConfiguration('auxiliary_model')
        
            params_file = os.path.join(
                s_turtlebot3_multi_robot, 'params', 'nav2_params.yaml')
        
        
        if robot['name'] == 'tb1':
            params_file = RewrittenYaml(source_file=params_file, root_key='',
                param_rewrites={'default_nav_to_pose_bt_xml': os.path.join(
                    s_turtlebot3_multi_robot, 'config', 'hilti_navigation.xml')},
                convert_types=True)

        # Initialize AMCL from the simulation spawn pose when it activates.
        # This avoids losing one-shot initialpose messages during slow startup.
        params_file = RewrittenYaml(source_file=params_file, root_key='',
            param_rewrites={'amcl.ros__parameters.set_initial_pose': 'true',
                            'amcl.ros__parameters.initial_pose.x': robot['x_pose'],
                            'amcl.ros__parameters.initial_pose.y': robot['y_pose'],
                            'amcl.ros__parameters.initial_pose.yaw': robot['yaw'],
                            'amcl.ros__parameters.tf_broadcast': PythonExpression(["'", LaunchConfiguration('simulation_localization'), "'.lower() != 'true'"])},
            convert_types=True)

        # Gazebo world odometry was verified to share the map axes and origin.
        ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
            name='simulation_map_to_odom', namespace=namespace,
            arguments=['--frame-id','map','--child-frame-id','odom'],
            remappings=remappings, parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('simulation_localization'))))

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0,}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', model_sdf,
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', robot['z_pose'], '-Y', robot['yaw'],
                '-unpause',
            ],
            output='screen',
        )

        # scan_filter_node = Node(
        #     package='crs_multibot',
        #     executable='scan_filter.py',
        #     namespace=namespace,
        #     output='screen',
        #     parameters=[{'max_fraction_max_range': 0.6}],
        #     remappings=[
        #         ('scan', 'scan'),
        #         ('scan_filtered', 'scan_filtered')
        #     ]
        # )

        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_respawn': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn',
                                    'velocity_output_topic': LaunchConfiguration('velocity_output_topic', default='cmd_vel')}.items(),
                condition=None if robot['name'] == 'tb1' else
                IfCondition(LaunchConfiguration('auxiliary_navigation')))

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            ld.add_action(bringup_cmd)
            # ld.add_action(scan_filter_node)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
			spawn_turtlebot3_burger,
			turtlebot_state_publisher,
			TimerAction(
				period=8.0,
				actions=[bringup_cmd]
			),
		    ],
                            # scan_filter_node,
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_turtlebot3_burger
    ######################

    ######################
    # Start rviz nodes and drive nodes after the last robot is spawned
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # Create a initial pose topic publish call
        yaw = float(robot['yaw'])
        message = yaml.safe_dump({
            'header': {'frame_id': 'map'},
            'pose': {'pose': {
                'position': {'x': float(robot['x_pose']),
                             'y': float(robot['y_pose']), 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0,
                                'z': math.sin(yaw / 2.0),
                                'w': math.cos(yaw / 2.0)}}}},
            default_flow_style=True)

        initial_pose_cmd = ExecuteProcess(
            condition=None if robot['name'] == 'tb1' else
            IfCondition(LaunchConfiguration('auxiliary_navigation')),
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        # Publish localization initial poses after all robots are spawned.
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, drive_turtlebot3_burger],
            )
        )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        last_action = initial_pose_cmd

        ld.add_action(post_spawn_event)
    ######################

    return ld
