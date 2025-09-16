
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()

    # Single robot configuration
    robot = {'name': 'tb1', 'x_pose': '0', 'y_pose': '0', 'z_pose': 0.01}

    TURTLEBOT3_MODEL = 'burger'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )
    slam = LaunchConfiguration('slam')  # Add slam LaunchConfiguration
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    turtlebot3_multi_robot = get_package_share_directory('crs_multibot')
    
    package_dir = get_package_share_directory('s_turtlebot3_multi_robot')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    urdf = os.path.join(
        #turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
        # turtlebot3_multi_robot, 'urdf', 'wajueji_3.urdf'
        turtlebot3_multi_robot, 'urdf', 'Hilti_4.urdf'
    )

    world = os.path.join(
        turtlebot3_multi_robot, 'worlds', 'ev_9.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params_hilti.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Map server and lifecycle manager
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(package_dir, 'config', 'EVfloor-Floor-Plan-Level-1.yaml')}],
        remappings=remappings
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    ld.add_action(map_server)
    ld.add_action(map_server_lifecycle)
    ld.add_action(declare_slam_cmd)
    # Robot configuration
    # namespace = '/' + robot['name']
    namespace = ''
    # Robot state publisher
    turtlebot_state_publisher = Node(
        package='robot_state_publisher',
        namespace=namespace,
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'publish_frequency': 10.0}],
        remappings=remappings,
        arguments=[urdf],
    )

    # Spawn robot
    spawn_turtlebot3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(turtlebot3_multi_robot, 'models', 'Hilti_4', 'Hilti_4.sdf'),##'wajueji_3', 'wajueji_3.sdf'),
            '-entity', robot['name'],
            '-robot_namespace', namespace,
            '-x', robot['x_pose'], '-y', robot['y_pose'],
            '-z', str(robot['z_pose']), '-Y', '0.0',
            '-unpause',
        ],
        output='screen',
    )

    # Navigation bringup
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'namespace': namespace,
            'use_namespace': 'True',
            'map': '',
            'map_server': 'False',
            'params_file': params_file,
            'default_bt_xml_filename': os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            'autostart': 'true',
            'use_sim_time': use_sim_time,
            'log_level': 'warn'
        }.items()
    )
    # Initial pose
    message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
              robot['x_pose'] + ', y: ' + robot['y_pose'] + \
              ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'
    initial_pose_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + '/initialpose',
             'geometry_msgs/PoseWithCovarianceStamped', message],
        output='screen'
    )

    # RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'rviz_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'use_namespace': 'True',
            'rviz_config': rviz_config_file,
            'log_level': 'warn'
        }.items(),
        condition=IfCondition(enable_rviz)
    )

    # Drive node
    drive_turtlebot3 = Node(
        package='turtlebot3_gazebo',
        executable='turtlebot3_drive',
        namespace=namespace,
        output='screen',
        condition=IfCondition(enable_drive),
    )

    # Add actions
    ld.add_action(turtlebot_state_publisher)
    ld.add_action(spawn_turtlebot3)
    ld.add_action(bringup_cmd)
    ld.add_action(initial_pose_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(drive_turtlebot3)


    return ld
