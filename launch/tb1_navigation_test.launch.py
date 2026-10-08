"""Step 2: one Nav2 stack, stationary auxiliaries, and one tb1 RViz viewer."""
import os
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('crs_multibot')
    # Keep auxiliary sensors/collisions intact, but prevent passive physics drift.
    tree = ET.parse(os.path.join(share, 'models', 'turtlebot3_burger', 'model_scaled_x2.sdf'))
    model = tree.getroot().find('model')
    static = model.find('static')
    if static is None:
        static = ET.SubElement(model, 'static')
    static.text = 'true'
    with tempfile.NamedTemporaryFile(prefix='step2_auxiliary_', suffix='.sdf', delete=False) as f:
        # spawn_entity parses text with lxml; omit an encoding declaration.
        tree.write(f, encoding='utf-8', xml_declaration=False)
        auxiliary_model = f.name
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch', 's_gazebo_multi_nav2_world.launch.py')),
            launch_arguments={'enable_rviz': 'false', 'enable_drive': 'false',
                              'enable_legacy_formation': 'false',
                              'auxiliary_navigation': 'false',
                              'leader_params': os.path.join(share, 'params', 'nav2_params_tb1_test.yaml'),
                              'auxiliary_model': auxiliary_model,
                              'gazebo_gui': LaunchConfiguration('gazebo_gui')}.items()),
        Node(package='crs_multibot', executable='fleet_rviz_relay.py',
             parameters=[{'use_sim_time': True, 'data_robots': ['tb1'],
                          'stationary_auxiliaries': True}], output='screen'),
        Node(package='rviz2', executable='rviz2', namespace='tb1', name='rviz_navigation_test',
             arguments=['-d', os.path.join(share, 'rviz', 'tb1_navigation_test.rviz')],
             parameters=[{'use_sim_time': True}],
             remappings=[('/tf', '/fleet_visualization/tf'),
                         ('/tf_static', '/fleet_visualization/tf_static'),
                         ('navigate_to_pose', '/tb1/navigate_to_pose'),
                         ('/navigate_to_pose', '/tb1/navigate_to_pose'),
                         ('/initialpose', '/tb1/initialpose')], output='screen'),
    ])
