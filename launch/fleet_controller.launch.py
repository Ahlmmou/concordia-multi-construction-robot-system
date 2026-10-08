from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share=get_package_share_directory('crs_multibot')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=os.path.join(share,'config','fleet_mission.yaml')),
        DeclareLaunchArgument('manual_navigation',default_value='false'),
        Node(package='crs_multibot',executable='fleet_mission_controller.py',output='screen',
             parameters=[LaunchConfiguration('config'),{'manual_navigation':LaunchConfiguration('manual_navigation')}])
    ])
