import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('rm_flatness_nav'), 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('csv_path', default_value='motor_voltage_log.csv'),
        DeclareLaunchArgument('tag_id', default_value='3'),
        DeclareLaunchArgument('tag_size', default_value='0.10'),
        Node(package='rm_flatness_nav', executable='flatness_nav', name='flatness_nav',
             output='screen',
             parameters=[cfg, {'csv_path': LaunchConfiguration('csv_path'),
                               'tag_id': ParameterValue(LaunchConfiguration('tag_id'), value_type=int),
                               'tag_size': ParameterValue(LaunchConfiguration('tag_size'), value_type=float)}]),
    ])
