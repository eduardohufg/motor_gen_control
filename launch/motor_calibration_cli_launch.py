import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(get_package_share_directory('motor_gen_control'), 'config', 'config.yaml')

    calibration_node = Node(
        name='motor_calibration_cli',
        package='motor_gen_control',
        executable='motor_calibration_cli',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([calibration_node])