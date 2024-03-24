import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot_system'
    config = os.path.join(get_package_share_directory(package_name), 'config', 'bno055_params.yaml')
    sensor_launch = Node(
        package='bno055',
        executable='bno055',
        parameters=[config]
    )
    
    return LaunchDescription([
        sensor_launch
    ])