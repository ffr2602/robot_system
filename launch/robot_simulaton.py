import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot_system'
    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot_simulation.xacro')
    robot_dec = xacro.process_file(xacro_file).toxml()
    params = {'robot_description': robot_dec}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gazebo_params_maps = os.path.join(get_package_share_directory(package_name),'world','map_maze.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
        launch_arguments={
            'extra_gazebo_args':'--ros-args --params-file ' + gazebo_params_file,
            'world': gazebo_params_maps
            }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic','robot_description','-entity','my_bot'],
        output='screen'
    )

    robot_controller = Node(
        package='robot_system',
        executable='robot_control_simulation.py',
        name='control_robot'
    )

   

    return LaunchDescription([
        node_robot_state_publisher,
        robot_controller,
        gazebo,
        spawn_entity
    ])