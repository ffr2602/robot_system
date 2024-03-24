

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    lidar_front = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_front',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
                'topic_name': 'lidar_front_scan',
                'inverted': False
        }]
    )

    lidar_back = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_back',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
                'topic_name': 'lidar_back_scan',
                'inverted': False
        }]
    )

    return LaunchDescription([
        lidar_front,
        lidar_back
    ])
