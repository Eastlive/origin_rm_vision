import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('virtual_serial_driver'), 'config', 'serial_driver.yaml')

    virtual_serial_driver_node = Node(
        package='virtual_serial_driver',
        executable='virtual_serial_driver_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([virtual_serial_driver_node])
