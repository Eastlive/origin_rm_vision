from launch import LaunchDescription
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_path = os.path.join(get_package_share_directory("hero_bringup"), "config", "test_outpost.yaml")
    image_path = os.path.join(get_package_share_directory("hero_bringup"), "resource", "yunhao2.png")
    video_path = os.path.join(get_package_share_directory("hero_bringup"), "resource", "outpost.avi")

    camera_node = Node(
        package='rmoss_cam',
        executable='virtual_cam',
        name='virtual_cam',
        output='screen',
        parameters=[param_path,
                    {
                        'camera_name': "",
                        'image_path': image_path,
                        'video_path': video_path,
                        'camera_info_url': 'package://hero_bringup/config/virtual_camera_info.yaml',
                        'fps': 30
                    }]
    )

    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector',
        output='screen',
        emulate_tty=True,
        parameters=[param_path]
    )

    tracker_node = Node(
        package='armor_tracker',
        executable='armor_tracker_node',
        name='armor_tracker',
        output='screen',
        emulate_tty=True,
        parameters=[param_path]
    )
    
    ld = LaunchDescription([
        camera_node,
        detector_node,
        tracker_node
    ])

    return ld