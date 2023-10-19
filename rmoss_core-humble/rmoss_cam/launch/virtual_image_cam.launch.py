# Copyright 2021 RoboMaster-OSS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    image_path = os.path.join(get_package_share_directory('rmoss_cam'), 'resource', 'test.jpg')
    video_path = os.path.join(get_package_share_directory('rmoss_cam'), 'resource', 'bubing.mp4')
    calibration_path = 'package://rmoss_cam/resource/image_cam_calibration.yaml'
    config_path = os.path.join(get_package_share_directory('rmoss_cam'), 'config', 'cam_params.yaml')

    print(config_path)
    virtual_cam = Node(
        package='rmoss_cam',
        executable='virtual_cam',
        name='virtual_cam',
        parameters=[config_path,
            {'image_path': image_path,
             'video_path': video_path,
             'camera_info_url': calibration_path,
             'fps': 30,
             }],
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(virtual_cam)
    return ld
