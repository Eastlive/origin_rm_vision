import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml

def generate_launch_description():
    param_path = os.path.join(get_package_share_directory(
        "hero_bringup"), "config/node_params.yaml")

    with open(param_path, 'r') as f:
        virtual_serial_params = yaml.safe_load(f)['/virtual_serial_driver']['ros__parameters']
    with open(param_path, 'r') as f:
        virtual_outpost_params = yaml.safe_load(f)['/virtual_outpost']['ros__parameters']
    with open(param_path, 'r') as f:
        tracker_params = yaml.safe_load(f)['/armor_tracker']['ros__parameters']   
    
    package_name = 'hero_description'
    urdf_name = "hero_description.urdf"
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    rviz_config_path = os.path.join(pkg_share, f'launch/{"view_model.rviz"}')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
        )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        )
    # 创建容器
    rm_container = Node(
        name='rm_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )
    

    load_compose = LoadComposableNodes(
        target_container='rm_container',
        composable_node_descriptions=[
            ComposableNode(
                package='virtual_serial_driver',
                plugin='virtual_serial_driver::VirtualSerialDriver',
                name='virtual_serial_driver',
                parameters=[virtual_serial_params],
            ),
            ComposableNode(
                package='outpost_detector',
                plugin='rm_auto_aim::OutpostDetectorNode',
                name='outpost_detector',
                parameters=[virtual_outpost_params],
            ),            
            ComposableNode(
                package='armor_tracker',
                plugin='rm_auto_aim::ArmorTrackerNode',
                name='armor_tracker',
                parameters=[tracker_params],
            )  
        ]
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(load_compose)
    ld.add_action(rm_container)
    ld.add_action(rviz2_node)
    return ld
