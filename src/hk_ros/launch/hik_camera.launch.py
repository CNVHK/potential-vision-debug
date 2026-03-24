from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')
    
    return LaunchDescription([
        ComposableNodeContainer(
            name='hik_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='hik_camera',
                    plugin='hik_camera::HikCameraNode',
                    name='hik_camera',
                    parameters=[
                        params_file,
                        {'camera_info_url': 'package://hik_camera/config/camera_info.yaml',
                         'use_sensor_data_qos': False}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
            ros_arguments=['--ros-args', '--log-level', 'info'],
        )
    ])