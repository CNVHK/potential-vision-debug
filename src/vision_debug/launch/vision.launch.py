import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vision_share_dir = get_package_share_directory('vision_debug')
    config_file_path = os.path.join(vision_share_dir, 'config', 'vision_params.yaml')
    hik_camera_node = Node(
        package='hik_camera',
        executable='hik_camera_node', 
        namespace='hik_camera',
        name='hik_camera',
        output='both',
        respawn=True,           
        respawn_delay=2.0,    
        parameters=[{
            'use_sensor_data_qos': True, # 确保开启 Best Effort 传输
            # 如果你有相机的 yaml 配置文件，也可以在这里传进去
        }]
    )


    vision_node = Node(
        package='vision_debug',
        executable='vision_node',
        name='vision_node',
        output='screen', 
        # prefix=['xterm -e gdb -ex run --args'],
        # prefix=['perf record -e cpu-clock --call-graph dwarf -o perf_vision.data -- '],
        parameters=[{
            'config_path': config_file_path
        }]
    )


    # mavlink_node = Node(
    #     package='serial_mav',       
    #     executable='serial_mav_node',
    #     name='serial_mav_node',       
    #     output='screen'
    # )


    return LaunchDescription([
        hik_camera_node,
        vision_node,
        # mavlink_node
    ])
