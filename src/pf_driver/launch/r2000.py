import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_name = 'r2000_params.yaml'

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pf_driver'),
        'config',
        config_name
        )

    node = Node(
        package='pf_driver',
        name='ros_main',
        executable='ros_main',
        output='screen',
        parameters=[config],
        remappings=[
            ('/scan', '/sensor/lidar/front')
        ]
    )
    ld.add_action(node)
    return ld


