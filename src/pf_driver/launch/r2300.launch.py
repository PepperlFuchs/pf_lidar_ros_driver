import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = 'r2300_params.yaml'
    correction_params_file = 'correction_params.yaml'

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pf_driver'),
        'config',
        config_file
        )
    correction_params = os.path.join(
        get_package_share_directory('pf_driver'),
        'config',
        correction_params_file
        )

    node = Node(
        package='pf_driver',
        name='ros_main',
        executable='ros_main',
        output='screen',
        parameters=[config, correction_params]
    )

    include_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("pf_description"), '/launch', '/pf_bringup.launch.py'],
            launch_arguments = {'scanner': 'r2300'}
        )
    )
    ld.add_action(node)
    return ld
