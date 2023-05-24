import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = 'r2300_params.yaml'

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pf_driver'),
        'config',
        config_file
        )

    node = Node(
        package='pf_driver',
        name='ros_main',
        executable='ros_main',
        output='screen',
        parameters=[config]
    )

    include_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("pf_description"), '/launch', '/pf_bringup.launch.py'
        ]),
        launch_arguments = {'scanner': 'r2300'}.items()
    )

    ld.add_action(node)
    ld.add_action(include_desc)
    return ld
