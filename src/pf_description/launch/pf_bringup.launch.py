from ament_index_python.packages import get_package_share_directory
import os
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
import launch


def launch_setup(context, *args, **kwargs):
    package_name = 'pf_description'
    scanner_arg = LaunchConfiguration('scanner').perform(context)

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package=package_name).find(package_name)
    scanner_description_path = os.path.join(
        pkg_share, 'urdf', scanner_arg + '_world.urdf.xacro')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', scanner_description_path])}]
    )


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "scanner", default_value=TextSubstitution(text="r2000")),
        OpaqueFunction(function=launch_setup)
    ])
