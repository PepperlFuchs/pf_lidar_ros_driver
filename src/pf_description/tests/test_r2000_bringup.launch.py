from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import os
import pytest

@pytest.mark.rostest
def generate_test_description():

    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("pf_description"), "launch"),
                "/pf_bringup.launch.py",
            ]
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=5.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return (LaunchDescription([launch_desc, ready_to_test]), {})