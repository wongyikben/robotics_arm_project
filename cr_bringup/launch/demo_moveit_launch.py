import os
from ament_index_python import get_package_share_directory
from cr_bringup.utils import declare_argument, forward_argumemt
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    argument = declare_argument()

    node_list = []

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cr_bringup"),
                "launch",
                "moveit_launch.py")
        ),
        launch_arguments=forward_argumemt()
    )
    node_list.append(moveit_launch)

    moveit_demo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cr_planning"),
                "launch",
                "moveit_demo_client_launch.py")
        ),
        launch_arguments=forward_argumemt()
    )
    node_list.append(
        TimerAction(
            period=5.0,
            actions=[moveit_demo_client_launch]
        )
    )

    return LaunchDescription(
        argument + node_list
    )
