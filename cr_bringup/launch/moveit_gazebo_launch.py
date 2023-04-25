import os
from ament_index_python import get_package_share_directory
from cr_bringup.utils import declare_argument, forward_argumemt
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    argument = declare_argument()

    node_list = []

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cr_simulation"),
                "launch",
                "sim_launch.py")
        ),
    )
    node_list.append(gazebo_launch)

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cr_bringup"),
                "launch",
                "moveit_launch.py")
        ),
        launch_arguments=forward_argumemt()
    )

    pcl_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cr_planning"),
                "launch",
                "pcl_obstacle_launch.py")
        ),
        launch_arguments=forward_argumemt()
    )

    node_list.append(
        TimerAction(
            period=2.0,
            actions=[moveit_launch]
        )
    )

    node_list.append(
        TimerAction(
            period=5.0,
            actions=[pcl_moveit_launch]
        )
    )
    return LaunchDescription(
        argument + node_list
    )
