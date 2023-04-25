import os
from ament_index_python import get_package_share_directory

from cr_bringup.utils import declare_argument, forward_argumemt
from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    argument = declare_argument()

    cr_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cr_bringup"),
                "launch",
                "cr_robot_launch.py")
        ),
        launch_arguments=forward_argumemt()
    )

    # https://github.com/ros/joint_state_publisher
    joint_state_gui_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            [os.path.join(
                get_package_share_directory('cr_description'),
                'rviz',
                'default.rviz')]],
    )

    return LaunchDescription(
        argument + [
            joint_state_gui_publisher,
            cr_robot_launch,
            rviz
        ]
    )
