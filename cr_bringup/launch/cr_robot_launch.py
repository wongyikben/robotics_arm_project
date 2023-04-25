import os
from ament_index_python import get_package_share_directory

from cr_bringup.utils import declare_argument, get_urdf, get_controller, forward_argumemt

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # https://answers.ros.org/question/397123/
    robot_model = LaunchConfiguration("robot_model").perform(context)

    robot_desc = get_urdf(robot_model)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{"robot_description": robot_desc}])

    return [
        robot_state_publisher,
    ]


def generate_launch_description():
    argument = declare_argument()
    return LaunchDescription(argument + [OpaqueFunction(function=launch_setup)])
