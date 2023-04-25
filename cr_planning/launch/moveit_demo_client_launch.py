import os
from ament_index_python import get_package_share_directory

from cr_bringup.utils import declare_argument, get_urdf, get_controller, forward_argumemt

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    default_obstacle_config_file = os.path.join(
        get_package_share_directory("cr_planning"),
        "config",
        "obstacle.yaml"
    )

    default_planning_config_file = os.path.join(
        get_package_share_directory("cr_planning"),
        "config",
        "planning.yaml"
    )

    argument = [
        DeclareLaunchArgument(
            name="obstacle_config_file",
            default_value=default_obstacle_config_file,
            description="configuration file for obstacle, user can replace it with launch argument"
        ),
        DeclareLaunchArgument(
            name="planning_config_file",
            default_value=default_planning_config_file,
            description=("ROS2 configuration file for FK IK Planning input"
                + ", user can replace it with launch argument")
        )
    ]

    moveit_demo_client = Node(
        package='cr_planning',
        executable='moveit_demo_client.py',
        output='both',
        parameters=[
            {"obstacle_config_path": LaunchConfiguration("obstacle_config_file")},
            LaunchConfiguration("planning_config_file")
        ]
    )

    return LaunchDescription(argument + [moveit_demo_client])
