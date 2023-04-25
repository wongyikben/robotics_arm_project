import os
from ament_index_python import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable


def declare_argument():
    argument = [
        DeclareLaunchArgument(
            name="robot_model",
            default_value="ur10",
            description="Robot model to be used"
        )
    ]
    return argument

def forward_argumemt():
    return [
        ("robot_model", LaunchConfiguration("robot_model"))
    ]

def get_urdf(robot_model: str):
    # https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Substitutions.html
    xacro_path = os.path.join(
        get_package_share_directory("cr_description"),
        "xacro",
        robot_model + ".xacro")

    if (not os.path.isfile(xacro_path)):
        raise Exception(f"Robot model {robot_model} is currently not supported")

    robot_desc = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_path
    ])

    return robot_desc

def get_controller(robot_model: str):
    controller_path = os.path.join(
        get_package_share_directory("cr_bringup"),
        "config",
        robot_model,
        "ros2_control.yaml")

    if (not os.path.isfile(controller_path)):
        raise Exception(f"Robot model {robot_model} is currently not supported")

    return controller_path
