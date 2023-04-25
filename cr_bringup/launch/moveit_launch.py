import os
import yaml
from ament_index_python import get_package_share_directory

from cr_bringup.utils import declare_argument, forward_argumemt, get_urdf, get_controller
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def load_file(file_path):
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    robot_model = LaunchConfiguration("robot_model").perform(context)
    moveit_dir = os.path.join(
        get_package_share_directory("cr_bringup"),
        "config",
        robot_model)

    if (not os.path.isdir(moveit_dir)):
        raise Exception(f"Robot model {robot_model} is currently not supported")

    # https://github.com/ros-planning/moveit2_tutorials/blob/foxy/doc/quickstart_in_rviz/launch/demo.launch.py
    robot_description = {"robot_description": get_urdf(robot_model)}

    robot_description_semantic = {
        "robot_description_semantic": load_file(
            os.path.join(moveit_dir, f"{robot_model}.srdf")
        )
    }

    kinematics_yaml = load_yaml(
            os.path.join(moveit_dir, "kinematics.yaml")
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "planning_pipelines": {
            "pipeline_names": ["ompl"],
        },
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_pipeline_config["ompl"].update(
        load_yaml(
            os.path.join(moveit_dir, "ompl_planning.yaml")
        )
    )

    # Trajectory Execution Functionality
    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml(
            os.path.join(moveit_dir, "controller.yaml")
        ),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'warn']
    )

    # ROS2 Control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {"robot_description": get_urdf(robot_model)},
                get_controller(robot_model)],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "manipulator_trajectory_controller",
            "--controller-manager",
            "/controller_manager"],
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
                'default.rviz')],
            '--ros-args',
            '--log-level',
            'warn'
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    cr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cr_bringup"),
                "launch",
                "cr_robot_launch.py")
        ),
        launch_arguments=forward_argumemt()
    )

    cr_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cr_bringup"),
                "launch",
                "cr_robot_launch.py")
        ),
        launch_arguments=forward_argumemt()
    )

    return [
        run_move_group_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        cr_robot_launch,
        rviz]



def generate_launch_description():
    argument = declare_argument()

    return LaunchDescription(argument + [OpaqueFunction(function=launch_setup)])
