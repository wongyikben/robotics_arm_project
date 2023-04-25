import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():


    sdf_path = os.path.join(
        get_package_share_directory("cr_simulation"),
        "world",
        "depth_camera.sdf"
    )

    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_ign_gazebo"),
                "launch",
                "ign_gazebo.launch.py")
        ),
        launch_arguments={
            "ign_args": f"{sdf_path} -r --render-engine ogre"
        }.items()
    )

    ros_ign_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/model/ur10/joint/shoulder_pan_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/ur10/joint/shoulder_lift_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/ur10/joint/elbow_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/ur10/joint/wrist_1_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/ur10/joint/wrist_2_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
            "/model/ur10/joint/wrist_3_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
        ],
        output="both"
    )

    robot_sdf_path = os.path.join(
        get_package_share_directory("cr_simulation"),
        "model",
        "ur10.sdf"
    )
    robot_model = Node(package='ros_ign_gazebo', executable='create',
        arguments=['-name', 'ur10', '-file', robot_sdf_path],
        output='screen')

    joint_state_adaptor = Node(
        package="cr_simulation",
        executable="joint_state_adaptor.py",
        output="screen"
    )

    camera_frame_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.5", "0", "2.0", "0", "1.57", "0", "world", "depth_camera/link/depth_camera1"],
        output="both"
    )

    node_list = [
        sim_world,
        ros_ign_bridge,
        camera_frame_publisher,
        joint_state_adaptor,
        robot_model
    ]

    return LaunchDescription(node_list)
