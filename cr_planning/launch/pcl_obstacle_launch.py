from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    moveit_client = Node(
        package='cr_planning',
        executable='pcl_obstacle_moveit_client.py',
        output='both',
    )

    pcl_filter = Node(
        package='cr_planning',
        executable='pcl_filter',
        output='both',
    )

    return LaunchDescription([moveit_client, pcl_filter])
