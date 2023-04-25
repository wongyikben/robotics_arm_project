#!/usr/bin/python3
from typing import List
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import ros2_numpy

from builtin_interfaces.msg import Duration

from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints

from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.srv import GetMotionPlan

from moveit_msgs.action import ExecuteTrajectory


class PCLObstacleMoveitClient():
    def __init__(self, node: Node):
        self.node = node
        self.timer = self.node.create_timer(1.0, self.timer_callback)

        self.state_index = 0

        self.target_joint_states = [
            [
                -1.576088989920999,
                -1.0594571091129554,
                2.2387201086645354,
                1.9622915170649642,
                1.5760578757257548,
                -3.141538928885743,
            ], [
                0.0,
                -0.8882256492536663,
                1.3918496680175276,
                0.5851303396101659,
                0.0,
                -1.0888861819956226,
            ], [
                0.9568092428860427,
                -1.2761569683640246,
                2.308333966983247,
                -1.005497141784846,
                0.8592613624742048,
                -0.2729011513584933,
            ]
        ]

        self.joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
        )

        self.point_cloud = None
        self.point_cloud_cb_count = 0
        self.point_cloud_sub = self.node.create_subscription(
            PointCloud2,
            "/filtered/points",
            self.point_cloud_cb,
            10
        )

        self.collision_pub = self.node.create_publisher(
            CollisionObject,
            "/collision_object",
            10
        )

        self.planned_trajectory = None
        self.plan_path_future = None
        self.plan_path_cli = self.node.create_client(
            GetMotionPlan,
            "/plan_kinematic_path"
        )

        self.exec_traj_future = None
        self.exec_traj_cli = ActionClient(
            self.node,
            ExecuteTrajectory,
            '/execute_trajectory'
        )

    def joint_state_cb(self, msg: JointState):
        self.joint_state = msg

    def timer_callback(self):
        if (self.plan_path_future is None
            and self.exec_traj_future is None
            and self.joint_state is not None
            and self.point_cloud is not None):
            self.send_plan_path_request()
        else:
            self.future_handling()

    def point_cloud_cb(self, msg: PointCloud2):
        """
        Due to the lack of documentation for MoveIt2 Perception Pipeline configuration,
        I tried to publish a small sqaure collsion object for each point that the node received.
        """
        self.point_cloud = msg
        if (self.point_cloud_cb_count % 20 == 0):
            point_array = ros2_numpy.numpify(msg)
            for i in range(0, len(point_array), 3):

                msg = CollisionObject()
                msg.header.frame_id = "world"
                msg.id = f"{i}"
                msg.operation = CollisionObject.ADD

                prim = SolidPrimitive()
                prim.type = SolidPrimitive.BOX
                prim.dimensions = [0.05, 0.05, 0.05]
                msg.primitives.append(prim)

                prim_pose = Pose()
                prim_pose.position.x = point_array[i]['x'].item()
                prim_pose.position.y = point_array[i]['y'].item()
                prim_pose.position.z = point_array[i]['z'].item()
                prim_pose.orientation.w = 1.0

                msg.primitive_poses.append(prim_pose)

                self.collision_pub.publish(msg)
        self.point_cloud_cb_count += 1

    def send_plan_path_request(self):
        req = MotionPlanRequest()

        req.group_name = "manipulator"
        req.num_planning_attempts = 2000
        req.allowed_planning_time = 20.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        req.max_cartesian_speed = 1.0

        req.start_state.joint_state = self.joint_state

        goal_constraints = Constraints()

        joint_name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        joint_position = self.target_joint_states[self.state_index]
        tolerance = 0.01
        goal_constraints.joint_constraints = []

        for i, name in enumerate(joint_name):
            goal_constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=joint_position[i],
                    tolerance_above=tolerance,
                    tolerance_below=tolerance,
                    weight=1.0
                )
            )
        req.goal_constraints = [goal_constraints]
        self.plan_path_cli.wait_for_service(timeout_sec=1.0)
        self.plan_path_future = self.plan_path_cli.call_async(
            GetMotionPlan.Request(motion_plan_request=req))
        self.node.get_logger().info("Motion Request Sent")

    def future_handling(self):

        # Path Plan future handling
        if self.plan_path_future is not None and self.plan_path_future.done():
            plan_path_result = self.plan_path_future.result().motion_plan_response
            if plan_path_result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.node.get_logger().info("Plan Path Done")

                traj_goal = ExecuteTrajectory.Goal(
                    trajectory=plan_path_result.trajectory)
                self.exec_traj_future = self.exec_traj_cli.send_goal_async(traj_goal)
            else:
                self.node.get_logger().error(
                    f"Path Plan error code: {plan_path_result.error_code}"
                )
            self.plan_path_future = None
        if self.exec_traj_future is not None and self.exec_traj_future.done():

            self.exec_traj_future = None
            self.state_index += 1
            self.state_index %= 3


def main():
    rclpy.init()
    node = Node(
        "pcl_obstacle_moveit_client",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True
    )
    _ = PCLObstacleMoveitClient(node)
    rclpy.spin(node)
    rclpt.shutdown()


if __name__ == "__main__":
    main()
