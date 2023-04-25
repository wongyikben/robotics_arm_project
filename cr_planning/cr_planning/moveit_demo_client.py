#!/usr/bin/python3
import yaml
from typing import List
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration

from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetMotionPlan

from moveit_msgs.action import ExecuteTrajectory


class MoveitDemoClient():
    def __init__(self, node: Node):
        self.node = node
        self.timer = self.node.create_timer(1.0, self.timer_callback)
        self.joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
        )

        self.collision_pub = self.node.create_publisher(
            CollisionObject,
            "/collision_object",
            10
        )
        self.is_srv_called = False

        self.fk_future = None
        self.fk_cli = self.node.create_client(
            GetPositionFK,
            "/compute_fk"
        )

        self.ik_future = None
        self.ik_cli = self.node.create_client(
            GetPositionIK,
            "/compute_ik"
        )

        self.validity_future = None
        self.validity_cli = self.node.create_client(
            GetStateValidity,
            "/check_state_validity"
        )

        self.planned_trajectory = None
        self.plan_path_future = None
        self.plan_path_cli = self.node.create_client(
            GetMotionPlan,
            "/plan_kinematic_path"
        )

        self.exec_traj_cli = ActionClient(
            self.node,
            ExecuteTrajectory,
            '/execute_trajectory'
        )

        file_path = self.node.get_parameter("obstacle_config_path").value
        with open(file_path, "r") as file:
            self.obstacle_config =  yaml.safe_load(file)

    def joint_state_cb(self, msg: JointState):
        self.joint_state = msg

    def timer_callback(self):
        self.publish_obstacle()
        if not self.is_srv_called:
            self.send_fk_request()
            self.send_ik_request()
            self.send_check_validity_request()
            self.send_plan_path_request()
            self.is_srv_called = True
        else:
            self.future_handling()

    def publish_obstacle(self):
        """
        Publish CollisionObject msd to collision_object topic to add object
        in planning scene
        """
        for key in self.obstacle_config.keys():
            config = self.obstacle_config[key]

            msg = CollisionObject()
            msg.header.frame_id = "world"
            msg.id = key
            msg.operation = CollisionObject.ADD

            for i in range(len(config["primitives"])):
                prim_config = config["primitives"][i]

                prim = SolidPrimitive()
                prim.type = getattr(SolidPrimitive, prim_config["type"])
                prim.dimensions = prim_config["dimensions"]
                msg.primitives.append(prim)

                prim_pose = Pose()
                self.set_pose(prim_pose, prim_config["pose"])
                msg.primitive_poses.append(prim_pose)

            self.collision_pub.publish(msg)

    def send_fk_request(self):
        """
        Send Forward kinematic computation request to MoveIt server

        Users can request Fk request for different like TF but most of the users
        are only interested in endeffector
        """
        req = GetPositionFK.Request()
        req.header.frame_id = self.node.get_parameter("fk_req.frame").value
        req.fk_link_names = self.node.get_parameter("fk_req.fk_link_names").value
        req.robot_state.joint_state.name =\
            self.node.get_parameter("fk_req.joint_state.name").value
        req.robot_state.joint_state.position =\
            self.node.get_parameter("fk_req.joint_state.position").value
        self.fk_cli.wait_for_service(timeout_sec=1.0)
        self.fk_future = self.fk_cli.call_async(req)

    def send_ik_request(self):
        """
        Send Inverse kinematic computation request to MoveIt server
        """
        req = PositionIKRequest()
        req.group_name = self.node.get_parameter("ik_req.group_name").value
        req.avoid_collisions =\
            self.node.get_parameter("ik_req.avoid_collisions").value
        req.robot_state.joint_state.name =\
            self.node.get_parameter("ik_req.joint_state.name").value

        # initial condition for IK solver
        req.robot_state.joint_state.position =\
            self.node.get_parameter("ik_req.joint_state.position").value
        req.pose_stamped.header.frame_id =\
            self.node.get_parameter("ik_req.frame").value
        req.pose_stamped.pose.position.x =\
            self.node.get_parameter("ik_req.pose.position.x").value
        req.pose_stamped.pose.position.y =\
            self.node.get_parameter("ik_req.pose.position.y").value
        req.pose_stamped.pose.position.z =\
            self.node.get_parameter("ik_req.pose.position.z").value

        req.pose_stamped.pose.orientation.x =\
            self.node.get_parameter("ik_req.pose.orientation.x").value
        req.pose_stamped.pose.orientation.y =\
            self.node.get_parameter("ik_req.pose.orientation.y").value
        req.pose_stamped.pose.orientation.z =\
            self.node.get_parameter("ik_req.pose.orientation.z").value
        req.pose_stamped.pose.orientation.w =\
            self.node.get_parameter("ik_req.pose.orientation.w").value

        req.timeout = Duration(
            sec=self.node.get_parameter("ik_req.timeout.sec").value,
            nanosec=self.node.get_parameter("ik_req.timeout.nanosec").value)

        self.ik_cli.wait_for_service(timeout_sec=1.0)
        self.ik_future = self.ik_cli.call_async(
            GetPositionIK.Request(ik_request=req))

    def send_check_validity_request(self):
        req = GetStateValidity.Request()
        req.group_name = self.node.get_parameter("validity_req.group_name").value
        req.robot_state.joint_state.name =\
            self.node.get_parameter("validity_req.joint_state.name").value
        req.robot_state.joint_state.position =\
            self.node.get_parameter("validity_req.joint_state.position").value
        self.validity_cli.wait_for_service(timeout_sec=1.0)
        self.validity_future = self.validity_cli.call_async(req)

    def send_plan_path_request(self):
        req = MotionPlanRequest()

        req.group_name =\
            self.node.get_parameter("plan_path_req.group_name").value
        req.num_planning_attempts =\
            self.node.get_parameter("plan_path_req.num_planning_attempts").value
        req.allowed_planning_time =\
            self.node.get_parameter("plan_path_req.allowed_planning_time").value
        req.max_velocity_scaling_factor =\
            self.node.get_parameter("plan_path_req.max_velocity_scaling_factor").value
        req.max_acceleration_scaling_factor =\
            self.node.get_parameter("plan_path_req.max_acceleration_scaling_factor").value
        req.max_cartesian_speed =\
            self.node.get_parameter("plan_path_req.max_cartesian_speed").value

        req.start_state.joint_state = self.joint_state

        goal_constraints = Constraints()

        joint_name = self.node.get_parameter("plan_path_req.joint_state.name").value
        joint_position = self.node.get_parameter("plan_path_req.joint_state.position").value
        tolerance = self.node.get_parameter("plan_path_req.tolerance").value
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

    def future_handling(self):
        # FK future handling
        if self.fk_future is not None and self.fk_future.done():
            fk_result = self.fk_future.result()
            if fk_result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.node.get_logger().info("FK solution:")
                for i, link in enumerate(fk_result.fk_link_names):
                    pose_stamped = fk_result.pose_stamped[i]
                    log_str = (
                        f"TF for {link}\n"
                        + f" frame: {pose_stamped.header.frame_id}\n"
                        + f" position: {pose_stamped.pose.position.x}"
                        + f" {pose_stamped.pose.position.y}"
                        + f" {pose_stamped.pose.position.z}\n"
                        + f" orientation: {pose_stamped.pose.orientation.x}"
                        + f" {pose_stamped.pose.orientation.y}"
                        + f" {pose_stamped.pose.orientation.z}"
                        + f" {pose_stamped.pose.orientation.w}\n"
                    )
                    self.node.get_logger().info(log_str)
            else:
                self.node.get_logger().error(
                    f"FK error code: {fk_result.error_code}"
                )
            self.fk_future = None

        # IK future handling
        if self.ik_future is not None and self.ik_future.done():
            ik_result = self.ik_future.result()
            if ik_result.error_code.val == MoveItErrorCodes.SUCCESS:
                joint_state = ik_result.solution.joint_state
                log_str = "IK solution:\n"
                for i, joint_name in enumerate(joint_state.name):
                    log_str += f" {joint_name}: {joint_state.position[i]}\n"
                self.node.get_logger().info(log_str)
            else:
                self.node.get_logger().error(
                    f"IK error code: {ik_result.error_code}"
                )
            self.ik_future = None

        # Validity future handling
        if self.validity_future is not None and self.validity_future.done():
            validity_result = self.validity_future.result()
            self.node.get_logger().info("Check Validity Solution:\n"
                + f" valid: {validity_result.valid}"
            )
            self.validity_future = None

        # Path Plan future handling
        if self.plan_path_future is not None and self.plan_path_future.done():
            plan_path_result = self.plan_path_future.result().motion_plan_response
            if plan_path_result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.node.get_logger().info("Plan Path Done")

                traj_goal = ExecuteTrajectory.Goal(
                    trajectory=plan_path_result.trajectory)
                self.exec_traj_cli.send_goal_async(traj_goal)
            else:
                self.node.get_logger().error(
                    f"Path Plan error code: {plan_path_result.error_code}"
                )
            self.plan_path_future = None


    def set_pose(self, msg: Pose, xyzrpy: List):
        """
        https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        """

        msg.position.x = xyzrpy[0]
        msg.position.y = xyzrpy[1]
        msg.position.z = xyzrpy[2]

        cy = math.cos(xyzrpy[3] * 0.5)
        sy = math.sin(xyzrpy[3] * 0.5)
        cp = math.cos(xyzrpy[4] * 0.5)
        sp = math.sin(xyzrpy[4]* 0.5)
        cr = math.cos(xyzrpy[5] * 0.5)
        sr = math.sin(xyzrpy[5] * 0.5)

        msg.orientation.x = cy * cp * cr + sy * sp * sr
        msg.orientation.y = cy * cp * sr - sy * sp * cr
        msg.orientation.z = sy * cp * sr + cy * sp * cr
        msg.orientation.w = sy * cp * cr - cy * sp * sr

        return


def main():
    rclpy.init()
    node = Node(
        "moveit_demo_client",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True
    )
    _ = MoveitDemoClient(node)
    rclpy.spin(node)
    rclpt.shutdown()


if __name__ == "__main__":
    main()
