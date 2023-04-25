#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class JointStateAdaptor():
    def __init__(self, node: Node):
        self.node = node
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
        )

        self.joint_pub_dict = {}
        for name in [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]:
            self.joint_pub_dict[name] =\
                self.node.create_publisher(
                    Float64,
                    f"/model/ur10/joint/{name}/cmd_pos",
                    10)


    def joint_state_cb(self, msg: JointState):
        for i, joint_name in enumerate(msg.name):
            self.joint_pub_dict[joint_name].publish(
                Float64(data=msg.position[i]))


def main():
    rclpy.init()
    node = Node(
        "joint_state_adaptor",
    )
    _ = JointStateAdaptor(node)
    rclpy.spin(node)
    rclpt.shutdown()


if __name__ == "__main__":
    main()
