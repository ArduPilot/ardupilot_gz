#!/usr/bin/env python3
"""Forwards arm_controller state from join trajectory action server to joint gz pos commands."""

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64

JOINT_TOPICS = {
    'shoulder_pan':  '/shoulder_pan/cmd_pos',
    'shoulder_lift': '/shoulder_lift/cmd_pos',
    'elbow':         '/elbow/cmd_pos',
    'wrist_flex':    '/wrist_flex/cmd_pos',
    'wrist_roll':    '/wrist_roll/cmd_pos',
    'grip_left':     '/grip/cmd_pos',
}


class MoveItToGzBridge(Node):
    def __init__(self):
        super().__init__('moveit_to_gz_bridge')
        self.pubs = {}
        for joint_name, topic in JOINT_TOPICS.items():
            self.pubs[joint_name] = self.create_publisher(Float64, topic, 10)
        self.create_subscription(
            JointTrajectoryControllerState, '/arm_controller/state',
            self.on_state, 10)

    def on_state(self, msg):
        for i, name in enumerate(msg.joint_names):
            if name in self.pubs and i < len(msg.actual.positions):
                cmd = Float64()
                cmd.data = msg.actual.positions[i]
                self.pubs[name].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItToGzBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
