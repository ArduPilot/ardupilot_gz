import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Bool


class WaitForBool(Node):
    def __init__(self):
        super().__init__("wait_for_bool")

        self.declare_parameter("topic", "trigger")
        topic = self.get_parameter("topic").value

        latching_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(Bool, topic, self.cb, latching_qos)
        self.get_logger().info(f"Waiting for trigger on '{topic}'...")

    def cb(self, msg):
        if msg.data:
            self.get_logger().info("Received trigger, exiting...")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaitForBool()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
