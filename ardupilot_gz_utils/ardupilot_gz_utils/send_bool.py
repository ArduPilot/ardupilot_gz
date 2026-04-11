import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Bool


class SendBool(Node):

    def __init__(self):
        super().__init__("send_bool")

        self.declare_parameter("topic", "trigger")
        self.topic = self.get_parameter("topic").value

        latching_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        pub = self.create_publisher(Bool, self.topic, qos_profile=latching_qos)
        msg = Bool()
        msg.data = True
        pub.publish(msg)
        self.get_logger().info(f"Publishing trigger on '{self.topic}'")


def main(args=None):
    rclpy.init(args=args)
    node = SendBool()

    try:
        rclpy.spin_once(node, timeout_sec=120)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected")
    finally:
        node.get_logger().info("Shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
