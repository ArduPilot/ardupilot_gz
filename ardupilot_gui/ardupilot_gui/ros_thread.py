"""
Subclassed thread for running ROS node
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from PyQt5.QtCore import QThread


class ROSThread(QThread):
    """Separate thread for ROS spinning to avoid Qt threading conflicts"""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.running = True

    def run(self):
        while self.running and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                self.node.get_logger().warn(f"ROS spin error: {e}")

    def stop(self):
        self.running = False
        self.wait()
