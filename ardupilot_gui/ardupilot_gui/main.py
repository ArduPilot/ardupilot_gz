#!/usr/bin/env python3

import sys
import traceback
import signal

import rclpy
from PyQt5.QtWidgets import QApplication

from .ardupilot_node import ArduPilotNode
from .gui import ArduPilotGUI


# Setup signal handler for graceful shutdown
def signal_handler(sig, frame):
    QApplication.quit()


def main(args=None):
    node = None
    gui = None
    app = None

    signal.signal(signal.SIGINT, signal_handler)
    try:
        # Initialize ROS2
        rclpy.init(args=args)

        # Create the ROS2 node
        node = ArduPilotNode()

        # Create the Qt application
        app = QApplication(sys.argv)
        app.setQuitOnLastWindowClosed(True)
        gui = ArduPilotGUI(node)
        gui.show()

        # Run the Qt event loop
        exit_code = app.exec_()
        sys.exit(exit_code)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        if gui:
            gui.ros_thread.stop()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
