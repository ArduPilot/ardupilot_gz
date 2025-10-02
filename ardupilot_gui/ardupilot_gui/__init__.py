"""
ArduPilot ROS2 GUI Package

A simple GUI for monitoring and controlling ArduPilot vehicles through ROS2.
Provides real-time visualization of telemetry data
"""

__version__ = "1.0.0"
__author__ = "Stephen Dade"

from .ardupilot_node import ArduPilotNode
from .gui import ArduPilotGUI
from .ros_thread import ROSThread

__all__ = [
    'ArduPilotNode',
    'ArduPilotGUI',
    'ROSThread'
]
