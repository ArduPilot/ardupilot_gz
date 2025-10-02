#!/usr/bin/env python3
# filepath: /home/stephen/Documents/rosWorkspaces/ardupilot-ROS2/src/ardupilot_gui/ardupilot_gui/ardupilot_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, BatteryState
from ardupilot_msgs.msg import Status
from . import mavlinkutils
from ardupilot_msgs.srv import ArmMotors, ModeSwitch


class ArduPilotNode(Node):
    def __init__(self):
        super().__init__('ardupilot_gui_node')

        # Initialize data storage
        self.gps_data = None
        self.battery_data = None
        self.status_data = None
        self.last_message_time = None
        self.veh_type = None

        # Callbacks will be set by GUI
        self.gui_log_callback = None

        # Configure QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Topic names
        self.gps_topic = '/ap/navsat'
        self.battery_topic = '/ap/battery'
        self.status_topic = '/ap/status'

        # Add velocity data storage
        self.ap_velocity_data = None

        # Add velocity update callbacks
        self.ap_velocity_update_callback = None

        # Create subscribers and publishers
        self.create_robust_subscriptions()
        self.create_velocity_subscriptions()  # Add this line

        # Timer to check for available topics
        self.topic_check_timer = self.create_timer(5.0, self.check_available_topics)

        self.get_logger().info('ArduPilot GUI Node initialized')

    def set_gui_log_callback(self, callback):
        """Set callback to log messages to GUI"""
        self.gui_log_callback = callback

    def log_to_gui(self, message):
        """Log message to GUI if callback is set"""
        if self.gui_log_callback:
            self.gui_log_callback(message)

    def create_robust_subscriptions(self):
        """Create subscriptions with error handling and proper QoS"""
        subscriptions = [
            (BatteryState, self.battery_topic, self.battery_callback, self.sensor_qos),
            (NavSatFix, self.gps_topic, self.gps_callback, self.sensor_qos),
            (Status, self.status_topic, self.status_callback, self.sensor_qos)
        ]

        for msg_type, topic, callback, qos in subscriptions:
            try:
                self.create_subscription(msg_type, topic, callback, qos)
                self.get_logger().info(f'Subscribed to {topic}')
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')
                continue

    def check_available_topics(self):
        """Check what topics are actually available"""
        try:
            topic_names_and_types = self.get_topic_names_and_types()
            available_topics = [name for name, _ in topic_names_and_types]

            for topic in [self.gps_topic, self.battery_topic, self.status_topic]:
                if topic not in available_topics:
                    self.get_logger().debug(f'Topic {topic} not available')
        except Exception as e:
            self.get_logger().warn(f'Error checking topics: {e}')

    def gps_callback(self, msg):
        try:
            self.gps_data = {
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            }
            self.last_message_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().error(f'GPS callback error: {e}')

    def battery_callback(self, msg):
        try:
            self.battery_data = {
                'voltage': msg.voltage,
                'percentage': msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage,
                'current': msg.current
            }
            self.last_message_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f'Battery callback error: {e}')

    def status_callback(self, msg):
        failsafe_text = "".join(
            str(mavlinkutils.FAILSAFE_ID.get(fs_id, '')) for fs_id in msg.failsafe
            )
        try:
            self.status_data = {
                'armed': msg.armed,
                'mode': mavlinkutils.mode_string(msg.vehicle_type, msg.mode),
                'failsafe': failsafe_text,
                'vehicle_type': mavlinkutils.VEHICLE_TYPES.get(msg.vehicle_type, "Unknown"),
            }
            self.last_message_time = self.get_clock().now()
            self.veh_type = msg.vehicle_type

        except Exception as e:
            self.get_logger().error(f'Status callback error: {e}')

    def arm_vehicle(self):
        """ARM command - typically uses a service call"""
        self.get_logger().info('ARM command requested')
        try:
            client = self.create_client(ArmMotors, '/ap/arm_motors')

            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Arm service not available')
                return

            request = ArmMotors.Request()
            request.arm = True

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            response = future.result()

            if response is not None:
                self.get_logger().info('Arm request sent successfully')
                if response.result:
                    self.log_to_gui('Vehicle armed successfully')
                else:
                    self.log_to_gui('Failed to arm vehicle')
            else:
                self.get_logger().error('Arm request failed')
        except Exception as e:
            self.get_logger().error(f'Error sending arm command: {e}')

    def disarm_vehicle(self):
        """DISARM command - typically uses a service call"""
        try:
            client = self.create_client(ArmMotors, '/ap/arm_motors')

            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Arm service not available')
                return

            request = ArmMotors.Request()
            request.arm = False

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            response = future.result()

            if response is not None:
                self.get_logger().info('Disarm request sent successfully')
                if response.result:
                    self.log_to_gui('Vehicle disarmed successfully')
                else:
                    self.log_to_gui('Failed to disarm vehicle')
            else:
                self.get_logger().error('Disarm request failed')
        except Exception as e:
            self.get_logger().error(f'Error sending disarm command: {e}')

    def mode_switch(self, mode_string):
        """Switch flight mode using service call"""
        try:
            client = self.create_client(ModeSwitch, '/ap/mode_switch')

            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Mode switch service not available')
                return

            request = ModeSwitch.Request()
            request.mode = mavlinkutils.mode_int(self.veh_type, mode_string)

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            response = future.result()
            if response is not None:
                self.get_logger().info(f'Mode switch to {mode_string} sent successfully')
                if response.status:
                    self.log_to_gui(f'Mode switched to {mode_string}')
                else:
                    self.log_to_gui(f'Failed to switch to mode {mode_string}')
            else:
                self.get_logger().error('Mode switch request failed')
        except Exception as e:
            self.get_logger().error(f'Error sending mode switch command: {e}')

    def create_velocity_subscriptions(self):
        """Create subscriptions for velocity monitoring"""
        velocity_subscriptions = [
            (Twist, '/ap/cmd_vel', self.ap_cmd_vel_callback, self.sensor_qos),
        ]

        for msg_type, topic, callback, qos in velocity_subscriptions:
            try:
                self.create_subscription(msg_type, topic, callback, qos)
                self.get_logger().info(f'Subscribed to {topic} for velocity monitoring')
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')

    def ap_cmd_vel_callback(self, msg):
        """Monitor velocity commands sent to ArduPilot via /ap/cmd_vel"""
        try:
            # Store ArduPilot velocity data
            self.ap_velocity_data = {
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y,
                'linear_z': msg.linear.z,
                'angular_x': msg.angular.x,
                'angular_y': msg.angular.y,
                'angular_z': msg.angular.z,
                'timestamp': self.get_clock().now()
            }

            # Log velocity commands occasionally
            if not hasattr(self, '_last_ap_vel_log_time') or \
               (self.get_clock().now() - self._last_ap_vel_log_time).nanoseconds / 1e9 > 2.0:

                # Only log if there's significant velocity
                if any(abs(v) > 0.05 for v in [msg.linear.x, msg.linear.y, msg.linear.z, 
                                               msg.angular.x, msg.angular.y, msg.angular.z]):
                    self.log_to_gui(f'/ap/cmd_vel: L({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) '
                                    f'A({msg.angular.x:.2f},{msg.angular.y:.2f},{msg.angular.z:.2f})')
                    self._last_ap_vel_log_time = self.get_clock().now()

            # Update GUI if callback is set
            if self.ap_velocity_update_callback:
                self.ap_velocity_update_callback(msg)

        except Exception as e:
            self.get_logger().error(f'ArduPilot velocity callback error: {e}')

    def set_ap_velocity_update_callback(self, callback):
        """Set callback to update ArduPilot velocity display in GUI"""
        self.ap_velocity_update_callback = callback
