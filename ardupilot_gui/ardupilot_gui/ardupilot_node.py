"""
ROS2 node for monitoring Ardupilot topics and services
"""


from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix, BatteryState
from std_srvs.srv import Trigger
from ardupilot_msgs.msg import Status
from . import mavlinkutils
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff


class ArduPilotNode(Node):
    def __init__(self):
        super().__init__('ardupilot_gui_node')

        # Initialize data storage
        self.gps_data = None
        self.battery_data = None
        self.status_data = {}
        self.veh_type = None

        # Callbacks will be set by GUI
        self.gui_log_callback = None

        # Configure QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
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
        self.create_velocity_subscriptions()

        # Pre-arm check data
        self.pre_arm_status = False

        self.vehicle_active = False

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

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        subscriptions = [
            (BatteryState, self.battery_topic, self.battery_callback, sensor_qos),
            (NavSatFix, self.gps_topic, self.gps_callback, sensor_qos),
            (Status, self.status_topic, self.status_callback, sensor_qos)
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

        except Exception as e:
            self.get_logger().error(f'GPS callback error: {e}')

    def battery_callback(self, msg):
        try:
            self.battery_data = {
                'voltage': msg.voltage,
                'percentage': msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage,
                'current': msg.current
            }
        except Exception as e:
            self.get_logger().error(f'Battery callback error: {e}')

    def status_callback(self, msg):
        failsafe_text = "".join(
            str(mavlinkutils.FAILSAFE_ID.get(fs_id, '')) for fs_id in msg.failsafe
            )
        try:
            # Create new dict atomically
            new_status_data = {
                'armed': msg.armed,
                'mode': mavlinkutils.mode_string(msg.vehicle_type, msg.mode),
                'failsafe': failsafe_text,
                'vehicle_type': mavlinkutils.VEHICLE_TYPES.get(msg.vehicle_type, "UNKNOWN"),
            }

            # Atomic assignment (Python dict assignment is atomic)
            self.status_data = new_status_data
            self.veh_type = msg.vehicle_type
            self.vehicle_active = True

        except Exception as e:
            self.get_logger().error(f'Status callback error: {e}')

    def get_vehicle_type(self):
        """Get the vehicle type and return as a string"""
        if self.veh_type in mavlinkutils.VEHICLE_TYPES:
            return mavlinkutils.VEHICLE_TYPES[self.veh_type].upper()
        return "UNKNOWN"

    def arm_vehicle(self):
        """ARM command - uses a service call"""
        self.get_logger().info('ARM command requested')
        try:
            client = self.create_client(ArmMotors, '/ap/arm_motors')

            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Arm service not available')
                self.log_to_gui('Arm service not available')
                return

            request = ArmMotors.Request()
            request.arm = True

            # Use async call instead of blocking
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut: self._handle_arm_response(fut, True)
            )

            self.get_logger().info('Arm request sent')
            self.log_to_gui('Requesting vehicle arm...')

        except Exception as e:
            self.get_logger().error(f'Error sending arm command: {e}')
            self.log_to_gui(f'Error sending arm command: {e}')

    def disarm_vehicle(self):
        """DISARM command - uses a service call"""
        try:
            client = self.create_client(ArmMotors, '/ap/arm_motors')

            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Arm service not available')
                self.log_to_gui('Disarm service not available')
                return

            request = ArmMotors.Request()
            request.arm = False

            # Use async call instead of blocking
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut: self._handle_arm_response(fut, False)
            )

            self.get_logger().info('Disarm request sent')
            self.log_to_gui('Requesting vehicle disarm...')

        except Exception as e:
            self.get_logger().error(f'Error sending disarm command: {e}')
            self.log_to_gui(f'Error sending disarm command: {e}')

    def mode_switch(self, mode_string):
        """Switch flight mode using service call"""
        try:
            client = self.create_client(ModeSwitch, '/ap/mode_switch')

            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Mode switch service not available')
                self.log_to_gui('Mode switch service not available')
                return

            request = ModeSwitch.Request()
            request.mode = mavlinkutils.mode_int(self.veh_type, mode_string)

            # Use async call instead of blocking
            future = client.call_async(request)

            # Add callback to handle response
            future.add_done_callback(
                lambda fut: self._handle_mode_switch_response(fut, mode_string)
            )

            self.get_logger().info(f'Mode switch to {mode_string} request sent')
            self.log_to_gui(f'Requesting mode switch to {mode_string}...')

        except Exception as e:
            self.get_logger().error(f'Error sending mode switch command: {e}')
            self.log_to_gui(f'Error sending mode switch command: {e}')

    def _handle_mode_switch_response(self, future, mode_string):
        """Handle mode switch service response"""
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f'Mode switch to {mode_string} response received')
                if response.status:
                    self.log_to_gui(f'Mode switched to {mode_string}')
                else:
                    self.log_to_gui(f'Failed to switch to mode {mode_string}')
            else:
                self.get_logger().error('Mode switch request failed - no response')
                self.log_to_gui('Mode switch request failed - no response')
        except Exception as e:
            self.get_logger().error(f'Mode switch response error: {e}')
            self.log_to_gui(f'Mode switch response error: {e}')

    def takeoff_copter(self, alt):
        """Service call to takeoff a copter to alt m"""
        try:
            client = self.create_client(Takeoff, '/ap/experimental/takeoff')

            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Takeoff service not available')
                self.log_to_gui('Takeoff service not available')
                return

            request = Takeoff.Request()
            request.alt = float(alt)

            # Use async call instead of blocking
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut: self._handle_takeoff_response(fut, alt)
            )

            self.get_logger().info(f'Takeoff to {alt}m request sent')
            self.log_to_gui(f'Requesting takeoff to {alt}m...')

        except Exception as e:
            self.get_logger().error(f'Error sending takeoff command: {e}')
            self.log_to_gui(f'Error sending takeoff command: {e}')

    def prearm_check(self):
        """Request pre-arm check using service call"""
        try:
            client = self.create_client(Trigger, '/ap/prearm_check')

            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Pre-arm check service not available')
                self.log_to_gui('Pre-arm check service not available')
                return

            request = Trigger.Request()

            # Use async call to avoid blocking
            future = client.call_async(request)
            future.add_done_callback(self._handle_prearm_response)

            self.get_logger().debug('Pre-arm check request sent')

        except Exception as e:
            self.get_logger().error(f'Error requesting pre-arm check: {e}')
            self.log_to_gui(f'Error requesting pre-arm check: {e}')

    def _handle_arm_response(self, future, is_arm):
        """Handle arm/disarm service response"""
        action = "arm" if is_arm else "disarm"
        try:
            response = future.result()

            if response is not None:
                self.get_logger().info(f'{action.capitalize()} response received')
                if response.result:
                    self.log_to_gui(f'Vehicle {action}ed successfully')
                else:
                    self.log_to_gui(f'Failed to {action} vehicle')
            else:
                self.get_logger().error(f'{action.capitalize()} request failed - no response')
                self.log_to_gui(f'{action.capitalize()} request failed - no response')
        except Exception as e:
            self.get_logger().error(f'{action.capitalize()} response error: {e}')
            self.log_to_gui(f'{action.capitalize()} response error: {e}')

    def _handle_takeoff_response(self, future, alt):
        """Handle takeoff service response"""
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f'Takeoff to {alt}m response received')
                if response.status:
                    self.log_to_gui(f'Takeoff to {alt}m initiated')
                else:
                    self.log_to_gui(f'Failed to initiate takeoff to {alt}m')
            else:
                self.get_logger().error('Takeoff request failed - no response')
                self.log_to_gui('Takeoff request failed - no response')
        except Exception as e:
            self.get_logger().error(f'Takeoff response error: {e}')
            self.log_to_gui(f'Takeoff response error: {e}')

    def create_velocity_subscriptions(self):
        """Create subscriptions for velocity monitoring"""
        velocity_subscriptions = [
            (TwistStamped, '/ap/cmd_vel', self.ap_cmd_vel_callback, self.sensor_qos),
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
                'x': msg.twist.linear.x,
                'y': msg.twist.linear.y,
                'z': msg.twist.linear.z,
                'roll': msg.twist.angular.x,
                'pitch': msg.twist.angular.y,
                'yaw': msg.twist.angular.z,
                'timestamp': self.get_clock().now()
            }

            # Update GUI if callback is set
            if self.ap_velocity_update_callback:
                self.ap_velocity_update_callback()

        except Exception as e:
            self.get_logger().error(f'ArduPilot velocity callback error: {e}')

    def set_ap_velocity_update_callback(self, callback):
        """Set callback to update ArduPilot velocity display in GUI"""
        self.ap_velocity_update_callback = callback

    def _handle_prearm_response(self, future):
        """Handle pre-arm check service response"""
        try:
            response = future.result()
            if response is not None:
                self.pre_arm_status = response.success

            else:
                self.get_logger().error('Pre-arm check request failed - no response')
                self.log_to_gui('Pre-arm check request failed - no response')

        except Exception as e:
            self.get_logger().error(f'Pre-arm check response error: {e}')
            self.log_to_gui(f'Pre-arm check response error: {e}')
