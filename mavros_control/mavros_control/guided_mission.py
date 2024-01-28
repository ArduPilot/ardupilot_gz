import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import numpy as np


class GuidedMission(Node):
    """Class to create a guided mission."""

    def __init__(self) -> None:
        super().__init__('mavros_control')

        # Create QoS profiles

        # STATE_QOS used for state topics, like ~/state, ~/mission/waypoints etc.
        STATE_QOS = rclpy.qos.QoSProfile(
            depth=10, 
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # SENSOR_QOS used for most of sensor streams
        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

        # Create subscribers
        self.vehicle_state_subscriber = self.create_subscription(
            State, '/mavros/state', self.vehicle_state_callback, STATE_QOS)
        self.vehicle_pose_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', 
            self.vehicle_pose_callback, SENSOR_QOS)
        
        # Create publishers
        self.setpoint_position_publisher = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', SENSOR_QOS)

        # Create service clients
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting again...')
        self.get_logger().info('Set mode service available')

        self.arm_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting again...')
        self.get_logger().info('Arming service available')

        # Initialize variables
        self.vehicle_state = State()
        self.vehicle_pose = np.array([0., 0.])
        self.arm_request = CommandBool.Request()
        self.set_mode_request = SetMode.Request()
        self.setpoint_position = PoseStamped()
        self.waypoint_id = 0
        self.last_request = self.get_clock().now().to_msg().sec

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def at_waypoint(self, waypoint, tolerance=0.5):
        """Check if the vehicle is at the waypoint."""
        dist = np.linalg.norm(self.vehicle_pose - np.array(waypoint))
        if dist < tolerance:
            return True
        else:
            return False

    def vehicle_state_callback(self, vehicle_state):
        """Callback function for vehicle_state topic subscriber."""
        self.vehicle_state = vehicle_state

    def vehicle_pose_callback(self, vehicle_pose):
        """Callback function for vehicle_pose topic subscriber."""
        self.vehicle_pose = np.array([vehicle_pose.pose.position.x, 
                                      vehicle_pose.pose.position.y])

    def arm(self):
        """Send an arm command to the vehicle."""
        self.arm_request.value = True
        self.arm_client.call_async(self.arm_request)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.arm_request.value = False
        self.arm_client.call_async(self.arm_request)
        self.get_logger().info('Disarm command sent')

    def engage_guided_mode(self):
        """Send a command to the vehicle to enter GUIDED mode."""
        if self.set_mode_client.wait_for_service(timeout_sec=0.5):
            self.set_mode_request.custom_mode = "GUIDED"
            self.set_mode_client.call_async(self.set_mode_request)
            self.get_logger().info('Engage GUIDED mode command sent')
            
    def set_waypoint(self, x, y, z=0.0):
        """Set the next waypoint of the vehicle."""
        self.setpoint_position.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_position.pose.position.x = x
        self.setpoint_position.pose.position.y = y
        self.setpoint_position.pose.position.z = z
        self.setpoint_position_publisher.publish(self.setpoint_position)

    def timer_callback(self) -> None:
        """Callback function for the timer."""

        # Engage GUIDED mode
        if self.vehicle_state.mode != "GUIDED" and \
            (self.get_clock().now().to_msg().sec - self.last_request) > 5.0:
            self.engage_guided_mode()
            self.last_request = self.get_clock().now().to_msg().sec

        # Arm the vehicle
        if not self.vehicle_state.armed and \
            self.vehicle_state.mode == "GUIDED" and \
            (self.get_clock().now().to_msg().sec - self.last_request) > 5.0:
            self.arm()
            self.last_request = self.get_clock().now().to_msg().sec

        # Set the position setpoint
        if self.waypoint_id == 0:
            if self.at_waypoint([5., 0.]):
                self.waypoint_id = 1
                self.get_logger().info("Waypoint 1 reached")
            self.set_waypoint(5., 0.,)
        if self.waypoint_id == 1:
            if self.at_waypoint([5., 5.]):
                self.waypoint_id = 2
                self.get_logger().info("Waypoint 2 reached")
            self.set_waypoint(5., 5.)
        if self.waypoint_id == 2:
            if self.at_waypoint([0., 5.]):
                self.waypoint_id = 3
                self.get_logger().info("Waypoint 3 reached")
            self.set_waypoint(0., 5.)
        if self.waypoint_id == 3:
            if self.at_waypoint([0., 0.]):
                self.waypoint_id = 4
                self.get_logger().info("Waypoint 4 reached")
                self.disarm()
                exit()
            self.set_waypoint(0., 0.)


def main(args=None) -> None:
    print('Starting guided mission')
    rclpy.init(args=args)

    mission = GuidedMission()

    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
