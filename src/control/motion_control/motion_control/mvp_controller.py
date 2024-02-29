import numpy as np
import math
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Messages
from geometry_msgs.msg import Twist, TransformStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
from steward_msgs.msg import Route

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("route_planner")

        self.setUpParameters()

        self.route = []

        self.create_subscription(Path, "/planning/trajectory", self.pathCb, 10)
        self.create_subscription(Route, "/planning/full_route", self.routeCb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TODO: Create proper QoS profile.
        self.twistPub = self.create_publisher(Twist, "/cmd_vel", 10)

        CONTROLLER_FREQ = 100  # Hz

        self.create_timer(1 / CONTROLLER_FREQ, self.updateController)

    def routeCb(self, msg: Route):
        route = []
        for point in msg.points:
            point: Point
            numpy_pt = np.array([point.x, point.y])
            route.append(numpy_pt)

        # This is intentionally a Python list of Numpy arrays
        self.route = route

    def updateController(self):

        WAYPOINT_REACHED_THRESHOLD = 1.0  # m

        if len(self.route) < 1:
            return

        try:
            base_link_to_map_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not find base_link->map transform. Skipping controller update."
            )
            return

        ego_x = base_link_to_map_tf.transform.translation.x
        ego_y = base_link_to_map_tf.transform.translation.y
        qz = base_link_to_map_tf.transform.rotation.z

        # This is a very rough approximation of yaw
        # TODO: Use scipy rotations
        ego_yaw = np.arcsin(qz) * 2 + math.pi / 2

        # Get ego distance from next route point
        next_route_pt = self.route[0]
        dx = next_route_pt[0] - ego_x
        dy = next_route_pt[1] - ego_y
        dist = np.sqrt(dx**2 + dy**2)
        if dist < WAYPOINT_REACHED_THRESHOLD:
            self.route.pop(0)

        target_heading = math.atan2(dy, dx)
        yaw_error = target_heading - ego_yaw
        self.get_logger().info(
            f"We're {dist} m away, heading {math.degrees(target_heading)}"
        )
        self.get_logger().info(
            f"Heading is {math.degrees(ego_yaw)}, err {math.degrees(yaw_error)}"
        )

    def pathCb(self, msg: Path):
        self.get_logger().info("Got a path!")

    def publishSampleTwist(self):
        twist_msg = Twist()
        # twist_msg.linear.x = 2.0  # m/s
        twist_msg.angular.z = 1.0  # rad/s

        self.twistPub.publish(twist_msg)

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = "map"  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def setUpParameters(self):
        # bounds_map_path_param_desc = ParameterDescriptor()
        # bounds_map_path_param_desc.description = "File path to planting bounds image"
        # bounds_map_path_param_desc.type = ParameterType.PARAMETER_STRING
        # self.declare_parameter("bounds_map_path", True, bounds_map_path_param_desc)
        pass


def main(args=None):
    rclpy.init(args=args)

    node = RoutePlanner()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
