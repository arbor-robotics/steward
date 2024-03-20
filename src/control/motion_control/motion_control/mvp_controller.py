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
from visualization_msgs.msg import Marker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("route_planner")

        self.setUpParameters()

        self.route = []

        persistent_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Note that the trajectory is not used in this minimal controller
        self.create_subscription(Path, "/planning/trajectory", self.trajectoryCb, 10)
        self.create_subscription(Route, "/planning/full_route", self.routeCb, 10)

        self.plan_marker_pub = self.create_publisher(
            Marker, "/vis/remaining_forest_plan", persistent_qos
        )

        self.route_marker_pub = self.create_publisher(
            Marker, "/vis/remaining_route", persistent_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TODO: Create proper QoS profile.
        self.twistPub = self.create_publisher(Twist, "/cmd_vel", 10)

        CONTROLLER_FREQ = 100  # Hz

        self.create_timer(1 / CONTROLLER_FREQ, self.updateController)

    def routeCb(self, msg: Route):
        if len(self.route) > 0:
            return  # route already found
        route = []
        for point in msg.points:
            point: Point
            numpy_pt = np.array([point.x, point.y])
            route.append(numpy_pt)

        # This is intentionally a Python list of Numpy arrays
        self.route = route

    def publishRemainingForestPlanMarker(self, points: list[Point]):
        self.get_logger().info(f"Publishing plan marker with {len(points)} points")
        marker_msg = Marker()
        marker_msg.header = self.getHeader()
        marker_msg.frame_locked = True
        marker_msg.type = Marker.CUBE_LIST
        marker_msg.points = points
        # https://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29
        marker_msg.scale.x = 1.0
        marker_msg.scale.y = 1.0
        marker_msg.scale.z = 3.0
        marker_msg.color.g = 1.0
        marker_msg.color.a = 1.0
        marker_msg.ns = "forest_plan"
        marker_msg.action = Marker.ADD
        marker_msg.id = 1
        self.plan_marker_pub.publish(marker_msg)

    def getPointsList(self, route: list[np.ndarray]):
        points = []
        for p in route:
            points.append(Point(x=p[0], y=p[1]))
        return points

    def publishRemainingRouteMarker(self, points: list[Point]):

        # Repeat points not on ends to satisfy LINE_LIST spec
        modified_points = []
        modified_points.append(points[0])
        for pt in points[1:-1]:
            modified_points.append(pt)
            modified_points.append(pt)  # Yes-- twice!
        modified_points.append(points[-1])

        self.get_logger().info(f"Publishing plan marker with {len(points)} points")
        marker_msg = Marker()
        marker_msg.header = self.getHeader()
        marker_msg.frame_locked = True
        marker_msg.type = Marker.LINE_LIST
        marker_msg.points = modified_points
        # https://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29
        marker_msg.scale.x = 0.5
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0
        marker_msg.ns = "route"
        marker_msg.action = Marker.ADD
        marker_msg.id = 1
        self.route_marker_pub.publish(marker_msg)

    def updateController(self):

        WAYPOINT_REACHED_THRESHOLD = 0.3  # m

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
        q = base_link_to_map_tf.transform.rotation

        # Use scipy to calculate ego yaw
        orientation_euler = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")

        # Add pi/2 to make "zero" point east (ENU) instead of north
        ego_yaw = orientation_euler[2] + np.pi / 2

        # Get ego distance from next route point
        next_route_pt = self.route[0]

        # Transform this point to the base_link frame
        # 1. First translate
        x_ = next_route_pt[0] - ego_x
        y_ = next_route_pt[1] - ego_y
        # 2. Then rotate given yaw
        x = x_ * np.cos(-ego_yaw) - y_ * np.sin(-ego_yaw)
        y = y_ * np.cos(-ego_yaw) + x_ * np.sin(-ego_yaw)
        yaw_error = np.arctan2(y, x)

        dx = next_route_pt[0] - ego_x
        dy = next_route_pt[1] - ego_y
        dist = np.sqrt(dx**2 + dy**2)
        if dist < WAYPOINT_REACHED_THRESHOLD:
            reached_point = self.route.pop(0)

            points = self.getPointsList([reached_point] + self.route)
            self.publishRemainingForestPlanMarker(points)
            self.publishRemainingRouteMarker(points)

        if yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        if yaw_error > math.pi:
            yaw_error -= 2 * math.pi

        command_msg = Twist()

        angular_twist_val = 8.0

        # 3 meters is close!
        if dist < 3:
            if yaw_error < math.radians(-3):
                self.get_logger().debug("TURN RIGHT")
                command_msg.linear.x = 0.0
            elif yaw_error > math.radians(3):
                self.get_logger().debug("TURN LEFT")
                command_msg.linear.x = 0.0
            else:
                command_msg.angular.z = 0.0
                command_msg.linear.x = 0.5
                self.get_logger().debug("DRIVE STRAIGHT")
        else:
            if yaw_error < math.radians(-10):
                self.get_logger().debug("TURN RIGHT")
                command_msg.linear.x = 0.0
            elif yaw_error > math.radians(10):
                self.get_logger().debug("TURN LEFT")
                command_msg.linear.x = 0.0
            else:
                command_msg.angular.z = 0.0
                command_msg.linear.x = 1.0
                self.get_logger().debug("DRIVE STRAIGHT")

        command_msg.angular.z = yaw_error
        self.twistPub.publish(command_msg)

    def trajectoryCb(self, msg: Path):
        """Not used in this node.

        Args:
            msg (Path): Trajectory message
        """
        self.get_logger().info("Got a trajectory!")

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = "map"  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def setUpParameters(self):
        """Not used."""
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
