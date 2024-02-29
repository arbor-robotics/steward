import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header
from steward_msgs.msg import Route


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("route_planner")

        self.setUpParameters()

        self.create_subscription(Path, "/planning/trajectory", self.pathCb, 10)

        # TODO: Create proper QoS profile.
        self.twistPub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_timer(0.5, self.publishSampleTwist)

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
