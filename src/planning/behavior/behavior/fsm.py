import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import json
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
import cv2
from enum import IntEnum
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import utm


# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from steward_msgs.msg import FailedChecks, HealthCheck, SystemwideStatus, Mode
from std_msgs.msg import Empty, String

# class SystemwideStatus:
#     HEALTHY = 0
#     WARN = 1
#     TELEOP_ONLY = 2
#     OOS = 3


class FsmNode(Node):
    def __init__(self):
        super().__init__("behavior_fsm")

        self.setUpParameters()

        self.get_logger().info("Hello, world.")

        self.create_subscription(
            Mode, "/planning/requested_mode", self.requestedModeCb, 1
        )
        self.create_subscription(String, "/planning/plan_json", self.planCb, 1)

        self.current_mode_pub = self.create_publisher(Mode, "/planning/current_mode", 1)
        self.seedling_reached_pub = self.create_publisher(
            Empty, "/behavior/seedling_reached", 1
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_mode = Mode.STOPPED
        self.seedling_points = []

        self.create_timer(0.1, self.publishCurrentMode)
        self.create_timer(0.1, self.checkSeedlingDistance)
        self.seedling_reached_distance = 1.0  # meters

    def checkSeedlingDistance(self):
        try:
            bl_to_map_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            ego_x = bl_to_map_tf.transform.translation.x
            ego_y = bl_to_map_tf.transform.translation.y
            self.ego_pos = [ego_x, ego_y]

        except TransformException as ex:
            return

        updated_points = []

        for point in self.seedling_points:
            dist = pdist([point, self.ego_pos])

            if dist < self.seedling_reached_distance:
                print("SEEDLING REACHED")
                self.seedling_reached_pub.publish(Empty())
            else:
                updated_points.append(point)

        self.seedling_points = updated_points

    def latLonToMap(self, lat: float, lon: float):
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)

        x, y, _, __ = utm.from_latlon(lat, lon)

        x = x - origin_x
        y = y - origin_y

        return (x, y)

    def planCb(self, msg: String):

        print("Received plan!")

        try:
            plan_obj = json.loads(msg.data)
        except json.decoder.JSONDecodeError as e:
            self.get_logger().error(f"Could not process plan message: {e}")
            return

        try:
            seedlings: list[object] = plan_obj["seedlings"]

            self.seedling_points = []
            print("Updated seedling_points!")

            for seedling in seedlings:
                seedling_x, seedling_y = self.latLonToMap(
                    seedling["lat"], seedling["lon"]
                )

                self.seedling_points.append([seedling_x, seedling_y])

            print(self.seedling_points)
        except KeyError as e:
            self.get_logger().error(f"{e}")

    def publishCurrentMode(self):
        self.current_mode_pub.publish(Mode(level=self.current_mode))

    def requestedModeCb(self, msg: Mode):
        self.get_logger().info(f"Current mode is now {msg.level}")

        self.current_mode = msg.level

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )


def main(args=None):
    rclpy.init(args=args)

    node = FsmNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
