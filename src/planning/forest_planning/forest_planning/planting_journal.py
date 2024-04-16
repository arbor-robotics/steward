import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from time import time
from tqdm import tqdm, trange
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ROS interfaces
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from steward_msgs.action import CreateForestPlan
from steward_msgs.msg import ForestPlan
from std_msgs.msg import Empty, Float32
from visualization_msgs.msg import Marker

"""
Subscribe to /vis/forest_plan and get the total number of seedlings.
Every time /planning/plant_seedling is received, add our current odom to a list.
Periodically publish this point list as a Marker.
"""


class PlantingJournal(Node):
    def __init__(self):
        super().__init__("forest_planner")

        self.create_subscription(
            Empty, "/planning/plant_seedling", self.recordPlantedSeedling, 10
        )
        self.create_subscription(Marker, "/vis/forest_plan", self.forestPlanCb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bounds_msg = None

        self.num_seedlings_in_plan = 9999
        self.planted_points = []
        self.stamps = []
        self.progress = 0  # 0 to 1

        # https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # TODO: Create proper QoS profile.
        self.journal_pub = self.create_publisher(
            Marker, "/planning/planted_points", map_qos
        )

        self.progress_pub = self.create_publisher(Float32, "/planning/progress", 10)
        self.eta_pub = self.create_publisher(Float32, "/planning/eta", 10)
        self.num_planted_seedlings_pub = self.create_publisher(
            Float32, "/planning/num_planted_seedlings", 10
        )

        MARKER_PUBLISH_RATE = 0.5  # Hz. TODO: Parameterize. WSH.
        self.create_timer(1 / MARKER_PUBLISH_RATE, self.publishJournalMarker)

    @property
    def num_remaining_seedlings(self):
        return self.num_seedlings_in_plan - len(self.planted_points)

    def forestPlanCb(self, msg: Marker):
        self.num_seedlings_in_plan = len(msg.points)

    def calculateSeedlingsPerMinute(self) -> float:

        if len(self.stamps) < 2:
            return 0

        return 60 / np.mean(np.diff(self.stamps))

    def estimateTotalPlantingTimeSecs(self) -> float:
        return self.calculateSeedlingsPerMinute() * self.num_seedlings_in_plan

    def estimateRemaingPlantingTimeSecs(self) -> float:
        return float(self.calculateSeedlingsPerMinute() * self.num_remaining_seedlings)

    def recordPlantedSeedling(self, msg: Empty):
        try:
            t = self.tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f"Could not transform from base_link to map: {ex}")
            return

        seedling_pt = [t.transform.translation.x, t.transform.translation.y]
        self.planted_points.append(seedling_pt)
        self.stamps.append(time())
        if self.num_seedlings_in_plan is not None:
            self.progress = len(self.planted_points) / self.num_seedlings_in_plan
            progress_msg = Float32()
            progress_msg.data = self.progress
            self.progress_pub.publish(progress_msg)

            eta_msg = Float32(data=self.estimateRemaingPlantingTimeSecs())
            self.eta_pub.publish(eta_msg)

            num_planted_seedlings_msg = Float32(data=float(len(self.planted_points)))
            self.num_planted_seedlings_pub.publish(num_planted_seedlings_msg)

        estimate_minutes = self.estimateTotalPlantingTimeSecs() / 60

        self.get_logger().info(
            f"Recorded planted seedling in the journal. Journal now has {len(self.planted_points)} entries. Progress is {self.progress}. SPM is {int(self.calculateSeedlingsPerMinute())}. Total estimate is {estimate_minutes} min. Remaining is {int(self.estimateRemaingPlantingTimeSecs() / 60)} min"
        )

    def publishJournalMarker(self):
        # Repeat points not on ends to satisfy LINE_LIST spec
        point_msgs = []
        for pt in self.planted_points:
            point_msg = Point(x=pt[0], y=pt[1])
            point_msgs.append(point_msg)

        # self.get_logger().info(
        #     f"Publishing journal marker with {len(point_msgs)} points"
        # )
        marker_msg = Marker()
        marker_msg.header = self.getHeader()
        marker_msg.frame_locked = True
        marker_msg.type = Marker.POINTS
        marker_msg.points = point_msgs
        # https://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29
        marker_msg.scale.x = 0.5  # point width
        marker_msg.scale.y = 0.5  # point height
        marker_msg.color.r = 1.0
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0
        marker_msg.ns = "planted_points"
        marker_msg.action = Marker.ADD
        marker_msg.id = 1
        self.journal_pub.publish(marker_msg)

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = "map"  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg


def main(args=None):
    rclpy.init(args=args)

    node = PlantingJournal()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
