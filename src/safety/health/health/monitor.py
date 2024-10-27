import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
import cv2
from enum import IntEnum
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener


# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from steward_msgs.msg import FailedChecks, HealthCheck, SystemwideStatus
from std_msgs.msg import Empty

# class SystemwideStatus:
#     HEALTHY = 0
#     WARN = 1
#     TELEOP_ONLY = 2
#     OOS = 3


class Check:
    def __init__(
        self,
        code: str,
        inspects: list[str],
        period_sec: float,
        triggers: SystemwideStatus,
        message: str,
        max_status: bytes = DiagnosticStatus.WARN,
    ):
        self.code = code
        self.inspects = inspects
        self.period_sec = period_sec
        self.trigger_status = triggers
        self.message = message
        self.last_ping = -1
        self.triggered = True

        # Status levels greater than this will cause the check to fail
        # See the values at https://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
        self.max_status = max_status

    def isStale(self):
        return time() - self.last_ping > self.period_sec

    def ping(self, trigger: bool = False):
        if trigger:
            self.triggered = True

        self.last_ping = time()


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("health_monitor")

        self.setUpParameters()

        self.get_logger().info("Hello, world.")

        self.checks = [
            Check(
                "BRIDGE_FAILURE",
                inspects=["sim_bridge", "bridge/warthog"],
                period_sec=0.2,
                triggers=SystemwideStatus.OUT_OF_SERVICE,
                message="Steward is unable to connect to the Warthog or the simulator and cannot operate.",
            )
        ]

        self.statuses = {}
        self.heartbeat_times = {}

        self.create_subscription(DiagnosticStatus, "/diagnostics", self.statusCb, 1)
        self.agg_pub = self.create_publisher(DiagnosticArray, "/diagnostics/agg", 1)
        self.system_status_pub = self.create_publisher(
            SystemwideStatus, "/health/system_wide_status", 1
        )

        self.failed_checks_pub = self.create_publisher(
            FailedChecks, "/health/failed_checks", 1
        )

        self.heartbeat_pub = self.create_publisher(Empty, "/hb/global", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.02, self.aggregate)

        self.create_timer(0.5, self.publishGlobalHeartbeat)
        self.create_timer(0.5, self.checkTransforms)

        self.STATE_ESTIMATION_AVAILABLE = False
        self.state_estimation_check_msg = HealthCheck()

    def checkTransforms(self):
        check_msg = HealthCheck()
        check_msg.code = "LOCALIZATION_UNAVAILABLE"
        check_msg.message = "Steward doesn't know where it is. Could not find base_link -> map transform. Is the localization node OK? Is the GNSS interface OK?"
        check_msg.trigger_status = SystemwideStatus()
        check_msg.trigger_status.level = SystemwideStatus.TELEOP_ONLY
        try:
            bl_to_map_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            self.STATE_ESTIMATION_AVAILABLE = True

        except TransformException as ex:
            self.STATE_ESTIMATION_AVAILABLE = False

        self.state_estimation_check_msg = check_msg

    def publishGlobalHeartbeat(self):
        self.heartbeat_pub.publish(Empty())

    def statusCb(self, msg: DiagnosticStatus):
        self.statuses[msg.name] = msg
        self.heartbeat_times[msg.name] = time()

    def setUpParameters(self):
        pass

    def aggregate(self):
        systemwide_status = SystemwideStatus.HEALTHY
        failed_checks = []

        for check in self.checks:

            # Current behavior: At least one node in the "inspects"
            # list needs to be both OK and not stale
            is_stale = True
            triggered = True
            for node_name in check.inspects:

                # Have we received a status at all?
                try:
                    status: DiagnosticStatus = self.statuses[node_name]
                except KeyError:
                    continue  # If not, skip

                # Is it current (not stale)?
                if time() - self.heartbeat_times[node_name] < check.period_sec:
                    is_stale = False

                    if status.level <= check.max_status:
                        triggered = False

            if (triggered or is_stale) and check.trigger_status > systemwide_status:
                systemwide_status = check.trigger_status

                check_msg = HealthCheck()
                check_msg.code = check.code
                check_msg.message = check.message
                check_msg.trigger_status = SystemwideStatus()
                check_msg.trigger_status.level = check.trigger_status
                failed_checks.append(check_msg)

        if not self.STATE_ESTIMATION_AVAILABLE:
            failed_checks.append(self.state_estimation_check_msg)
            systemwide_status = self.state_estimation_check_msg.trigger_status.level

        msg = SystemwideStatus()
        msg.level = systemwide_status
        self.system_status_pub.publish(msg)

        failed_checks_msg = FailedChecks()
        failed_checks_msg.checks = failed_checks
        self.failed_checks_pub.publish(failed_checks_msg)


def main(args=None):
    rclpy.init(args=args)

    node = HealthMonitor()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
