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

# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from steward_msgs.msg import FailedChecks, HealthCheck, SystemwideStatus


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
    ):
        self.code = code
        self.inspects = inspects
        self.period_sec = period_sec
        self.trigger_status = triggers
        self.message = message
        self.last_ping = -1
        self.triggered = True

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
                inspects=["bridge/steward", "bridge/warthog"],
                period_sec=0.2,
                triggers=SystemwideStatus.OUT_OF_SERVICE,
                message="Steward is unable to connect to the Warthog or the simulator and cannot operate.",
            )
        ]

        self.create_subscription(DiagnosticStatus, "/diagnostics", self.statusCb, 1)
        self.agg_pub = self.create_publisher(DiagnosticArray, "/diagnostics/agg", 1)
        self.system_status_pub = self.create_publisher(
            SystemwideStatus, "/health/system_wide_status", 1
        )

        self.failed_checks_pub = self.create_publisher(
            FailedChecks, "/health/failed_checks", 1
        )

        self.create_timer(0.02, self.aggregate)

    def statusCb(msg: DiagnosticStatus):
        pass

    def setUpParameters(self):
        pass

    def aggregate(self):
        systemwide_status = SystemwideStatus.HEALTHY
        failed_checks = []

        for check in self.checks:
            if (
                check.triggered or check.isStale
            ) and check.trigger_status > systemwide_status:
                systemwide_status = check.trigger_status

                check_msg = HealthCheck()
                check_msg.code = check.code
                check_msg.message = check.message
                check_msg.trigger_status = SystemwideStatus()
                check_msg.trigger_status.level = check.trigger_status
                failed_checks.append(check_msg)

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
