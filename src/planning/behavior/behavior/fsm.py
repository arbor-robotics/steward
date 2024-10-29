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
from steward_msgs.msg import FailedChecks, HealthCheck, SystemwideStatus, Mode
from std_msgs.msg import Empty

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

        self.current_mode_pub = self.create_publisher(Mode, "/planning/current_mode", 1)
        self.create_subscription(
            Mode, "/planning/requested_mode", self.requestedModeCb, 1
        )

        self.current_mode = Mode.STOPPED

        self.create_timer(0.1, self.publishCurrentMode)

    def publishCurrentMode(self):
        self.current_mode_pub.publish(Mode(level=self.current_mode))

    def requestedModeCb(self, msg: Mode):
        self.get_logger().info(f"Current mode is now {msg.level}")

        self.current_mode = msg.level

    def setUpParameters(self):
        pass


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
