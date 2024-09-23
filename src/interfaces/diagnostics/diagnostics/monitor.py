import numpy as np
import math
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Messages
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from std_msgs.msg import Header

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R

import pynmea2
import serial
import utm


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("health_monitor")

        self.setUpParameters()

        self.wh_diagnostics_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics/warthog", self.whDiagnosticsCb, 10
        )

        self.agg_pub = self.create_publisher(DiagnosticArray, "/diagnostics/agg", 10)

        self.staleness_tolerances = {"warthog": 10.0}

        self.create_timer(0.1, self.aggregate)

        self.statuses = {}

    def aggregate(self):
        msg = DiagnosticArray()

        for key, value in self.statuses.items():
            msg.status.append(value)

        msg.header = self.getHeader()
        self.agg_pub.publish(msg)

    def whDiagnosticsCb(self, msg: DiagnosticArray):
        # self.get_logger().info("Received wh diag!")

        for status in msg.status:
            status: DiagnosticStatus
            # self.get_logger().info(str(status))

            self.statuses[status.name] = status

    def getHeader(self) -> Header:
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def setUpParameters(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = HealthMonitor()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.ser.close()
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
