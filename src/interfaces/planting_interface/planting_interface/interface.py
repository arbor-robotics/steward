import numpy as np
import math
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from time import time

# Messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header, Float32, Empty

import serial


class InterfaceNode(Node):
    def __init__(self):
        super().__init__("planting_interface")

        self.setUpParameters()  # Baud rate, port

        self.create_subscription(Empty, "/behavior/do_plant", self.doPlantCb, 1)
        self.create_subscription(
            Empty, "/behavior/do_pause_plant", self.doPausePlantCb, 1
        )

        self.on_plant_complete_pub = self.create_publisher(
            Empty, "/behavior/on_plant_complete", 1
        )

        self.create_timer(0.1, self.checkPlantingComplete)

        self.planting_in_progress = False
        self.planting_start_time = time()
        self.PLANTING_DURATION = 30  # sec. TODO: Check on this.

    def checkPlantingComplete(self):
        if (
            self.planting_in_progress
            and time() > self.planting_start_time + self.PLANTING_DURATION
        ):
            self.on_plant_complete_pub.publish(Empty())

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_STRING
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_INTEGER
        self.declare_parameter("serial_baud", 9600)

    def doPlantCb(self, msg: Empty):
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("serial_baud").value
        with serial.Serial(port, baud, timeout=1) as ser:
            ser.writelines(["Plant".encode("utf_8")])

            print(ser.readlines(40))

        self.get_logger().info("Sent Plant")
        self.planting_start_time = time()
        self.planting_in_progress = True

    def doPausePlantCb(self, msg: Empty):
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("serial_baud").value
        with serial.Serial(port, baud, timeout=1) as ser:
            ser.writelines(["Stop".encode("utf_8")])

        self.get_logger().info("Sent Stop")

        self.planting_in_progress = False


def main(args=None):
    rclpy.init(args=args)

    node = InterfaceNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.ser.close()
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
