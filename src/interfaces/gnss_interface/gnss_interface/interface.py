import numpy as np
import math
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, Float32

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R

import pynmea2
from pynmea2.types import talker
import serial
import utm


class InterfaceNode(Node):
    def __init__(self):
        super().__init__("gnss_interface")

        self.setUpParameters()

        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

        self.odom_pub = self.create_publisher(Odometry, "/gnss/odom", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self.yaw_pub = self.create_publisher(Float32, "/gnss/yaw", 10)

        self.create_timer(0.01, self.checkGnssMessages)

        self.yaw_enu = None
        self.track_deg = None
        self.speed = None

    def publishOdometry(self, sentence: pynmea2.GGA):
        msg = Odometry()

        lat, lon, alt = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat, lon)

        ego_x, ego_y, _, __ = utm.from_latlon(sentence.latitude, sentence.longitude)

        ego_x = ego_x - origin_x
        ego_y = ego_y - origin_y

        # print(f"{ego_x}, {ego_y}")

        msg.pose.pose.position.x = ego_x
        msg.pose.pose.position.y = ego_y
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        self.odom_pub.publish(msg)

        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = "earth"

        fix_msg.latitude = sentence.latitude
        fix_msg.longitude = sentence.longitude

        self.fix_pub.publish(fix_msg)

        if self.yaw_enu is not None:
            self.yaw_pub.publish(Float32(data=self.yaw_enu))

    def trueTrackToEnuRads(self, track_deg: float):
        enu_yaw = track_deg

        enu_yaw -= 90

        enu_yaw = 360 - enu_yaw

        if enu_yaw < 0:
            enu_yaw += 360
        elif enu_yaw > 360:
            enu_yaw -= 360

        enu_yaw *= math.pi / 180.0
        return enu_yaw

    def parseVtg(self, msg: talker.VTG):
        # VTG: Actual track made good and speed over ground
        if msg.true_track is not None:
            self.track_deg = msg.true_track
            yaw_enu = self.trueTrackToEnuRads(msg.true_track)
            self.yaw_enu = yaw_enu

        if msg.spd_over_grnd_kmph is not None:
            self.speed = msg.spd_over_grnd_kmph * 0.277778

    def checkGnssMessages(self):

        if self.ser.closed:
            self.get_logger().warning(
                "Cannot read from GNSS. Serial connection is closed."
            )
            return

        line = self.ser.readline()  # read a '\n' terminated line
        # print(line)

        line = line.decode()
        # print(line)

        try:
            msg = pynmea2.parse(line)

            if isinstance(msg, talker.GGA):
                # print(f"({msg.latitude}, {msg.longitude})")
                self.publishOdometry(msg)
            elif isinstance(msg, talker.ZDA):
                # UTC day, month, and year, and local time zone offset
                return
            elif isinstance(msg, talker.GST):
                # Position error statistics
                return
            elif isinstance(msg, talker.GSV):
                return  # Number of SVs in view, PRN, elevation, azimuth, and SNR
            elif isinstance(msg, talker.GLL):
                # Position data: position fix, time of position fix, and status
                return
            elif isinstance(msg, talker.RMC):
                # Position, Velocity, and Time
                # print(msg.spd_over_grnd)
                return
            elif isinstance(msg, talker.VTG):
                self.parseVtg(msg)
            elif "GSA" in line:
                return  # GPS DOP and active satellites
            else:
                self.get_logger().warning(f"Received unknown NMEA type {type(msg)}")

            print(f"{self.track_deg} // {self.yaw_enu}")
        except pynmea2.nmea.ParseError as e:
            self.get_logger().warning(f"Could not parse {line}")

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = "map"  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )


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
