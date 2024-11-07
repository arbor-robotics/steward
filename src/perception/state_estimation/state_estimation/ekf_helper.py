from numpy.random import uniform, randn
import numpy as np
import scipy
from matplotlib import pyplot as plt
from filterpy.monte_carlo import systematic_resample
from numpy.linalg import norm
from numpy.random import randn
import scipy.stats
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import utm

# ROS message definitions
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

"""
ADAPTED FROM ROGER LABBE: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
"""


class EkfHelper(Node):
    def __init__(self):
        super().__init__("ekf_helper")

        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.setUpParameters()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(GPSFix, "/gnss/gpsfix", self.gpsfixCb, 1)
        self.create_subscription(Odometry, "/odometry/filtered", self.filteredOdomCb, 1)

        self.odom_pub = self.create_publisher(Odometry, "/gnss/odom", 1)
        self.filtered_fix_pub = self.create_publisher(NavSatFix, "/filtered_fix", 1)

        self.zone_number, self.zone_letter = None, None

        # self.create_timer(0.1, self.publishOdom)

    def filteredOdomCb(self, msg: Odometry):
        pos = msg.pose.pose.position
        lat, lon = self.odomToLatLon(pos.x, pos.y)

        fix_msg = NavSatFix()
        fix_msg.latitude = lat
        fix_msg.longitude = lon

        self.filtered_fix_pub.publish(fix_msg)

    def odomToLatLon(self, ego_x: float, ego_y: float):
        lat0, lon0, alt0 = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, zone_number, zone_letter = utm.from_latlon(lat0, lon0)
        return utm.to_latlon(
            ego_x + origin_x, ego_y + origin_y, zone_number, zone_letter
        )

    def gpsfixCb(self, msg: GPSFix):
        # Form an odom message

        lat, lon, alt = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, self.zone_number, self.zone_letter = utm.from_latlon(
            lat, lon
        )

        ego_x, ego_y, _, __ = utm.from_latlon(msg.latitude, msg.longitude)

        ego_x = ego_x - origin_x
        ego_y = ego_y - origin_y

        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = ego_x
        odom_msg.pose.pose.position.y = ego_y
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"

        cov = np.zeros((6, 6))
        cov[:3, :3] = np.asarray(msg.position_covariance).reshape((3, 3)) * 0.1

        odom_msg.pose.covariance = cov.flatten().tolist()

        self.odom_pub.publish(odom_msg)

    # def publishOdom(self):
    #     try:
    #         bl_to_map_tf = self.tf_buffer.lookup_transform(
    #             "map", "base_link", rclpy.time.Time()
    #         )
    #         ego_x = bl_to_map_tf.transform.translation.x
    #         ego_y = bl_to_map_tf.transform.translation.y
    #         self.ego_pos = (ego_x, ego_y)

    #         q = bl_to_map_tf.transform.rotation

    #         msg = Odometry()
    #         msg.child_frame_id = "base_link"
    #         msg.header.frame_id = "map"
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.pose.pose =
    #     except TransformException as ex:
    #         self.get_logger().warning(f"Could not get ego position: {ex}")
    #         return np.ones((100, 100)) * 100

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )


def main(args=None):
    rclpy.init(args=args)

    broadcaster = EkfHelper()

    rclpy.spin(broadcaster)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
