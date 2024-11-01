import numpy as np
import math
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Messages
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header, Float32

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R

import utm


def trueTrackToEnuRads(track_deg: float):
    enu_yaw = track_deg

    enu_yaw -= 90

    enu_yaw = 360 - enu_yaw

    if enu_yaw < 0:
        enu_yaw += 360
    elif enu_yaw > 360:
        enu_yaw -= 360

    enu_yaw *= math.pi / 180.0
    return enu_yaw


fig, (ax1) = plt.subplots(1, 1)


class EkfHelper(Node):
    def __init__(self):
        super().__init__("ekf_helper")

        self.setUpParameters()

        self.create_subscription(GPSFix, "/gnss/gpsfix", self.gpsFixCb, 1)
        self.create_subscription(Odometry, "/odometry/filtered", self.filteredOdomCb, 1)

        self.odom_pub = self.create_publisher(Odometry, "/ekf_in/odom", 1)

        self.gnss_poses = []
        self.ekf_poses = []

    def filteredOdomCb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = np.arccos(msg.pose.pose.orientation.z) * 2

        self.ekf_poses.append([x, y, yaw])

    def gpsFixCb(self, msg: GPSFix):
        odom_msg = Odometry()

        lat0, lon0, alt0 = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)

        ego_x, ego_y, _, __ = utm.from_latlon(msg.latitude, msg.longitude)

        x = ego_x - origin_x
        y = ego_y - origin_y
        yaw = trueTrackToEnuRads(msg.track)

        odom_msg.header.frame_id = "map"
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.child_frame_id = msg.header.frame_id

        # Position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = msg.altitude - alt0

        # Orientation
        q = R.from_euler("xyz", [0.0, 0.0, yaw]).as_quat()  # Scalar last
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Pose covariance
        pose_cov = np.zeros((6, 6))
        pose_cov[:3, :3] = np.asarray(msg.position_covariance).reshape((3, 3))
        pose_cov[5, 5] = msg.err_track * np.pi / 180.0
        print(f"Yaw cov is {msg.err_track * np.pi / 180.0}")
        odom_msg.pose.covariance = pose_cov.flatten().tolist()

        self.odom_pub.publish(odom_msg)

        self.gnss_poses.append([x, y, yaw])
        self.plot()

    def plot(self):
        gnss_poses = np.asarray(self.gnss_poses)
        ax1.scatter(gnss_poses[:, 0], gnss_poses[:, 1], c="red")
        mean_x, mean_y = np.mean(gnss_poses[:, :2], axis=0)

        ax1.set_xlim(mean_x - 50, mean_x + 50)
        ax1.set_ylim(mean_y - 50, mean_y + 50)

        if len(self.ekf_poses) > 0:
            ekf_poses = np.asarray(self.ekf_poses)
            ax1.scatter(ekf_poses[:, 0], ekf_poses[:, 1], c="blue")

            plt.savefig("ekf.png")

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )


def main(args=None):
    rclpy.init(args=args)

    node = EkfHelper()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plt.close(fig)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
