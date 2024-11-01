"""Subscribe to:

- /gnss/gpsfix
- /gnss/twist
- /gnss/imu
- /odom/wheel
"""

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
from tf2_ros import TransformBroadcaster
import utm
import pandas as pd
from time import time

# ROS message definitions
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
    TwistWithCovarianceStamped,
)
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def trueTrackToEnuRads(track_deg: float):
    enu_yaw = track_deg

    enu_yaw -= 90

    enu_yaw = 360 - enu_yaw

    if enu_yaw < 0:
        enu_yaw += 360
    elif enu_yaw > 360:
        enu_yaw -= 360

    enu_yaw *= np.pi / 180.0
    return enu_yaw


class ParticleFilterLocalizer(Node):
    def __init__(self):
        super().__init__("particle_filter_localizer")

        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.setUpParameters()

        # TODO: Parameterize topic name
        self.create_subscription(
            GPSFix, "/gnss/gpsfix", self.gpsFixCb, sensor_qos_profile
        )

        self.create_subscription(Imu, "/gnss/imu", self.imuCb, sensor_qos_profile)
        self.create_subscription(
            Odometry, "/odom/wheel", self.wheelCb, sensor_qos_profile
        )
        self.create_subscription(
            TwistWithCovarianceStamped, "/gnss/twist", self.twistCb, sensor_qos_profile
        )

        self.start_time = time()
        self.create_timer(1.0, self.saveData)

        self.twists = []
        self.fixes = []
        self.imus = []
        self.wheels = []

    def saveData(self):

        time_elapsed = time() - self.start_time
        if time_elapsed < 195.0:
            print(int(195.0 - time_elapsed))
            return

        pd.DataFrame(
            self.imus,
            columns=[
                "t",
                "angular_vel_x",
                "angular_vel_y",
                "angular_vel_z",
                "linear_acc_x",
                "linear_acc_y",
                "linear_acc_z",
            ],
        ).to_pickle("imu.pkl")

        pd.DataFrame(
            self.twists,
            columns=[
                "t",
                "linear_vel_x",
                "linear_vel_y",
                "linear_vel_z",
            ],
        ).to_pickle("twists.pkl")

        # entry = [t, x, y, speed, yaw, x_err, y_err, speed_err, yaw_err]
        pd.DataFrame(
            self.fixes,
            columns=[
                "t",
                "pos_x",
                "pos_y",
                "speed",
                "yaw",
                "x_err",
                "y_err",
                "speed_err",
                "yaw_err",
            ],
        ).to_pickle("fixes.pkl")

        pd.DataFrame(
            self.wheels,
            columns=["t", "linear_vel_x", "linear_vel_y", "angular_vel_z"],
        ).to_pickle("wheels.pkl")

        exit()

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )

    def gpsFixCb(self, msg: GPSFix):
        # print("Saving GPS fix")

        lat0, lon0, alt0 = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)

        ego_x, ego_y, _, __ = utm.from_latlon(msg.latitude, msg.longitude)

        x = ego_x - origin_x
        y = ego_y - origin_y
        speed = msg.speed
        yaw = trueTrackToEnuRads(msg.track)
        yaw_err = msg.err_track * np.pi / 180.0

        x_err = msg.err_horz
        y_err = msg.err_vert
        speed_err = msg.err_speed
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        entry = [t, x, y, speed, yaw, x_err, y_err, speed_err, yaw_err]
        self.fixes.append(entry)
        # print(fix)

    def imuCb(self, msg: Imu):
        avel = msg.angular_velocity
        lacc = msg.linear_acceleration

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        entry = [t, avel.x, avel.y, avel.z, lacc.x, lacc.y, lacc.z]
        self.imus.append(entry)
        # print(entry)

    def wheelCb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cov = np.asarray(msg.twist.covariance).reshape((6, 6))

        lin = msg.twist.twist.linear
        yawrate = msg.twist.twist.angular.z
        entry = [t, lin.x, lin.y, yawrate]

        self.wheels.append(entry)

    def twistCb(self, msg: TwistWithCovarianceStamped):

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        linear = msg.twist.twist.linear

        entry = [t, linear.x, linear.y, linear.z]

        self.twists.append(entry)


def main(args=None):
    rclpy.init(args=args)

    node = ParticleFilterLocalizer()

    rclpy.spin(node)

    imu_df = pd.read_pickle("data/pickles/imu.pkl")
    print(imu_df)
    twists_df = pd.read_pickle("data/pickles/twists.pkl")
    print(twists_df)
    fixes_df = pd.read_pickle("data/pickles/fixes.pkl")
    print(fixes_df)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
