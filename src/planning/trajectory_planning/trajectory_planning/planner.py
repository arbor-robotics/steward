import numpy as np
import rclpy
from array import array as Array
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from scipy.linalg import norm
from matplotlib import pyplot as plt
from matplotlib import patches
import cv2
from enum import IntEnum
import json
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import utm
import math
from scipy.spatial.transform.rotation import Rotation as R
from skimage.draw import disk


# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, Point, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from steward_msgs.msg import (
    FailedChecks,
    HealthCheck,
    SystemwideStatus,
    TrajectoryCandidates,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String


class PlannerNode(Node):
    def __init__(self):
        super().__init__("trajectory_planner")

        self.setUpParameters()

        self.create_subscription(String, "planning/plan_json", self.planCb, 1)
        self.create_subscription(Twist, "/cmd_vel", self.cmdVelCb, 1)
        self.create_subscription(OccupancyGrid, "/cost/total", self.totalCostCb, 1)

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.twist_path_pub = self.create_publisher(Path, "/cmd_vel/path", 1)
        self.candidates_pub = self.create_publisher(Path, "/planning/candidates", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Hello, world!")

        self.goal_point = None
        self.ego_pos = None
        self.ego_yaw = None
        self.seedling_points = []
        self.total_cost_map = None

        self.create_timer(0.1, self.updateTrajectory)

    def totalCostCb(self, msg: OccupancyGrid):
        arr = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)

        self.total_cost_map = arr

    def cmdVelCb(self, msg: Twist):

        # if self.ego_pos is None:
        #     self.get_logger().warning(f"Ego position unavailable.")
        #     return

        # if self.ego_yaw is None:
        #     self.get_logger().warning(f"Ego yaw unavailable.")
        #     return

        # Visualize the twist trajectory up to some time horizon
        TIME_HORIZON = 0.5  # sec
        dt = 0.05  # sec

        # v = msg.linear.x
        # omega = msg.angular.z

        # FAKE DATA

        V = np.linspace(0.2, 1.0, 5)
        Omega = np.linspace(-0.2, 0.1, 5)

        candidates = []

        for v in V:
            for omega in Omega:
                pose = np.zeros(4)
                poses = []

                for t in np.arange(0, TIME_HORIZON, dt):
                    poses.append(pose.copy())
                    pose[0] += v * math.cos(pose[2])
                    pose[1] += v * math.sin(pose[2])
                    pose[2] += omega
                    pose[3] = t + dt

                poses = np.asarray(poses)
                candidates.append(poses)

        v = 1.0
        omega = 0.1

        pose = np.zeros(4)
        poses = []

        for t in np.arange(0, TIME_HORIZON, dt):
            poses.append(pose.copy())
            pose[0] += v * math.cos(pose[2])
            pose[1] += v * math.sin(pose[2])
            pose[2] += omega
            pose[3] = t + dt

        poses = np.asarray(poses)

        # Form a Path message
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        fig, ax = plt.subplots()
        x = range(100)

        if self.total_cost_map is None:
            return

        ax.imshow(self.total_cost_map, extent=[-8, 12, -10, 10], cmap="summer")

        # candidate
        for pose in poses:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "base_link"
            pose_msg.header.stamp = self.dt_to_time_msg(pose[3])

            pose_msg.pose.position.x = pose[0]
            pose_msg.pose.position.y = pose[1]
            pose_msg.pose.orientation.z = math.cos(pose[2] / 2)
            pose_msg.pose.orientation.w = math.sin(pose[2] / 2)
            path_msg.poses.append(pose_msg)

        for candidate in candidates:
            ax.plot(candidate[:, 0], candidate[:, 1], color="lightcoral", linewidth=2)

        ax.plot(poses[:, 0], poses[:, 1], color="red", linewidth=5)

        rect = patches.Rectangle(
            (-0.5, -0.5), 1, 1, linewidth=3, edgecolor="white", facecolor="none"
        )
        ax.add_patch(rect)

        self.twist_path_pub.publish(path_msg)

        total_cost = self.get_total_cost(poses)
        print(total_cost)

        # ax.plot(x, x, "--", linewidth=5, color="firebrick")
        plt.show()

    def get_total_cost(self, poses: np.ndarray, collision_radius: float = 1.0):

        collision_radius_px = collision_radius / 0.2

        if self.total_cost_map is None:
            return

        grid_coords = poses.copy()[:, :2]
        grid_coords /= 0.2
        grid_coords[:, 0] += 40
        grid_coords[:, 1] += 50

        img = np.zeros_like(self.total_cost_map)

        total_cost = 0

        for pixel_coord in grid_coords:
            print(pixel_coord)
            rr, cc = disk(pixel_coord, collision_radius_px, shape=img.shape)
            img[rr, cc] = 1
            total_cost += np.sum(self.total_cost_map[rr, cc])

        # plt.scatter(grid_coords[:, 0], grid_coords[:, 1])
        # plt.imshow(img)
        # plt.show()

        return total_cost

    def dt_to_time_msg(self, dt: float):
        msg = self.get_clock().now().to_msg()
        msg.nanosec += int(1e9 * (dt - math.floor(dt)))
        msg.sec += math.floor(dt)

        return msg

    def latLonToMap(self, lat: float, lon: float):
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)

        x, y, _, __ = utm.from_latlon(lat, lon)

        x = x - origin_x
        y = y - origin_y

        return (x, y)

    def planCb(self, msg: String):

        try:
            plan_obj = json.loads(msg.data)
        except json.decoder.JSONDecodeError as e:
            self.get_logger().error(f"Could not process plan message: {e}")
            return

        try:
            bl_to_map_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            ego_x = bl_to_map_tf.transform.translation.x
            ego_y = bl_to_map_tf.transform.translation.y
            self.ego_pos = (ego_x, ego_y)

            q = bl_to_map_tf.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            self.ego_yaw = r.as_euler("xyz")[2]

        except TransformException as ex:
            self.get_logger().warning(f"Could not get ego position: {ex}")
            return

        try:
            seedlings: list[object] = plan_obj["seedlings"]
            # self.get_logger().info(f"Got plan: {seedlings}")

            min_dist = 9999.9

            self.seedling_points = []

            for seedling in seedlings:
                seedling_x, seedling_y = self.latLonToMap(
                    seedling["lat"], seedling["lon"]
                )

                self.seedling_points.append([seedling_x, seedling_y])

                dist = pdist([[ego_x, ego_y], [seedling_x, seedling_y]])[0]

                if dist < min_dist:
                    min_dist = dist
                    self.goal_point = [seedling_x, seedling_y]

            # self.get_logger().info(
            #     f"Min dist was: {min_dist}. Goal point is now {self.goal_point}"
            # )

        except KeyError as e:
            self.get_logger().error(f"{e}")

    def updateTrajectory(self):

        if self.goal_point is None:
            return

        try:
            bl_to_map_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            ego_x = bl_to_map_tf.transform.translation.x
            ego_y = bl_to_map_tf.transform.translation.y
            self.ego_pos = (ego_x, ego_y)

            q = bl_to_map_tf.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            self.ego_yaw = r.as_euler("xyz")[2]

        except TransformException as ex:
            self.get_logger().warning(f"Could not get ego position: {ex}")
            return

        if self.goal_point is None or self.ego_yaw is None:
            return

        target_yaw = math.atan2(
            self.goal_point[1] - self.ego_pos[1],
            self.goal_point[0] - self.ego_pos[0],
        )

        dist_err = pdist([self.goal_point, self.ego_pos])
        yaw_err = target_yaw - self.ego_yaw

        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi

        # Construct a Twist message to send
        msg = Twist()

        yaw_kP = 1.3
        msg.angular.z = yaw_err * yaw_kP

        lin_kP = 0.5

        if abs(yaw_err) < math.pi / 4:
            self.get_logger().info("Driving forward!")
            msg.linear.x = min(float(dist_err), 0.5)

        else:
            msg.linear.x = 0.0

        if dist_err < 1.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.onGoalPointReached()

        # self.twist_pub.publish(msg)
        # self.get_logger().info(
        #     f"Lin err: {dist_err}, yaw err: {yaw_err}. CMD: {msg.linear.x}, {msg.angular.z}"
        # )

    def onGoalPointReached(self):

        if len(self.seedling_points) < 1:
            self.get_logger().warn("All seedlings reached. End.")
            self.goal_point = None
            return
        # Remove the seedling point that we just reached
        self.seedling_points.remove(self.goal_point)

        # Set the goal point to be the next closest seedling point
        min_dist = 9999.9
        for seedling_pt in self.seedling_points:
            seedling_x, seedling_y = seedling_pt

            dist = pdist([self.ego_pos, [seedling_x, seedling_y]])[0]

            if dist < min_dist:
                min_dist = dist
                self.goal_point = [seedling_x, seedling_y]

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
