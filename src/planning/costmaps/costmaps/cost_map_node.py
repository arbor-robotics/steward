import numpy as np
import rclpy
from array import array as Array
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import cv2
from enum import IntEnum
import json
import math
from matplotlib import pyplot as plt
from scipy.spatial.distance import pdist, squareform
from scipy.spatial.transform.rotation import Rotation as R
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from time import time
from tqdm import tqdm, trange
import utm

# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header, String
from steward_msgs.msg import FailedChecks, HealthCheck, SystemwideStatus
from sensor_msgs.msg import PointCloud2, PointField


class CostMapNode(Node):
    def __init__(self):
        super().__init__("cost_map_node")

        self.setUpParameters()

        self.create_subscription(String, "/planning/plan_json", self.planCb, 1)
        self.create_subscription(OccupancyGrid, "/cost/occupancy", self.occCb, 1)

        self.seedling_dist_map_pub = self.create_publisher(
            OccupancyGrid, "/cost/dist_to_seedlings", 1
        )

        self.total_cost_pub = self.create_publisher(OccupancyGrid, "/cost/total", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.updateCosts)

        self.seedling_points = None
        self.seedling_pts_bl = None
        self.cached_occ = np.zeros((100, 100))

    def occCb(self, msg: OccupancyGrid):
        arr = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)

        self.cached_occ = arr

    def latLonToMap(self, lat: float, lon: float):
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)

        x, y, _, __ = utm.from_latlon(lat, lon)

        x = x - origin_x
        y = y - origin_y

        return (x, y)

    def getDistanceToSeedlingMap(self) -> np.ndarray:
        if self.seedling_points is None:
            # self.get_logger().warning(
            #     "Seedling locations not available. Skipping distance to seedling cost."
            # )
            return np.ones((100, 100)) * 100

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
            return np.ones((100, 100)) * 100

        nearby_seedlings = []

        for seedling_pt in self.seedling_points:
            seedling_x, seedling_y = seedling_pt

            dist = pdist([self.ego_pos, [seedling_x, seedling_y]])[0]

            if dist < 20:
                nearby_seedlings.append(seedling_pt)

        # self.get_logger().info(f"There are {len(nearby_seedlings)} nearby seedlings.")

        RES = 0.2  # meters per pixel
        ORIGIN_X_PX = 40
        ORIGIN_X_M = ORIGIN_X_PX * RES
        ORIGIN_Y_PX = 50
        ORIGIN_Y_M = ORIGIN_Y_PX * RES
        GRID_WIDTH = 100
        GRID_HEIGHT = GRID_WIDTH

        distance_to_seedling_map = np.zeros((GRID_HEIGHT, GRID_WIDTH))

        # Transform points to base_link
        self.seedling_pts_bl = self.transformToBaselink(nearby_seedlings)

        if self.seedling_pts_bl is None:
            self.get_logger().warning("No nearby points.")
            distance_to_seedling_map += 100
        else:
            points = self.seedling_pts_bl

            points /= RES
            points = points.astype(np.int8)

            # Discard indices outside of bounds

            # Offset by origin
            points[:, 0] += ORIGIN_X_PX
            points[:, 1] += ORIGIN_Y_PX

            points = points[
                np.logical_and(points[:, 0] > 0, points[:, 0] < GRID_HEIGHT)
            ]
            points = points[np.logical_and(points[:, 1] > 1, points[:, 1] < GRID_WIDTH)]

            distance_to_seedling_map[tuple(points.T)] = 1000

        distance_to_seedling_map = cv2.GaussianBlur(
            distance_to_seedling_map, (99, 99), 0
        )

        # Normalize
        MAX_DISTANCE_COST = 60
        distance_to_seedling_map /= np.max(distance_to_seedling_map)
        distance_to_seedling_map *= MAX_DISTANCE_COST
        distance_to_seedling_map = (
            np.ones_like(distance_to_seedling_map) * MAX_DISTANCE_COST
            - distance_to_seedling_map
        )
        distance_to_seedling_map = distance_to_seedling_map.astype(np.uint8)
        # grid = grid.astype(np.uint8)

        # FLIP AXES
        distance_to_seedling_map = distance_to_seedling_map.T

        return distance_to_seedling_map.astype(np.uint8)

    def updateCosts(self, do_plot=False):

        RES = 0.2  # meters per pixel
        ORIGIN_X_PX = 40
        ORIGIN_X_M = ORIGIN_X_PX * RES
        ORIGIN_Y_PX = 50
        ORIGIN_Y_M = ORIGIN_Y_PX * RES
        GRID_WIDTH = 100
        GRID_HEIGHT = GRID_WIDTH

        distance_to_seedling_map = self.getDistanceToSeedlingMap()

        # Now add the layers together
        total_cost_map = distance_to_seedling_map + self.cached_occ
        total_cost_map[total_cost_map > 100] = 100

        if np.min(total_cost_map) < 0:
            self.get_logger().error(f"Total cost had elements less than 0. Correcting.")
            total_cost_map[total_cost_map < 0] = 0

        origin = Point(x=-ORIGIN_X_M, y=-ORIGIN_Y_M)

        info = MapMetaData(resolution=RES, width=GRID_WIDTH, height=GRID_HEIGHT)
        info.origin.position = origin
        msg = OccupancyGrid()

        # self.get_logger().info(f"{grid}")
        if do_plot:
            fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
            ax1.set_title("Distance to planting locations")
            ax2.set_title("Occupancy")
            ax3.set_title("Total cost")

            ax1.imshow(distance_to_seedling_map, cmap="summer")
            ax2.imshow(self.cached_occ, cmap="summer")
            pos = ax3.imshow(total_cost_map, cmap="summer")
            # fig.colorbar(pos, ax=ax3, label="Cost", shrink=0.6)
            plt.show()

        try:
            msg.data = distance_to_seedling_map.astype(np.uint8).flatten().tolist()
        except AssertionError as e:
            self.get_logger().warning(
                f"{e}. Min was {np.min(distance_to_seedling_map)}, max was {np.max(distance_to_seedling_map)}, dtype was {distance_to_seedling_map.dtype}"
            )
        msg.info = info

        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        self.seedling_dist_map_pub.publish(msg)

        try:
            data_list = total_cost_map.astype(np.uint8).flatten().tolist()
            msg.data = data_list
        except AssertionError as e:
            self.get_logger().warning(
                f"{e}. Min was {np.min(total_cost_map)}, max was {np.max(total_cost_map)}"
            )

        self.total_cost_pub.publish(msg)
        # ax2.imshow(grid)
        # plt.show()

    # def numpyToCostmap(self, arr: np.ndarray):

    def transformToBaselink(self, points):

        if len(points) < 1:
            return None

        # Form a 2D homogeneous transform matrix
        t = -self.ego_yaw
        u = -self.ego_pos[0]
        v = -self.ego_pos[1]

        H = np.identity(3)
        H[0, 2] = u
        H[1, 2] = v

        R = np.asarray(
            [
                [math.cos(t), -math.sin(t)],
                [math.sin(t), math.cos(t)],
            ]
        )

        points = np.asarray(points)

        # fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
        # ax1.scatter(points[:, 0], points[:, 1])
        # ax1.scatter(*self.ego_pos, c="red")
        # ax1.set_title(f"Yaw: {self.ego_yaw}")

        # Transform to base_link
        try:
            pts_homog = np.vstack((points.T, np.ones(len(points))))
            # self.get_logger().info(f"{pts_homog}")
            pts_tfed = H @ pts_homog
            pts_tfed = pts_tfed.T
            pts_tfed = pts_tfed[:, :-1]
            # self.get_logger().info(f"{self.ego_yaw}")
            # self.get_logger().info(f"{pts_tfed}")

        except ValueError as e:
            # There were no points nearby
            return None

        # ax2.scatter(pts_tfed[:, 0], pts_tfed[:, 1])
        # ax2.scatter(0, 0, c="red")
        # ax2.set_xlim((-20, 20))
        # ax2.set_ylim((-20, 20))

        pts_tfed = (R @ pts_tfed.T).T
        # ax3.scatter(pts_tfed[:, 0], pts_tfed[:, 1])
        # ax3.scatter(0, 0, c="red")
        # ax3.set_xlim((-20, 20))
        # ax3.set_ylim((-20, 20))

        # plt.show()

        return pts_tfed

    def planCb(self, msg: String):

        print("Received plan!")

        try:
            plan_obj = json.loads(msg.data)
        except json.decoder.JSONDecodeError as e:
            self.get_logger().error(f"Could not process plan message: {e}")
            return

        try:
            seedlings: list[object] = plan_obj["seedlings"]

            self.seedling_points = []
            print("Updated seedling_points!")

            for seedling in seedlings:
                seedling_x, seedling_y = self.latLonToMap(
                    seedling["lat"], seedling["lon"]
                )

                self.seedling_points.append([seedling_x, seedling_y])

        except KeyError as e:
            self.get_logger().error(f"{e}")

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )


def main(args=None):
    rclpy.init(args=args)

    node = CostMapNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
