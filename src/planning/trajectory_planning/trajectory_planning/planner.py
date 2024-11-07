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
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose, Point, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from steward_msgs.msg import (
    FailedChecks,
    HealthCheck,
    SystemwideStatus,
    TrajectoryCandidates,
    TrajectoryCandidate,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String, Float32


class Candidate:
    def __init__(
        self, speed: float, omega: float, trajectory: np.ndarray, cost: float = -1
    ):
        self.speed = speed
        self.omega = omega
        self.trajectory = trajectory
        self.cost = cost


class PlannerNode(Node):
    def __init__(self):
        super().__init__("trajectory_planner")

        self.setUpParameters()

        self.generateCandidates()

        self.create_subscription(String, "planning/plan_json", self.planCb, 1)
        self.create_subscription(Twist, "/cmd_vel", self.cmdVelCb, 1)
        self.create_subscription(OccupancyGrid, "/cost/total", self.totalCostCb, 1)
        self.create_subscription(
            GeoPoint, "/planning/goal_pose_geo", self.goalPointGeoCb, 1
        )
        self.create_subscription(Float32, "/gnss/yaw", self.egoYawCb, 1)

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.twist_path_pub = self.create_publisher(Path, "/cmd_vel/path", 1)
        self.candidates_pub = self.create_publisher(
            TrajectoryCandidates, "/planning/candidates", 1
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Hello, world!")

        self.goal_point = None
        self.ego_pos = None
        self.ego_yaw = None
        self.seedling_points = []
        self.total_cost_map = None
        self.grid_info = None

        self.create_timer(0.1, self.updateTrajectory)

    def egoYawCb(self, msg: Float32):
        # self.get_logger().info("Updated ego yaw")
        self.ego_yaw = msg.data

    def goalPointGeoCb(self, msg: GeoPoint):
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)
        x, y, _, __ = utm.from_latlon(msg.latitude, msg.longitude)

        x = x - origin_x
        y = y - origin_y
        self.goal_point = [x, y]
        print(msg)

    def totalCostCb(self, msg: OccupancyGrid):
        arr = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)

        self.total_cost_map = arr
        self.grid_info = msg.info

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
        Omega = np.linspace(-0.2, 0.2, 5)

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

        # TODO WSH: Add a "stop" candidate with v & omega = 0.

        v_user = msg.linear.x
        omega_user = msg.angular.z

        pose = np.zeros(4)
        poses = []

        for t in np.arange(0, TIME_HORIZON, dt):
            poses.append(pose.copy())
            pose[0] += v_user * math.cos(pose[2])
            pose[1] += v_user * math.sin(pose[2])
            pose[2] += omega_user
            pose[3] = t + dt

        poses = np.asarray(poses)

        # Form a Path message
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # fig, ax = plt.subplots()
        x = range(100)

        if self.total_cost_map is None:
            return

        # ax.imshow(self.total_cost_map, extent=[-8, 12, -10, 10], cmap="summer")

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

        # for candidate in candidates:
        #     ax.plot(candidate[:, 0], candidate[:, 1], color="lightcoral", linewidth=2)

        # ax.plot(poses[:, 0], poses[:, 1], color="red", linewidth=5)

        rect = patches.Rectangle(
            (-0.5, -0.5), 1, 1, linewidth=3, edgecolor="white", facecolor="none"
        )
        # ax.add_patch(rect)

        self.twist_path_pub.publish(path_msg)

        # total_cost = self.get_total_cost(poses)
        # print(total_cost)

        # ax.plot(x, x, "--", linewidth=5, color="firebrick")
        # plt.savefig("candidates.png")

    def generateCandidates(self, top_speed=1.5):

        dt = 0.1  # sec
        time_horizon = 5.0  # sec

        # Start with top speed
        trajectories = []
        candidates = []

        v = top_speed
        Omega = np.linspace(-0.1, 0.1, 3)
        candidates_at_speed = []
        for omega in Omega:
            pose = np.zeros(4)  # Start at ego position, zero speed, zero (relative) yaw
            trajectory = []

            for t in np.arange(0, time_horizon, dt):
                trajectory.append(pose.copy())
                pose[0] += v * math.cos(pose[2]) * dt
                pose[1] += v * math.sin(pose[2]) * dt
                pose[2] += omega * dt
                pose[3] = t + dt

            trajectory = np.asarray(trajectory)
            trajectories.append(trajectory)
            candidate = Candidate(v, omega, trajectory)
            mask = self.getCandidateMask(candidate)
            candidates_at_speed.append([omega, mask])
            # plt.imshow(mask, extent=[-8, 12, -10, 10])
            # plt.show()

        candidates.append([v, candidates_at_speed])

        # Now med speed
        v = top_speed * 0.667
        Omega = np.linspace(-0.2, 0.2, 5)
        candidates_at_speed = []
        for omega in Omega:
            pose = np.zeros(4)  # Start at ego position, zero speed, zero (relative) yaw
            trajectory = []

            for t in np.arange(0, time_horizon, dt):
                trajectory.append(pose.copy())
                pose[0] += v * math.cos(pose[2]) * dt
                pose[1] += v * math.sin(pose[2]) * dt
                pose[2] += omega * dt
                pose[3] = t + dt

            trajectory = np.asarray(trajectory)
            trajectories.append(trajectory)
            candidate = Candidate(v, omega, trajectory)
            mask = self.getCandidateMask(candidate)
            candidates_at_speed.append([omega, mask])
            # plt.imshow(mask, extent=[-8, 12, -10, 10])
            # plt.show()

        candidates.append([v, candidates_at_speed])

        # Now low speed
        v = top_speed * 0.333
        Omega = np.linspace(-0.3, 0.3, 7)
        candidates_at_speed = []
        for omega in Omega:
            pose = np.zeros(4)  # Start at ego position, zero speed, zero (relative) yaw
            trajectory = []

            for t in np.arange(0, time_horizon, dt):
                trajectory.append(pose.copy())
                pose[0] += v * math.cos(pose[2]) * dt
                pose[1] += v * math.sin(pose[2]) * dt
                pose[2] += omega * dt
                pose[3] = t + dt

            trajectory = np.asarray(trajectory)
            trajectories.append(trajectory)
            candidate = Candidate(v, omega, trajectory)
            mask = self.getCandidateMask(candidate)
            candidates_at_speed.append([omega, mask])
            # plt.imshow(mask, extent=[-8, 12, -10, 10])
            # plt.show()

        candidates.append([v, candidates_at_speed])

        plt.figure()
        plt.gca().set_aspect("equal")

        print(candidates)

        for trajectory in trajectories:
            trajectory = np.asarray(trajectory)
            plt.plot(trajectory[:, 0], trajectory[:, 1])

        plt.savefig("candidates.png")

        self.candidates = candidates

    def getCandidates(
        self,
        min_speed=0.5,
        max_speed=1.0,
        speed_steps=4,
        min_omega=-0.3,
        max_omega=0.3,
        omega_steps=7,
        time_horizon=10.0,
        dt=0.5,
    ) -> list[Candidate]:
        V = np.linspace(min_speed, max_speed, speed_steps)
        Omega = np.linspace(min_omega, max_omega, omega_steps)

        trajectories = []
        candidates: list[Candidate] = []

        for v in V:
            for omega in Omega:
                pose = np.zeros(
                    4
                )  # Start at ego position, zero speed, zero (relative) yaw
                trajectory = []

                for t in np.arange(0, time_horizon, dt):
                    trajectory.append(pose.copy())
                    pose[0] += v * math.cos(pose[2]) * dt
                    pose[1] += v * math.sin(pose[2]) * dt
                    pose[2] += omega * dt
                    pose[3] = t + dt

                trajectory = np.asarray(trajectory)
                trajectories.append(trajectory)
                candidates.append(Candidate(v, omega, trajectory))

        # plt.figure()
        # plt.gca().set_aspect("equal")

        # for trajectory in trajectories:
        #     trajectory = np.asarray(trajectory)
        # plt.plot(trajectory[:, 0], trajectory[:, 1])

        # plt.show()

        return candidates

    def getCandidateMask(self, candidate: Candidate, collision_radius: float = 1.0):

        collision_radius_px = collision_radius / 0.2

        grid_coords = candidate.trajectory.copy()[:, :2]
        grid_coords[:, 1] += 10
        grid_coords[:, 0] += 8
        grid_coords /= 0.2

        # grid_coords[:, 0] = self.grid_info.height - grid_coords[:, 1]

        # display_img = img.copy()
        mask = np.zeros((100, 100), dtype=bool)

        print(candidate.speed, candidate.omega)
        for pixel_coord in grid_coords:
            print(pixel_coord)
            rr, cc = disk(pixel_coord, collision_radius_px, shape=(100, 100))
            # display_img[cc, rr] = 50
            mask[cc, rr] = True

        return mask

    def getTotalCost(self, candidate: Candidate, collision_radius: float = 1.0):

        collision_radius_px = collision_radius / self.grid_info.resolution

        if self.total_cost_map is None:
            return

        grid_coords = candidate.trajectory.copy()[:, :2]
        grid_coords[:, 1] -= self.grid_info.origin.position.y
        grid_coords[:, 0] -= self.grid_info.origin.position.x
        grid_coords /= self.grid_info.resolution

        # grid_coords[:, 0] = self.grid_info.height - grid_coords[:, 1]

        img = self.total_cost_map.copy()
        # display_img = img.copy()
        mask = np.zeros_like(self.total_cost_map)

        total_cost = 0

        # fig = plt.figure()

        for pixel_coord in grid_coords:
            rr, cc = disk(pixel_coord, collision_radius_px, shape=img.shape)
            # display_img[cc, rr] = 50
            mask[cc, rr] = 1

            total_cost += np.sum(self.total_cost_map[cc, rr])

        total_cost = np.sum(self.total_cost_map[mask > 0])

        # plt.plot(candidate.trajectory[:, 0], candidate.trajectory[:, 1])
        # plt.imshow(display_img, extent=[-8, 12, -10, 10], cmap="summer")
        # plt.title(f"v={candidate.speed}, o={candidate.omega}, {total_cost}")
        # # plt.show()
        # plt.savefig("candidate_cost.png")
        # plt.close(fig)

        # Now penalize slow candidates

        if (candidate.speed) > 0.9:
            total_cost *= 0.5

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

    def getClosestSeedlingInBaselink(self):
        # if self.seedling_points is None or len(self.seedling_points) < 1:
        #     self.get_logger().warning(
        #         f"Could not find closest seedling point. Seedling points unknown."
        #     )
        #     return None

        # closest_distance = 999999.9
        # for seedling_pt in self.seedling_points:
        #     seedling_x, seedling_y = seedling_pt

        #     dist = pdist([self.ego_pos, [seedling_x, seedling_y]])[0]

        #     if dist < closest_distance:
        #         closest_distance = dist
        #         closest_seedling = seedling_pt

        # closest_seedling_bl = self.transformToBaselink([closest_seedling])
        # print(self.ego_yaw, closest_seedling_bl)

        if self.total_cost_map is None:
            self.get_logger().warning(
                f"Could not find closest seedling point. Seedling points unknown."
            )
            return None

        x = self.total_cost_map
        pixel_coords = np.unravel_index(x.argmin(), x.shape)
        return [pixel_coords[1] - 40, pixel_coords[0] - 50]

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

            self.get_logger().info(
                f"Min dist was: {min_dist}. Goal point is now {self.goal_point}"
            )

        except KeyError as e:
            self.get_logger().error(f"{e}")

    def getYawError(self, goal_point_bl):
        yaw_error = math.atan2(goal_point_bl[0], goal_point_bl[1]) - np.pi / 2
        yaw_error *= -1

        if yaw_error < -np.pi:
            yaw_error += 2 * np.pi
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        # print(f"{goal_point_bl} -> {yaw_error / np.pi * 180}")

        return yaw_error

    def pointTurnFromYawError(self, yaw_error, omega=0.4):
        cmd_msg = Twist()

        if yaw_error < 0:
            omega *= -1

        cmd_msg.angular.z = omega

        self.twist_pub.publish(cmd_msg)

    def updateTrajectory(self):

        if self.total_cost_map is None:
            self.get_logger().warning(
                "Could not plan trajectory. Total cost unavailable."
            )
            return
        elif self.ego_yaw is None:
            self.get_logger().warning("Could not plan trajectory. Ego yaw unavailable.")
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

        # Check yaw error. If |yaw err| > pi/4 (45 deg), point turn.
        goal_point = self.getClosestSeedlingInBaselink()
        print(goal_point)
        if goal_point is None:
            self.get_logger().warning(
                "Could not get goal point in base_link. Skipping trajectory generation."
            )
            return

        yaw_error = self.getYawError(goal_point)

        POINT_TURN_YAW_ERROR_THRESHOLD = np.pi / 8
        if abs(yaw_error) > POINT_TURN_YAW_ERROR_THRESHOLD:
            self.pointTurnFromYawError(yaw_error)
            return

        # cmd_msg = Twist()
        # self.twist_pub.publish(cmd_msg)
        # return

        # print(self.candidates)
        # exit()
        print(time())
        cmd_msg = Twist()
        for speed_level in self.candidates:
            speed, candidates = speed_level

            obstacle_free_candidates = []
            for candidate in candidates:
                # print(candidate)
                omega, mask = candidate
                # print(omega, mask)

                max_cost = np.max(self.total_cost_map[mask])
                # print(max_cost)

                if max_cost >= 100:
                    print(f"Obst [{speed}, {omega}] had obstacles")
                    continue  # Skip to next if there's an obstacle
                else:
                    total_cost = np.sum(self.total_cost_map[mask])
                    obstacle_free_candidates.append([speed, omega, total_cost])

            if len(obstacle_free_candidates) > 0:
                print(f"RESULTS: {obstacle_free_candidates}")

                # Select the result with the lowest cost
                best_candidate = None
                best_candidate_cost = 999999

                for candidate in obstacle_free_candidates:
                    if candidate[2] < best_candidate_cost:
                        best_candidate_cost = candidate[2]
                        best_candidate = candidate

                cmd_msg.linear.x = best_candidate[0]
                cmd_msg.angular.z = best_candidate[1]
                self.twist_pub.publish(cmd_msg)
                return
            else:
                print(f"No obstacle-free candidates found at speed {speed}")

        self.twist_pub.publish(cmd_msg)

        return

        # Get cost of each candidate
        start = time()
        for candidate in candidates:
            cost = self.getTotalCost(candidate)
            candidate.cost = cost

        print(f"Took {time() - start} sec")

        # Publish candidates
        candidates_msg = TrajectoryCandidates()
        for candidate in candidates:
            candidate_msg = TrajectoryCandidate(
                cost=float(candidate.cost), speed=candidate.speed, omega=candidate.omega
            )
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "base_link"

            for pose in candidate.trajectory:
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = pose[0]
                pose_msg.pose.position.y = pose[1]
                path_msg.poses.append(pose_msg)

            candidate_msg.trajectory = path_msg
            candidates_msg.candidates.append(candidate_msg)

        self.candidates_pub.publish(candidates_msg)

        # Choose candidate with lowest cost
        lowest_cost = 9999999
        best_candidate = None

        for candidate in candidates:
            if candidate.cost < lowest_cost:
                lowest_cost = candidate.cost
                best_candidate = candidate

        # Extract v, omega from selected candidate
        v = best_candidate.speed
        omega = best_candidate.omega

        # Convert v, theta to twist message
        twist_msg = Twist()
        twist_msg.angular.z = omega
        twist_msg.linear.x = v

        # Publish twist
        self.twist_pub.publish(twist_msg)

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
