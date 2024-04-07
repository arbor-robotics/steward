import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
from std_msgs.msg import Header
from skimage import data, color
from skimage.draw import disk
from skimage.transform import rescale, resize, downscale_local_mean
import cv2

# ROS interfaces
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from steward_msgs.action import CreateForestPlan
from steward_msgs.msg import ForestPlan


class BoundCell:
    DO_NOT_PLANT = 100
    PLANT_HERE = 0
    PASS_ONLY = 50


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("forest_planner")

        self.setUpParameters()

        self.create_subscription(OccupancyGrid, "/map/height", self.heightMapCb, 10)
        self.create_subscription(
            OccupancyGrid, "/map/bounds", self.plantingBoundsCb, 10
        )

        self.bounds_msg = None

        # https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # TODO: Create proper QoS profile.
        self.forest_plan_pub = self.create_publisher(
            OccupancyGrid, "/planning/forest_plan", map_qos
        )

        self.forest_plan = None

        PLAN_PUBLISH_RATE = self.get_parameter("plan_publish_freq").value
        self.create_timer(1 / PLAN_PUBLISH_RATE, self.publishPlan)

        self.action_server = ActionServer(
            self, CreateForestPlan, "create_forest_plan", self.actionCb
        )

    def actionCb(self, goal_handle: ServerGoalHandle):
        result = CreateForestPlan.Result()
        goal_handle.canceled()
        return result

    def publishPlan(self) -> None:
        if self.forest_plan is None:
            self.forest_plan = self.createForestPlan()
            if self.forest_plan is None:
                # If it's still none, then the inputs (like bounds)
                # are not yet available.
                return
        self.get_logger().debug("Publishing Forest Plan")
        self.forest_plan_pub.publish(self.forest_plan)

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = "map"  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def heightMapCb(self, msg: OccupancyGrid):
        self.heightmap = msg

    def plantingBoundsCb(self, msg: OccupancyGrid):
        self.bounds_msg = msg

    def createForestPlan(
        self, target_seedling_count=10000, do_plotting=False
    ) -> OccupancyGrid:

        if self.bounds_msg is None:
            self.get_logger().warning("Bounds message not yet received. Skipping.")
            return

        RES = self.get_parameter("plan_resolution").value
        MINIMUM_SPACING_METERS = self.get_parameter("minimum_spacing").value
        MINIMUM_SPACING_PX = np.floor(MINIMUM_SPACING_METERS / RES)

        self.get_logger().info(
            f"Generating Forest Plan with {MINIMUM_SPACING_METERS} meter spacing"
        )

        msg = OccupancyGrid()

        bounds = (
            np.asarray(self.bounds_msg.data, dtype=np.int8)
            .reshape(self.bounds_msg.info.height, self.bounds_msg.info.width)
            .astype(np.uint8)
        )
        bounds = np.flip(bounds, axis=0)

        if bounds.shape[0] != bounds.shape[1]:
            self.get_logger().warning("Planting bounds map is not square.")

        GRID_SIZE = bounds.shape[0]  # px

        if do_plotting:
            fig, axs = plt.subplots(
                ncols=2, nrows=2, figsize=(6.0, 6.0), layout="constrained"
            )

            axs[0, 0].set_title("Planting bounds")

            axs[0, 0].imshow(bounds)

            axs[1, 0].imshow(heightmap)
            axs[1, 0].set(title=f"Height map")

        # We're basically creating a grayscale picture.
        plan = np.zeros_like(bounds)

        current_seedling_count = 0

        rng = np.random.default_rng()

        MAX_ITERS = int(
            GRID_SIZE**2 / 10
        )  # This is a heuristic. Algorithm asymptotically approaches max density.
        nearby_aborts = 0

        for i in trange(MAX_ITERS):
            # pick a random pixel
            random_idx = rng.uniform(low=0, high=GRID_SIZE, size=2)
            random_idx = np.floor(random_idx).astype(int)

            # Check if within planting bounds
            if bounds[random_idx[0], random_idx[1]] != BoundCell.PLANT_HERE:
                continue

            # Check if too close to another seedling.
            # Includes if it's right on top of an exisiting seedling.
            # https://scikit-image.org/docs/stable/api/skimage.draw.html#skimage.draw.disk
            nearby_cells = disk(
                center=random_idx, radius=MINIMUM_SPACING_PX, shape=plan.shape
            )
            seedling_nearby = np.sum(plan[nearby_cells]) > 0

            if seedling_nearby:
                continue

            # Mark this cell as planted.
            plan[random_idx[0], random_idx[1]] = 1

            # Increment the seedling count
            current_seedling_count += 1

        if do_plotting:

            axs[0, 1].imshow(plan)
            axs[0, 1].set(title=f"Forest plan")

            # remove the x and y ticks for all subplots
            for i, ax in np.ndenumerate(axs):
                ax.set_xticks([])
                ax.set_yticks([])

            plt.show()

        flattened_plan = plan.flatten().astype(int)

        msg.data = flattened_plan.tolist()
        msg.header = self.getHeader()
        msg.info.height = plan.shape[0]
        msg.info.width = plan.shape[1]
        msg.info.resolution = RES

        msg.info.origin = self.bounds_msg.info.origin
        msg.info.map_load_time = msg.header.stamp

        self.get_logger().info(
            f"Forest Plan generated with {current_seedling_count} seedlings"
        )

        return msg

    def setUpParameters(self):
        bounds_map_path_param_desc = ParameterDescriptor()
        bounds_map_path_param_desc.description = "File path to planting bounds image"
        bounds_map_path_param_desc.type = ParameterType.PARAMETER_STRING
        self.declare_parameter("bounds_map_path", True, bounds_map_path_param_desc)

        minimum_spacing_param_desc = ParameterDescriptor()
        minimum_spacing_param_desc.description = (
            "Target planting density, in seedlings per meter."
        )
        minimum_spacing_param_desc.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("minimum_spacing", 1.0, minimum_spacing_param_desc)

        plan_publish_freq_param_desc = ParameterDescriptor()
        plan_publish_freq_param_desc.description = (
            "Frequency at which the Forest Plan result is published, in Hz."
        )
        plan_publish_freq_param_desc.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("plan_publish_freq", 0.5, plan_publish_freq_param_desc)

        plan_resolution_param_desc = ParameterDescriptor()
        plan_resolution_param_desc.description = (
            "Side length of cells within the Forest Plan, in meters."
        )
        plan_resolution_param_desc.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("plan_resolution", 0.2, plan_resolution_param_desc)


def main(args=None):
    rclpy.init(args=args)

    node = RoutePlanner()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
