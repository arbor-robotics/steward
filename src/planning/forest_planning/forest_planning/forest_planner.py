import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from steward_msgs.msg import ForestPlan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
from std_msgs.msg import Header
from skimage import data, color
from skimage.draw import disk
from skimage.transform import rescale, resize, downscale_local_mean
import cv2


class BoundCell:
    DO_NOT_PLANT = 0
    PLANT_HERE = 1
    PASS_ONLY = 2


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("route_planner")

        self.setUpParameters()

        self.create_subscription(OccupancyGrid, "/map/height", self.heightMapCb, 10)
        self.create_subscription(
            OccupancyGrid, "/map/planting_bounds", self.plantingBoundsCb, 10
        )

        # TODO: Create proper QoS profile.
        self.forestPlanPub = self.create_publisher(
            OccupancyGrid, "/planning/forest_plan", 10
        )

        self.forestPlan = self.createForestPlan()

        PLAN_PUBLISH_RATE = 0.2  # Hz. TODO: Parameterize this.
        plan_pub_timer = self.create_timer(1 / PLAN_PUBLISH_RATE, self.publishPlan)

    def publishPlan(self) -> None:
        self.forestPlanPub.publish(self.forestPlan)

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = "map"  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def heightMapCb(self, msg: OccupancyGrid):
        self.get_logger().info("Got height map!")

    def plantingBoundsCb(self, msg: OccupancyGrid):
        self.get_logger().info("Got planting bounds map!")

    def createForestPlan(
        self, target_seedling_count=10000, do_plotting=False
    ) -> OccupancyGrid:

        self.get_logger().info("Generating Forest Plan.")
        startTime = time()

        msg = OccupancyGrid()

        # TODO: Send these as OccupancyGrid msgs
        # Read satellite imagery, heightmap, bounds map from disk
        imagery = cv2.imread("data/maps/flagstaff/imagery.jpg")
        heightmap = cv2.imread("data/maps/flagstaff/heightmap.jpg")
        bounds = cv2.imread(
            "data/maps/flagstaff/planting-bounds.jpg", 0
        )  # "0" means grayscale mode

        # Map bounds to our classification scheme, where:
        # 0 = don't enter, 1 = plant here, 2 = enter but don't plant
        bounds[bounds > 250] = BoundCell.PLANT_HERE
        bounds[bounds > 100] = BoundCell.PASS_ONLY
        bounds = bounds.astype(np.uint8)  # This may be redundant

        if bounds.shape[0] != bounds.shape[1]:
            self.get_logger().warning("Planting bounds map is not square.")

        GRID_SIZE = bounds.shape[0]  # px
        RES = 0.2  # m per pixel side

        if do_plotting:
            fig, axs = plt.subplots(
                ncols=2, nrows=2, figsize=(6.0, 6.0), layout="constrained"
            )

            axs[0, 0].set_title("Planting bounds")

            axs[0, 0].imshow(bounds)

            axs[1, 0].imshow(heightmap)
            axs[1, 0].set(title=f"Height map")

            axs[1, 1].imshow(imagery)
            axs[1, 1].set(title=f"Satellite imagery")

        # We're basically creating a grayscale picture.
        plan = np.zeros_like(bounds)

        current_seedling_count = 0

        MINIMUM_SPACING_METERS = 0.5
        MINIMUM_SPACING_PX = 5
        # MINIMUM_SPACING_PX = np.floor(MINIMUM_SPACING_METERS / RES)

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
        print(np.max(flattened_plan))
        print(np.min(flattened_plan))

        msg.data = flattened_plan.tolist()
        msg.header = self.getHeader()
        msg.info.height = plan.shape[0]
        msg.info.width = plan.shape[1]
        msg.info.resolution = RES
        # msg.info.origin TODO
        msg.info.map_load_time = msg.header.stamp

        self.get_logger().info(f"Forest Plan generated in {time() - startTime} seconds")

        return msg

    def setUpParameters(self):
        bounds_map_path_param_desc = ParameterDescriptor()
        bounds_map_path_param_desc.description = "File path to planting bounds image"
        bounds_map_path_param_desc.type = ParameterType.PARAMETER_STRING
        self.declare_parameter("bounds_map_path", True, bounds_map_path_param_desc)

    def createPlantingPoints(
        self, quantity: int, distance: float = 100.0, use_seed=True
    ) -> np.ndarray:
        if use_seed:
            rng = np.random.default_rng(99999)
        else:
            rng = np.random.default_rng()
        points = rng.uniform(-distance, distance, (quantity, 2))  # 2D coordinates
        return points


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
