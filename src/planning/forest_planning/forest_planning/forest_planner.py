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
from PIL import Image, ImageOps
from skimage import data, color
from skimage.transform import rescale, resize, downscale_local_mean


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("route_planner")

        self.setUpParameters()

        self.create_subscription(OccupancyGrid, "/map/height", self.heightMapCb, 10)
        self.create_subscription(
            OccupancyGrid, "/map/planting_bounds", self.plantingBoundsCb, 10
        )

        self.forest_plan = self.createForestPlan()

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = "map"  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def heightMapCb(self, msg: OccupancyGrid):
        self.get_logger().info("Got height map!")

    def plantingBoundsCb(self, msg: OccupancyGrid):
        self.get_logger().info("Got planting bounds map!")

    def createForestPlan(self, target_seedling_count=12000) -> ForestPlan:
        msg = OccupancyGrid()

        GRID_SIZE = 318  # px

        # We're basically creating a grayscale picture.
        plan = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)

        current_seedling_count = 0

        rng = np.random.default_rng()

        while current_seedling_count < target_seedling_count:
            # pick a random pixel
            random_idx = rng.uniform(low=0, high=GRID_SIZE, size=2)

            random_idx = np.floor(random_idx).astype(int)
            print(random_idx)
            # mark it
            plan[random_idx[0], random_idx[1]] = 1

            # increment the count
            current_seedling_count += 1

        plt.imshow(plan)
        plt.title(f"Forest plan with {target_seedling_count} seedlings, 0.2m/px")
        plt.xlabel("x (pixels)")
        plt.xlabel("y (pixels)")
        plt.show()

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
