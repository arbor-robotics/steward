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

    def createForestPlan(self, seedling_count=550) -> ForestPlan:
        msg = ForestPlan()
        msg.header = self.getHeader()

        # TODO: Pull from parameter
        # bounds_image_path = self.get_parameter("bounds_map_path").value
        # self.get_logger().info(str(type(bounds_image_path)))
        # self.get_logger().info(bounds_image_path)
        # bounds_img = Image.open(bounds_image_path)

        imagery_img = Image.open("data/maps/schenley/imagery.png")
        # imagery_img = ImageOps.grayscale(imagery_img)
        imagery_img = np.asarray(imagery_img)
        print(imagery_img.shape)
        imagery_img_rescaled = rescale(imagery_img, 2.5, channel_axis=2)
        plt.imshow(imagery_img_rescaled)
        plt.show()

        imagery_img_pil = Image.fromarray(np.uint8(imagery_img_rescaled)).convert("RGB")
        imagery_img_pil.save("data/maps/schenley/imagery_rescaled.png")

        bounds_img = Image.open("data/maps/schenley/planting-bounds.png")
        bounds_img = ImageOps.grayscale(bounds_img)

        bounds_img = np.asarray(bounds_img)

        # meters per pixel side. TODO: Read this from MapMetaData message
        IMAGE_RES = 1.0

        TARGET_RES = 0.4  # meters per pixel side

        scale_factor = IMAGE_RES / TARGET_RES

        bounds_img_rescaled = rescale(bounds_img, scale_factor)

        print(bounds_img_rescaled.shape)

        # if (bounds_img.shape

        plt.imshow(bounds_img_rescaled)
        plt.show()
        # print(bounds_img)

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
