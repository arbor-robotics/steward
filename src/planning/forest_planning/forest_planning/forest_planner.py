import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from steward_msgs.msg import ForestPlan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
from std_msgs.msg import Header


class RoutePlanner(Node):

    def __init__(self):
        super().__init__('route_planner')

        self.setUpParameters()

        height_map_sub = self.create_subscription(OccupancyGrid, '/map/height', self.heightMapCb, 10)
        planting_bounds_sub = self.create_subscription(OccupancyGrid, '/map/planting_bounds', self.plantingBoundsCb, 10)

        self.full_route_pub = self.create_publisher(
            Route, '/planning/full_route', 10)

        fake_plan = self.createFakeForestPlan()
        self.forestPlanCb(fake_plan)

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = 'map'  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg
    
    def heightMapCb(self, msg: OccupancyGrid):
        self.get_logger().info("Got height map!")

    def plantingBoundsCb(self, msg: OccupancyGrid):
        self.get_logger().info("Got planting bounds map!")

    def createFakeForestPlan(self, seedling_count=550) -> ForestPlan:
        msg = ForestPlan()
        msg.header = self.getHeader()

        planting_points = self.createPlantingPoints(
            seedling_count, distance=64)

        for idx, point in enumerate(planting_points):
            point_msg = Point()
            point_msg.x = point[0]
            point_msg.y = point[1]
            msg.points.append(point_msg)
            msg.ids.append(idx)

            # Empty for now. We can put JSON or something here later.
            msg.plant_data.append("{}")

        return msg


    def setUpParameters(self):
        pass
        # use_fasttsp_param_desc = ParameterDescriptor()
        # use_fasttsp_param_desc.description = \
        #     "Use Fast-TCP instead of Ant Colony Optimization. Defaults to True."
        # use_fasttsp_param_desc.type = ParameterType.PARAMETER_BOOL
        # use_fasttspparam = self.declare_parameter(
        #     "use_fasttsp", True, use_fasttsp_param_desc)

    def createPlantingPoints(self, quantity: int, distance: float = 100.0, use_seed=True) -> np.ndarray:
        if use_seed:
            rng = np.random.default_rng(99999)
        else:
            rng = np.random.default_rng()
        points = rng.uniform(-distance, distance,
                             (quantity, 2))  # 2D coordinates
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


if __name__ == '__main__':
    main()
