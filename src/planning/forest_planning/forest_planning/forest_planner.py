import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from steward_msgs.msg import ForestPlan
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

        forest_plan_sub = self.create_subscription(
            ForestPlan, '/planning/forest_plan', self.forestPlanCb, 10)

        self.full_route_pub = self.create_publisher(
            Route, '/planning/full_route', 10)

        fake_plan = self.createFakeForestPlan()
        self.forestPlanCb(fake_plan)

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = 'map'  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg

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

    def forestPlanCb(self, forest_plan_msg: ForestPlan, do_plotting=False) -> None:
        self.get_logger().debug(
            f"Got a forest plan with {len(forest_plan_msg.points)} seedlings")

        # Convert list of ROS Point messages to np array
        planting_points = []
        for point_msg in forest_plan_msg.points:
            point_msg: Point
            planting_points.append([point_msg.x, point_msg.y])
        planting_points = np.asarray(planting_points)

        print("Calculating distances.")
        distances = pdist(planting_points)
        print("Converting distances to square form.")
        use_fasttsp = self.get_parameter(
            'use_fasttsp').value

        start = time()  # For measuring runtime

        if use_fasttsp:
            distances = squareform(distances.astype(
                np.uint16), force='tomatrix', checks=False)
            self.get_logger().debug(f"Using Fast-TCP.")
            route = np.asarray(
                fast_tsp.find_tour(distances.astype(int)))

        else:
            distances = squareform(distances, force='tomatrix', checks=False)
            np.fill_diagonal(distances, np.inf)  # Required by AntColony class
            ant_colony = AntColony(
                distances, n_ants=1, n_best=1, n_iterations=1000, decay=0.95, alpha=1, beta=1)
            route = ant_colony.run()
            route = np.asarray(route[0])[:, 0]

        self.get_logger().debug(f"Routing took {time() - start} seconds")

        # Form a Route message
        route_msg = Route()
        route_msg.header = self.getHeader()

        for route_idx in route:
            route_msg.points.append(forest_plan_msg.points[route_idx])
            route_msg.ids.append(forest_plan_msg.ids[route_idx])

        # Publish the Route message
        assert len(route) == len(route_msg.ids), "Route ID length mismatch"
        self.full_route_pub.publish(route_msg)

        if do_plotting:

            plt.style.use('seaborn')

            # Re-order planting points w.r.t. route
            planting_points = planting_points[route]

            plt.plot(planting_points[:, 0], planting_points[:, 1])
            plt.scatter(planting_points[:, 0], planting_points[:, 1])
            plt.scatter(planting_points[0, 0],
                        planting_points[0, 1], c='g', s=250)

            plt.xlabel("x (meters)")
            plt.ylabel("y (meters)")
            plt.title(
                f"Iterative stochastic local search result for {len(route)} seedlings")
            plt.show()

    def setUpParameters(self):
        use_fasttsp_param_desc = ParameterDescriptor()
        use_fasttsp_param_desc.description = \
            "Use Fast-TCP instead of Ant Colony Optimization. Defaults to True."
        use_fasttsp_param_desc.type = ParameterType.PARAMETER_BOOL
        use_fasttspparam = self.declare_parameter(
            "use_fasttsp", True, use_fasttsp_param_desc)

    def acoProgressCb(self, iter: int):
        print(iter)

    def createPlantingPoints(self, quantity: int, distance: float = 100.0, use_seed=True) -> np.ndarray:
        if use_seed:
            rng = np.random.default_rng(99999)
        else:
            rng = np.random.default_rng()
        points = rng.uniform(-distance, distance,
                             (quantity, 2))  # 2D coordinates
        return points

    def getDistances(self, points: np.ndarray):
        assert points.ndim == 2 and points.shape[1] == 2, "Points must be 2D."

        N = points.shape[0]  # number of points

        distances = np.zeros((N, N))

        for i in trange(len(points), desc="Getting distances"):
            for j in range(len(points)):
                if i == j:  # points along a diagonal should be np.inf
                    distances[i, j] = np.inf
                else:
                    ptA = points[i, :]
                    ptB = points[j, :]
                    distance = np.linalg.norm(ptA-ptB)
                    distances[i, j] = distance

        return distances


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
