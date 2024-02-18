import numpy as np
import rclpy
from route_planning.ant_colony import AntColony
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from steward_msgs.msg import Route
from tf2_ros import TransformBroadcaster
import networkx as nx
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
import fast_tsp


class AcoRoutePlanner(Node):

    def __init__(self):
        super().__init__('aco_route_planner')

        # TODO: Create custom message type
        # forest_plan_sub = self.create_subscription()

        self.full_route_pub = self.create_publisher(
            Route, '/planning/full_route', 10)

        planting_points = self.createPlantingPoints(
            10000)  # fifty thousand trees, approx one acre

        distances = pdist(planting_points)
        distances = squareform(distances)

        # np.fill_diagonal(distances, np.inf)

        print(distances)

        # distances = np.array([[np.inf, 2, 2, 5, 7],
        #                       [2, np.inf, 4, 8, 2],
        #                       [2, 4, np.inf, 1, 3],
        #                       [5, 8, 1, np.inf, 2],
        #                       [7, 2, 3, 2, np.inf]])
        # print(distances)

        start = time()
        tour = fast_tsp.find_tour(distances.astype(int))
        print(tour)
        print(f"ACO took {time() - start} seconds")
        # print("shorted_path: {}".format(shortest_path))

    def acoProgressCb(self, iter: int):
        print(iter)

    def createPlantingPoints(self, quantity: int, distance: float = 100.0) -> np.ndarray:
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

    broadcaster = AcoRoutePlanner()

    rclpy.spin(broadcaster)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
