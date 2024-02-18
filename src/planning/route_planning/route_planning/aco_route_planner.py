import numpy as np
import rclpy
from route_planning.ant_colony import AntColony
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from steward_msgs.msg import Route
from tf2_ros import TransformBroadcaster


class AcoRoutePlanner(Node):

    def __init__(self):
        super().__init__('aco_route_planner')

        # TODO: Create custom message type
        # forest_plan_sub = self.create_subscription()

        self.full_route_pub = self.create_publisher(
            Route, '/planning/full_route', 10)

        distances = np.array([[np.inf, 2, 2, 5, 7],
                              [2, np.inf, 4, 8, 2],
                              [2, 4, np.inf, 1, 3],
                              [5, 8, 1, np.inf, 2],
                              [7, 2, 3, 2, np.inf]])

        ant_colony = AntColony(distances, 1, 1, 100, 0.95, alpha=1, beta=1)
        shortest_path = ant_colony.run()
        print("shorted_path: {}".format(shortest_path))


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
