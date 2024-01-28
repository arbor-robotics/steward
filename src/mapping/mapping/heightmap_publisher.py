import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster


# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html


class HeightmapPublisher(Node):

    def __init__(self):
        super().__init__('heightmap_publisher')


def main(args=None):
    rclpy.init(args=args)

    heightmap_publisher = HeightmapPublisher()

    rclpy.spin(heightmap_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heightmap_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
