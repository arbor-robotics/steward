import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from os import path
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt


# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html


class HeightmapPublisher(Node):
    def __init__(self):
        super().__init__("heightmap_publisher")

        self.map_pub = self.create_publisher(OccupancyGrid, "/map/height", 10)

        self.declareParams()

        self.grid_msg = self.loadHeightmapToMessage()

        self.create_timer(5, self.publishMap)

    def publishMap(self) -> None:
        self.map_pub.publish(self.grid_msg)
        self.get_logger().debug("Publishing map!")

    def declareParams(self) -> None:
        # Declare the map name as a ROS parameter
        descr = ParameterDescriptor()
        descr.description = "The directory containing heightmap.png"
        descr.type = ParameterType.PARAMETER_STRING
        self.declare_parameter("map_dir", descriptor=descr)

        # Map resolution
        descr = ParameterDescriptor()
        descr.description = "Map resolution (m/pixel)"
        descr.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("resolution", descriptor=descr)

        # Map origin
        descr = ParameterDescriptor()
        descr.description = "Map origin [x,y,z]"
        descr.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter("origin", descriptor=descr)

    def getHeader(self) -> Header:
        """Forms a message Header.

        Returns:
            Header: The stamped and framed Header.
        """
        header = Header()

        # Our map is in the... map frame. Yes, really!
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()

        return header

    def loadHeightmapToMessage(self) -> OccupancyGrid:

        # Actually load the value of the ROS param
        map_dir = self.get_parameter("map_dir").value

        self.get_logger().info(map_dir)
        # print(map_dir)
        im_frame = Image.open(path.join(map_dir, "heightmap.png"))
        np_frame = np.array(im_frame, dtype=np.float32) / 255.0

        # Convert to grayscale from RGB, RGBA etc if necessary
        if np_frame.ndim > 2:
            np_frame = np_frame[:, :, 0]

        grid_msg = OccupancyGrid()
        grid_msg.header = self.getHeader()
        grid_msg.info.height = np_frame.shape[0]
        grid_msg.info.width = np_frame.shape[1]
        grid_msg.info.resolution = self.get_parameter("resolution").value
        grid_msg.info.map_load_time = self.get_clock().now().to_msg()

        origin = self.get_parameter("origin").value
        grid_msg.info.origin.position.x = origin[0]
        grid_msg.info.origin.position.y = origin[1]
        grid_msg.info.origin.position.z = origin[2]

        # Normalize and format the height data
        np_frame = np_frame.flatten()
        np_frame -= np.min(np_frame)
        np_frame /= np.max(np_frame)
        np_frame *= 100
        np_frame = np_frame.astype(int)
        # np_frame -= 128

        # plt.imshow(np_frame.reshape((1000, -1)))
        # plt.show()

        # assert np_frame.min() > - \
        #     129, f"The heightmap must have values above -129. Min was {np_frame.min()}"
        # assert np_frame.max(
        # ) < 128, f"The heightmap must have values below 128. Max was {np_frame.max()}"

        grid_msg.data = np_frame.flatten().tolist()

        return grid_msg


def main(args=None):
    rclpy.init(args=args)

    heightmap_publisher = HeightmapPublisher()

    rclpy.spin(heightmap_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heightmap_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
