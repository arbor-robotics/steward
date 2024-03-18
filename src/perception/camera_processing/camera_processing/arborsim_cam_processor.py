import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image
import sys
import numpy as np
from matplotlib import pyplot as plt
import cv2
import cv_bridge
from std_msgs.msg import Header

# We use code from ros2_numpy, which converts ROS Image messages to/from ndarrays.
# https://github.com/Box-Robotics/ros2_numpy/blob/humble/ros2_numpy/image.py

name_to_dtypes = {
    "rgb8": (np.uint8, 3),
    "rgba8": (np.uint8, 4),
    "rgb16": (np.uint16, 3),
    "rgba16": (np.uint16, 4),
    "bgr8": (np.uint8, 3),
    "bgra8": (np.uint8, 4),
    "bgr16": (np.uint16, 3),
    "bgra16": (np.uint16, 4),
    "mono8": (np.uint8, 1),
    "mono16": (np.uint16, 1),
    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8": (np.uint8, 1),
    "bayer_bggr8": (np.uint8, 1),
    "bayer_gbrg8": (np.uint8, 1),
    "bayer_grbg8": (np.uint8, 1),
    "bayer_rggb16": (np.uint16, 1),
    "bayer_bggr16": (np.uint16, 1),
    "bayer_gbrg16": (np.uint16, 1),
    "bayer_grbg16": (np.uint16, 1),
    # OpenCV CvMat types
    "8UC1": (np.uint8, 1),
    "8UC2": (np.uint8, 2),
    "8UC3": (np.uint8, 3),
    "8UC4": (np.uint8, 4),
    "8SC1": (np.int8, 1),
    "8SC2": (np.int8, 2),
    "8SC3": (np.int8, 3),
    "8SC4": (np.int8, 4),
    "16UC1": (np.uint16, 1),
    "16UC2": (np.uint16, 2),
    "16UC3": (np.uint16, 3),
    "16UC4": (np.uint16, 4),
    "16SC1": (np.int16, 1),
    "16SC2": (np.int16, 2),
    "16SC3": (np.int16, 3),
    "16SC4": (np.int16, 4),
    "32SC1": (np.int32, 1),
    "32SC2": (np.int32, 2),
    "32SC3": (np.int32, 3),
    "32SC4": (np.int32, 4),
    "32FC1": (np.float32, 1),
    "32FC2": (np.float32, 2),
    "32FC3": (np.float32, 3),
    "32FC4": (np.float32, 4),
    "64FC1": (np.float64, 1),
    "64FC2": (np.float64, 2),
    "64FC3": (np.float64, 3),
    "64FC4": (np.float64, 4),
}


def imageToNumpy(msg):
    if not msg.encoding in name_to_dtypes:
        raise TypeError("Unrecognized encoding {}".format(msg.encoding))

    dtype_class, channels = name_to_dtypes[msg.encoding]
    dtype = np.dtype(dtype_class)
    dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
    shape = (msg.height, msg.width, channels)

    data = np.frombuffer(msg.data, dtype=dtype).reshape(shape)
    data.strides = (msg.step, dtype.itemsize * channels, dtype.itemsize)

    if channels == 1:
        data = data[..., 0]
    return data


def numpyToImage(arr: np.ndarray, encoding):
    msg = Image()
    msg.data = arr.flatten()


class ArborsimCamProcessor(Node):
    def __init__(self):
        super().__init__("pose_to_transform_broadcaster")

        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # TODO: Parameterize topic name
        self.raw_camera_sub = self.create_subscription(
            Image, "/camera/front_center/image_color", self.imageCb, sensor_qos_profile
        )
        self.corrected_image_pub = self.create_publisher(
            Image, "/camera/front_center/image_color_corrected", sensor_qos_profile
        )

        self.cv_bridge = cv_bridge.CvBridge()

    def getHeader(self) -> Header:
        """Forms a message Header.

        Returns:
            Header: The stamped and framed Header.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        return header

    def imageCb(self, msg: Image):
        arr = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        arr = cv2.flip(arr, 0)  # Flip the image vertically
        corrected_img = self.cv_bridge.cv2_to_imgmsg(arr, "rgb8")
        corrected_img.header = self.getHeader()
        self.corrected_image_pub.publish(corrected_img)
        # self.get_logger().info(str(arr.shape))
        # plt.imshow(arr)
        # plt.show()


def main(args=None):
    rclpy.init(args=args)

    node = ArborsimCamProcessor()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
