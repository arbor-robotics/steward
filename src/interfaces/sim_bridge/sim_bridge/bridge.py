import asyncio
from websockets.asyncio.server import serve, Server, ServerConnection

from matplotlib import pyplot as plt
from matplotlib import image as mpimg
import io
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from PIL import Image as PILImage


class WebsocketBridge(Node):
    def __init__(self):
        super().__init__("websocket_bridge")

        self.bridge = CvBridge()

        self.setUpParameters()

        # TODO: Create proper QoS profile.
        self.imagePub = self.create_publisher(Image, "/rgb/image_rect_color", 10)

        # CONTROLLER_FREQ = 100  # Hz
        self.create_timer(0.1, self.publishHeartbeat)
        # self.create_timer(0.01, self.checkMessages)

        # server = serve(self.handleConnection, "localhost", 8765)
        # server.server.get_loop().run_until_complete()

        # asyncio.create_task(self.runServer)

    def publishHeartbeat(self):
        # self.get_logger().info("I'm still alive!")
        pass

    async def handleConnection(self, websocket):
        async for message in websocket:
            await websocket.send(message)

    def publishImageBytes(self, bytes):
        bytes_arr = np.array(PILImage.open(io.BytesIO(bytes)))

        msg = self.bridge.cv2_to_imgmsg(bytes_arr, "rgba8")

        # bytes_arr = mpimg.imread()
        # plt.imshow(bytes_arr)
        # self.get_logger().info(str(bytes_arr))
        # msg: Image = self.numpy_to_image(bytes_arr)
        self.imagePub.publish(msg)
        # plt.show()

    def numpy_to_image(self, arr, encoding="rgba8"):
        # if not encoding in name_to_dtypes:
        #     raise TypeError('Unrecognized encoding {}'.format(encoding))

        im = Image(encoding=encoding)

        # extract width, height, and channels
        # dtype_class, exp_channels = name_to_dtypes[encoding]
        dtype_class, exp_channels = (np.uint8, 4)
        dtype = np.dtype(dtype_class)
        if len(arr.shape) == 2:
            im.height, im.width, channels = arr.shape + (1,)
        elif len(arr.shape) == 3:
            im.height, im.width, channels = arr.shape
        else:
            raise TypeError("Array must be two or three dimensional")

        # check type and channels
        if exp_channels != channels:
            raise TypeError(
                "Array has {} channels, {} requires {}".format(
                    channels, encoding, exp_channels
                )
            )
        if dtype_class != arr.dtype.type:
            raise TypeError(
                "Array is {}, {} requires {}".format(
                    arr.dtype.type, encoding, dtype_class
                )
            )

        # make the array contiguous in memory, as mostly required by the format
        contig = np.ascontiguousarray(arr)
        im.data = contig.tostring()
        im.step = contig.strides[0]
        im.is_bigendian = (
            arr.dtype.byteorder == ">"
            or arr.dtype.byteorder == "="
            and sys.byteorder == "big"
        )

        return im

    # async def runServer(self):

    def setUpParameters(self):
        """Not used."""
        pass


import asyncio

import rclpy
from std_msgs.msg import String

rclpy.init()
# node = rclpy.create_node("async_subscriber")
node = WebsocketBridge()


async def echo(websocket):
    async for message in websocket:
        node.publishImageBytes(message)
        await websocket.send(message)


async def serverMain():
    async with serve(echo, "localhost", 8765):
        await asyncio.get_running_loop().create_future()  # run forever


async def ros_loop():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1e-5)


def main():
    future = asyncio.wait([ros_loop(), serverMain()])
    asyncio.get_event_loop().run_until_complete(future)
