import asyncio
from websockets.asyncio.server import serve, Server, ServerConnection

from matplotlib import pyplot as plt
from matplotlib import image as mpimg
import io
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import numpy as np
from cv_bridge import CvBridge
from PIL import Image as PILImage
import cv2
from random import randbytes

# ROS message types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class MessageType:
    IMAGE = 0x00
    SUBSCRIBE = 0x01
    TELEOP = 0x02


class WebsocketBridge(Node):
    def __init__(self):
        super().__init__("websocket_bridge")

        self.bridge = CvBridge()

        self.setUpParameters()

        # TODO: Create proper QoS profile.
        self.image_pub = self.create_publisher(Image, "/rgb/image_rect_color", 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmdVelCb, 10
        )

        # CONTROLLER_FREQ = 100  # Hz
        self.create_timer(0.1, self.publishHeartbeat)
        self.create_timer(0.1, self.sendWsTwist)
        self.twist_connection: ServerConnection = None

    async def sendWsTwist(self):
        if self.twist_connection is None:
            return

        await self.twist_connection.send(randbytes(2))

    def cmdVelCb(self, msg: Twist):
        """Send the Twist message to the Websocket server

        Args:
            msg (Twist): _description_
        """
        pass

    def publishHeartbeat(self):
        """This is where a "heartbeat" or more detailed diagnostics should be published."""
        # self.get_logger().info("I'm still alive!")
        pass

    async def handleConnection(self, websocket):
        async for message in websocket:
            await websocket.send(message)

    def publishImageBytes(self, img_str):
        # print(img_str)
        # nparr = np.fromstring(img_str, np.uint8)
        # img_np = cv2.imdecode(nparr, cv2.IMREAD_COLOR)  # cv2.IMREAD_COLOR in OpenCV 3.1
        img_np = np.array(PILImage.open(io.BytesIO(img_str)))

        # bytes_arr = np.array(cv2.imdecode(io.BytesIO(bytes)))

        img_np = cv2.flip(img_np, 0)
        # img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)

        if img_np.shape[-1] == 4:
            msg = self.bridge.cv2_to_imgmsg(img_np, "rgba8")
        else:
            msg = self.bridge.cv2_to_imgmsg(img_np, "rgb8")

        # bytes_arr = mpimg.imread()
        # plt.imshow(bytes_arr)
        # self.get_logger().info(str(bytes_arr))
        self.image_pub.publish(msg)
        # plt.show()

    def setUpParameters(self):
        """Not used. Declare ROS params here."""
        pass


import asyncio

import rclpy
from std_msgs.msg import String

rclpy.init()
# node = rclpy.create_node("async_subscriber")
node = WebsocketBridge()


async def echo(websocket: ServerConnection):
    async for message in websocket:
        # Check the first byte, which is the message type
        # message.
        if type(message) != bytes:
            raise NotImplementedError

        message: bytes

        # Check the message type
        if message[0] == MessageType.IMAGE:
            print("Got image")
            node.publishImageBytes(message[1:])

        elif message[0] == MessageType.SUBSCRIBE:
            # while True:
            #     await websocket.send(randbytes(2))
            #     await asyncio.sleep(0.01)

            node.twist_connection = websocket

        # message.
        await websocket.send(message)


async def spinWebsocketServer():
    async with serve(echo, "localhost", 8765):
        await asyncio.get_running_loop().create_future()  # run forever


async def spinRos(freq=1e4):
    while rclpy.ok():
        # Manually spin the ROS node at a rate of 1e4 Hz.
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1 / freq)


def main():
    # Run both the ROS loop and the Websocket server loop
    future = asyncio.wait([spinRos(), spinWebsocketServer()])

    # Spin until both loops complete or are cancelled
    asyncio.get_event_loop().run_until_complete(future)
