import asyncio
from websockets.asyncio.server import serve, Server, ServerConnection
from websockets.exceptions import ConnectionClosedError

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
from random import randbytes, randint

# ROS message types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage


class MessageType:
    IMAGE = 0x00
    SUBSCRIBE = 0x01
    TELEOP = 0x02


class WebsocketBridge(Node):
    def __init__(self):
        super().__init__("websocket_bridge")

        self.bridge = CvBridge()

        # Cached variables for teleop commands
        self.throttle = 0
        self.turn = 0

        self.setUpParameters()

        # TODO: Create proper QoS profile.
        self.image_pub = self.create_publisher(Image, "/rgb/image_rect_color", 10)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, "/rgb/image_rect_color_compressed", 10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmdVelCb, 10
        )

        # CONTROLLER_FREQ = 100  # Hz
        self.create_timer(0.1, self.publishHeartbeat)
        self.create_timer(0.1, self.sendWsTwist)
        self.teleop_connections: list[ServerConnection] = []

    async def sendWsTwist(self):
        if len(self.teleop_connections) < 1:
            print("no connections")
            return

        # message = [MessageType.TELEOP] + randbytes(2)
        # self.get_logger().info(message)

        # Remove inactive connections
        self.teleop_connections = [x for x in self.teleop_connections if x.state == 1]

        # message = [MessageType.TELEOP] + bytearray(randbytes(2))

        throttle_byte = int((self.throttle + 1) * 128 - 1)
        turn_byte = int((self.turn + 1) * 128 - 1)

        print(f"{self.throttle} -> {throttle_byte}")
        print(f"{self.turn} -> {turn_byte}")

        if throttle_byte > 256:
            throttle_byte = 256
            self.get_logger().warning(
                f"Throttle byte {throttle_byte} was too big. Capping to 256."
            )
        elif throttle_byte < 0:
            throttle_byte = 0
            self.get_logger().warning(
                f"Throttle byte {throttle_byte} was too small. Capping to 0."
            )
        if turn_byte > 256:
            turn_byte = 256
            self.get_logger().warning(
                f"Turn byte {turn_byte} was too big. Capping to 256."
            )
        elif turn_byte < 0:
            turn_byte = 0
            self.get_logger().warning(
                f"Turn byte {turn_byte} was too small. Capping to 0."
            )
        message = bytearray([MessageType.TELEOP, throttle_byte, turn_byte])
        # print(f"{self.tur}, {turn_byte}")

        # print(self.teleop_connections)

        for connection in self.teleop_connections:
            # self.get_logger().info(
            #     f"Sending teleop message to {connection.id} {connection.state}"
            # )
            await connection.send(message)

    def cmdVelCb(self, msg: Twist):
        """Send the Twist message to the Websocket server

        Args:
            msg (Twist): _description_
        """
        # self.get_logger().info("HELLO")
        self.throttle = msg.linear.x
        self.turn = msg.angular.z
        self.get_logger().info(f"Received throt {self.throttle}, turn {self.turn}")

    def publishHeartbeat(self):
        """This is where a "heartbeat" or more detailed diagnostics should be published."""
        # self.get_logger().info("I'm still alive!")
        pass

    async def handleConnection(self, websocket):
        async for message in websocket:
            await websocket.send(message)

    def publishImageBytes(self, img_str):

        # Start with compressed publisher
        compressed_msg = CompressedImage()

        # print(img_str)
        compressed_msg.data = img_str
        compressed_msg.format = "jpeg"
        # TODO: Header

        self.compressed_image_pub.publish(compressed_msg)

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


async def handleConnection(connection: ServerConnection):
    node.get_logger().info(
        f"Opened new connection to {connection.remote_address[0]} with ID {connection.id}"
    )
    try:
        async for message in connection:
            # Check the first byte, which is the message type
            # message.
            if type(message) != bytes:
                raise NotImplementedError

            message: bytes

            # Check the message type
            if message[0] == MessageType.IMAGE:
                node.publishImageBytes(message[1:])

            elif message[0] == MessageType.SUBSCRIBE:
                # while True:
                #     await websocket.send(randbytes(2))
                #     await asyncio.sleep(0.01)
                if message[1] == MessageType.TELEOP:
                    node.get_logger().info(
                        f"Creating TELEOP subscriber for connection {connection.id}"
                    )
                    node.teleop_connections.append(connection)
                else:
                    raise NotImplementedError

            else:
                raise NotImplementedError(f"Received invalid Message Type {message[0]}")

            # message.
            await connection.send(message)

    except ConnectionClosedError as e:
        node.get_logger().warning(
            f"Connection {connection.id} from {connection.remote_address[0]} closed unexpectedly."
        )


async def spinWebsocketServer():

    node.get_logger().info("Starting KISS Server")

    async with serve(handleConnection, "localhost", 8765):
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
