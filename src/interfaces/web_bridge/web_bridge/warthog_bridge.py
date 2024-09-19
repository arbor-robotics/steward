import asyncio
from websockets.asyncio.server import serve, Server, ServerConnection
from websockets.exceptions import ConnectionClosedError

from websockets.asyncio.client import connect, ClientConnection
import websocket


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

from collections import deque

# ROS message types
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header

import json


class MessageType:
    IMAGE = 0x00
    SUBSCRIBE = 0x01
    TELEOP = 0x02


class WarthogBridge(Node):
    def __init__(self):
        super().__init__("websocket_bridge")

        self.bridge = CvBridge()

        self.ws = websocket.WebSocket()

        self.ws.connect("ws://192.168.131.1:9090", timeout=0.1)
        print("Connected!")

        # Cached variables for teleop commands
        self.throttle = 0
        self.turn = 0

        self.setUpParameters()

        self.outgoing_messages = deque()

        # TODO: Create proper QoS profile.
        self.image_pub = self.create_publisher(Image, "/rgb/image_rect_color", 10)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, "/rgb/image_rect_color_compressed", 10
        )

        # CONTROLLER_FREQ = 100  # Hz
        self.create_timer(1.0, self.publishHeartbeat)
        self.requestTopics()
        self.subscribeToDiagnostics()
        # self.create_timer(0.1, self.sendWsTwist)
        self.teleop_connections: list[ServerConnection] = []

        self.create_timer(0.1, self.checkMessages)

        self.warthog_diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics/warthog", 10
        )

    def republishDiagnostics(self, msgDictionary: dict):
        """
        Takes a JSON-formatted DiagnosticArray message, repackages it as a
        ROS2 DiagnosticArray message, and republishes it in ROS2.
        """
        headerDict = msgDictionary["header"]
        msg = DiagnosticArray()

        msg.header.stamp.sec = headerDict["stamp"]["secs"]
        msg.header.stamp.nanosec = headerDict["stamp"]["nsecs"]
        msg.header.frame_id = headerDict["frame_id"]

        for status in msgDictionary["status"]:
            status_msg = DiagnosticStatus()
            status_msg.level = bytes([status["level"]])
            status_msg.name = status["name"]
            status_msg.message = status["message"]
            status_msg.hardware_id = status["hardware_id"]

            for key_val in status["values"]:
                kv = KeyValue()
                kv.key = key_val["key"]
                kv.value = key_val["value"]

                status_msg.values.append(kv)

            msg.status.append(status_msg)

        self.warthog_diagnostic_pub.publish(msg)

    def checkMessages(self):
        # print("Checcking messages")
        try:
            msg = json.loads(self.ws.recv())

            if msg["op"] == "service_response":
                return

            elif msg["op"] == "publish":
                # We've received a message from a subscription
                if msg["topic"] == "/diagnostics_agg":
                    # Process diagnostics
                    self.republishDiagnostics(msg["msg"])

                else:
                    self.get_logger().warning(f"Unknown topic {msg['topic']}")
            # print("\n\nDIAGNOSTICS")
            # print(msg)

            # exit()
            # msg = msg["msg"]

            # status_list = msg["status"]

            # for status in status_list:
            #     print(status)

            # print(msg)
        except Exception as e:
            if str(e).find("Connection timed out") != -1:
                # self.get_logger().warning(f"No messages. {e}")
                return
            else:
                self.get_logger().error(f"{e}")

    def requestTopics(self):
        # self.outgoing_messages.append(
        #     json.dumps({"op": "call_service", "service": "/rosapi/topics"})
        # )

        if not self.ws.connected:
            self.get_logger().warning("Not connected to Warthog. Can't get topics.")
            return

        self.ws.send(json.dumps({"op": "call_service", "service": "/rosapi/topics"}))

    def subscribeToDiagnostics(self):
        if not self.ws.connected:
            self.get_logger().warning(
                "Not connected to Warthog. Can't sub to diagnostics."
            )
            return

        self.ws.send(json.dumps({"op": "subscribe", "topic": "/diagnostics_agg"}))

    def publishHeartbeat(self):
        """This is where a "heartbeat" or more detailed diagnostics should be published."""
        # self.get_logger().info("I'm still alive!")

        pass

    # async def handleConnection(self, websocket):
    #     async for message in websocket:
    #         await websocket.send(message)

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


rclpy.init()
node = WarthogBridge()


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


def main():
    rclpy.spin(node)

    node.ws.close()
    node.destroy_node()
    rclpy.shutdown()

    # Spin until both loops complete or are cancelled
    # asyncio.get_event_loop().run_until_complete(future)
