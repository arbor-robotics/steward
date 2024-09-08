import asyncio
from websockets.asyncio.server import serve, Server, ServerConnection

from matplotlib import pyplot as plt
from matplotlib import image as mpimg
import io
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class WebsocketBridge(Node):
    def __init__(self):
        super().__init__("websocket_bridge")

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
        self.get_logger().info("I'm still alive!")

    async def handleConnection(self, websocket):
        async for message in websocket:
            await websocket.send(message)

    # async def runServer(self):

    def setUpParameters(self):
        """Not used."""
        pass


# def main(args=None):
#     rclpy.init(args=args)

#     node = WebsocketBridge()

#     async def runServer():
#         async with serve(node.handleConnection, "localhost", 8765):
#             await asyncio.get_running_loop().create_future()  # run forever

#     async def spinNode():
#         rclpy.spin_once(node)

#     async def run():
#         return await asyncio.gather(runServer(), spinNode())

#     rclpy.spin_once(node)

#     rclpy.get_global_executor().create_task(runServer)

#     # asyncio.run(run())

#     rclpy.spin(node)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     node.destroy_node()
#     rclpy.shutdown()


import asyncio

import rclpy
from std_msgs.msg import String

rclpy.init()
# node = rclpy.create_node("async_subscriber")
node = WebsocketBridge()


async def echo(websocket):
    async for message in websocket:
        await websocket.send(message)


async def serverMain():
    async with serve(echo, "localhost", 8765):
        await asyncio.get_running_loop().create_future()  # run forever


async def start():
    print("Node started.")

    async def msg_callback(msg):
        print(f"Message received: {msg}")

    node.create_subscription(String, "/test", msg_callback, 10)
    print("Listening to topic /test...")


async def ros_loop():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1e-4)


def main():
    future = asyncio.wait([ros_loop(), start(), serverMain()])
    asyncio.get_event_loop().run_until_complete(future)
