import array
import asyncio
import math
import sys
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
import struct  # for byte <-> float conversions

# ROS message types
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, PointField, NavSatFix
from std_msgs.msg import Header, Float32


class MessageType:
    IMAGE = 0x00
    SUBSCRIBE = 0x01
    TELEOP = 0x02
    POINTCLOUD = 0x03
    GNSS_FIX = 0x04


# mappings between PointField types and numpy types
type_mappings = [
    (PointField.INT8, np.dtype("int8")),
    (PointField.UINT8, np.dtype("uint8")),
    (PointField.INT16, np.dtype("int16")),
    (PointField.UINT16, np.dtype("uint16")),
    (PointField.INT32, np.dtype("int32")),
    (PointField.UINT32, np.dtype("uint32")),
    (PointField.FLOAT32, np.dtype("float32")),
    (PointField.FLOAT64, np.dtype("float64")),
]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)


def numpyToPointCloud(points, parent_frame, stamp):
    """Creates a point cloud message. Original:https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    Args:
        points: Nx3 array of xyz positions (m)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [
        PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate("xyz")
    ]

    header = Header(frame_id=parent_frame, stamp=stamp)

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),
        row_step=(itemsize * 3 * points.shape[0]),
        data=data,
    )


def dtype_to_fields(dtype):
    """Convert a numpy record datatype into a list of PointFields."""
    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = int(np.prod(shape))
            np_field_type = item_dtype
        else:
            pf.count = 1

        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields


def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    """Converts a numpy record array to a sensor_msgs.msg.PointCloud2."""
    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = sys.byteorder != "little"
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step * cloud_arr.shape[1]
    cloud_msg.is_dense = all(
        [np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names]
    )

    # The PointCloud2.data setter will create an array.array object for you if you don't
    # provide it one directly. This causes very slow performance because it iterates
    # over each byte in python.
    # Here we create an array.array object using a memoryview, limiting copying and
    # increasing performance.
    memory_view = memoryview(cloud_arr)
    if memory_view.nbytes > 0:
        array_bytes = memory_view.cast("B")
    else:
        # Casting raises a TypeError if the array has no elements
        array_bytes = b""
    as_array = array.array("B")
    as_array.frombytes(array_bytes)
    cloud_msg.data = as_array
    return cloud_msg


class WebsocketBridge(Node):
    def __init__(self):
        super().__init__("sim_bridge")

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

        self.depth_pcd_pub = self.create_publisher(
            PointCloud2, "/depth_pcd", 10
        )  # TODO: Alter topic to match ZED

        self.diagnostic_pub = self.create_publisher(DiagnosticStatus, "/diagnostics", 1)

        self.gnss_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/gnss/pose", 1
        )

        self.gnss_fix_pub = self.create_publisher(NavSatFix, "/gnss/fix", 1)
        self.gnss_yaw_pub = self.create_publisher(Float32, "/gnss/yaw", 1)

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmdVelCb, 10
        )

        # CONTROLLER_FREQ = 100  # Hz
        self.create_timer(0.1, self.publishHeartbeat)
        self.create_timer(0.1, self.sendWsTwist)
        self.teleop_connections: list[ServerConnection] = []

        self.connected_to_client = False

        self.create_timer(0.1, self.publishStatus)

    def publishStatus(self):

        # self.get_logger().info("PUBLISHING STATUS")

        if self.connected_to_client:
            level = DiagnosticStatus.OK
            msg = "Connected."

        else:
            level = DiagnosticStatus.ERROR
            msg = "Not connected."

        status_msg = DiagnosticStatus(level=level, name=self.get_name(), message=msg)

        self.diagnostic_pub.publish(status_msg)

    async def sendWsTwist(self):
        if len(self.teleop_connections) < 1:
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

        if throttle_byte > 255:
            throttle_byte = 255
            self.get_logger().warning(
                f"Throttle byte {throttle_byte} was too big. Capping to 256."
            )
        elif throttle_byte < 0:
            throttle_byte = 0
            self.get_logger().warning(
                f"Throttle byte {throttle_byte} was too small. Capping to 0."
            )
        if turn_byte > 255:
            turn_byte = 255
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

    def handlePointcloudBytes(self, message: bytes):

        def bytesToTuple(b):
            x_bytes = b[:2]
            x = int.from_bytes(x_bytes, sys.byteorder, signed=True)

            y_bytes = b[2:4]
            y = int.from_bytes(y_bytes, sys.byteorder, signed=True)

            z_bytes = b[4:]
            z = int.from_bytes(z_bytes, sys.byteorder, signed=True)

            # node.get_logger().info(f"{str(b)} -> {x}, {y}, {z}")

            return (x, y, z)

        point_idx = 0
        point_count = math.floor((len(message) - 1) / 6)

        # for byte in message:

        # node.get_logger().info(str(byte))

        points = []
        for i in range(point_count):
            point_bytes = message[(i * 6) + 1 : (i * 6) + 7]
            points.append(bytesToTuple(point_bytes))

        # node.get_logger().info(
        #     f"Received pcd with {len(message)} bytes ({point_count} points)"
        # )

        points = np.asarray(points) / 100  # Convert back to meters!
        # self.get_logger().info(
        #     f"Shape: {points.shape}, dtype: {points.dtype}, names: {points.dtype.names}"
        # )
        # msg = array_to_pointcloud2(
        #     points, stamp=self.get_clock().now().to_msg(), frame_id="base_link"
        # )  # TODO: Change this to camra frame

        msg = numpyToPointCloud(
            points, "base_link", stamp=self.get_clock().now().to_msg()
        )

        self.depth_pcd_pub.publish(msg)

    def handleGnssFix(self, message: bytes):

        lat = struct.unpack("f", message[1:5])[0]
        lon = struct.unpack("f", message[5:9])[0]
        alt = struct.unpack("f", message[9:13])[0]
        yaw = struct.unpack("f", message[13:17])[0]

        self.get_logger().info(f"{lat}, {lon}, {alt}, {yaw}")

        fix_msg = NavSatFix()
        fix_msg.header.frame_id = "earth"
        fix_msg.header.stamp = self.get_clock().now().to_msg()

        fix_msg.latitude = lat
        fix_msg.longitude = lon
        fix_msg.altitude = alt

        self.gnss_fix_pub.publish(fix_msg)

        yaw_msg = Float32(data=yaw)
        self.gnss_yaw_pub.publish(yaw_msg)


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

    node.connected_to_client = True
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

            elif message[0] == MessageType.POINTCLOUD:
                node.handlePointcloudBytes(message)
            elif message[0] == MessageType.GNSS_FIX:
                node.handleGnssFix(message)
            else:
                raise NotImplementedError(f"Received invalid Message Type {message[0]}")

            await connection.send(message)

    except ConnectionClosedError as e:
        node.get_logger().warning(
            f"Connection {connection.id} from {connection.remote_address[0]} closed unexpectedly."
        )

    finally:
        node.get_logger().info(f"KISS connection ended.")
        node.connected_to_client = False


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
