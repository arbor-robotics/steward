import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import serial

# ROS interfaces
from std_msgs.msg import Header
from std_msgs.msg import Empty


class SerialBridge(Node):
    """
    Subscribes to /diagnostics (DiagnosticArray)
    Publishes to /diagnostic_agg (DiagnosticArray)
    """

    def __init__(self):
        super().__init__("serial_bridge")

        self.trigger_sub = self.create_subscription(
            Empty, "/planning/plant_seedling", self.triggerCb, 10
        )

    def triggerCb(self, msg: Empty):
        self.get_logger().info("SENDING SERIAL COMMAND")
        with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as ser:
            ser.write(b"Plant")


def main(args=None):
    rclpy.init(args=args)

    node = SerialBridge()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
