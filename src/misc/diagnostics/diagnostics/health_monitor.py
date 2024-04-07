import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS interfaces
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus as Status
from std_msgs.msg import Header


class HealthMonitor(Node):
    """
    Subscribes to /diagnostics (DiagnosticArray)
    Publishes to /diagnostic_agg (DiagnosticArray)
    """

    def __init__(self):
        super().__init__("health_monitor")

        self.declareParams()

        self.create_subscription(DiagnosticArray, "/diagnostics", self.diagnosticCb, 10)

        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics_aggr", 10
        )

        self.required_ok_nodes = ["route_planner"]

        timer_freq = self.get_parameter("publish_freq").value
        self.diagnostic_pub_timer = self.create_timer(
            1 / timer_freq, self.publishStatus
        )

        self.statuses = {}

    def publishStatus(self):
        status_array = DiagnosticArray()

        for status in self.statuses.values():
            print(isinstance(status, Status))
            status_array.status.append(status)

        # status_array.header = self.getHeader()

        print(status_array)
        self.diagnostic_pub.publish(status_array)

    def diagnosticCb(self, msg: DiagnosticArray):
        # Our status is OK unless a node tells us otherwise
        global_status = Status()
        global_status.name = "global"

        for status in msg.status:
            status: Status

            self.statuses[status.name] = status

            if status.name in self.required_ok_nodes and status.level != Status.OK:
                global_status.level = Status.ERROR
                global_status.message += status.message
                self.get_logger().error(
                    f"Global status is now ERROR due to {status.name}: {status.message}"
                )

        self.statuses["global"] = global_status

    def declareParams(self) -> None:
        descr = ParameterDescriptor()
        descr.description = "Rate at which the aggregated array, includding the system-wide status, is published. Hz."
        descr.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("publish_freq", descriptor=descr, value=1.0)

        descr = ParameterDescriptor()
        descr.description = "Names of nodes that must have status of OK"
        descr.type = ParameterType.PARAMETER_STRING_ARRAY
        self.declare_parameter("required_ok_nodes", descriptor=descr)

    def getHeader(self) -> Header:
        """Forms a message Header.

        Returns:
            Header: The stamped and framed Header.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        return header


def main(args=None):
    rclpy.init(args=args)

    node = HealthMonitor()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
