import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS interfaces
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus as Status
from std_msgs.msg import Header
from steward_msgs.msg import State as StateMsg
from steward_msgs.srv import RequestState, GetGlobalHealth

State = {0: "PAUSED", 1: "DRIVING", 2: "PLANTING"}


class BehaviorFSM(Node):
    """
    Subscribes to /diagnostics (DiagnosticArray)
    Publishes to /diagnostic_agg (DiagnosticArray)
    """

    def __init__(self):
        super().__init__("behavior_fsm")

        self.declareParams()

        self.create_publisher(StateMsg, "/behavior/current_state", 10)
        self.create_service(
            RequestState, "/behavior/request_state", self.stateRequestCb
        )
        self.get_global_health_client = self.create_client(
            GetGlobalHealth, "/diagnostics/get_global_health"
        )

        self.req = GetGlobalHealth.Request()
        print("Calling service!")
        self.future = self.get_global_health_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result())

    def stateRequestCb(
        self, request: RequestState.Request, response: RequestState.Response
    ) -> RequestState.Response:
        self.get_logger().info(
            f"Got request to change state to {State[request.requested_state]}"
        )
        return response

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

    node = BehaviorFSM()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
