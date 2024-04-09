import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from behavior.behavior_fsm import StewardFSM

# ROS interfaces
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus as Status
from std_msgs.msg import Header
from steward_msgs.msg import State as StateMsg
from steward_msgs.srv import RequestTransition, GetGlobalHealth


class State:
    PAUSED = 0
    DRIVING = 1
    PLANTING = 2


class Transition:
    UNPAUSE = 0
    PAUSE = 1


STATE_TO_STRING = {0: "PAUSED", 1: "DRIVING", 2: "PLANTING"}
STRING_TO_STATE = {"PAUSED": 0, "DRIVING": 1, "PLANTING": 2}


class Health:
    # https://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
    OK = 0
    WARN = 1
    ERROR = 2
    # Staleness is not valid for global health


class BehaviorFSM(Node):
    """
    Subscribes to /diagnostics (DiagnosticArray)
    Publishes to /diagnostic_agg (DiagnosticArray)
    """

    def __init__(self):
        super().__init__("behavior_fsm")

        self.declareParams()

        self.current_state_pub = self.create_publisher(
            StateMsg, "/behavior/current_state", 10
        )

        publish_freq = self.get_parameter("publish_freq").value
        self.create_timer(1 / publish_freq, self.publishCurrentState)

        self.create_service(
            RequestTransition, "/behavior/request_state", self.transitionRequestCb
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics_aggr", self.diagnosticCb, 10
        )

        self.fsm = StewardFSM()

        self.current_health = Health.ERROR

    def publishCurrentState(self):
        print(self.fsm.state)
        state_msg = StateMsg(value=STRING_TO_STATE[self.fsm.state])
        self.current_state_pub.publish(state_msg)

    def transitionRequestCb(
        self, request: RequestTransition.Request, response: RequestTransition.Response
    ) -> RequestTransition.Response:

        print(request.transition)

        try:
            if request.transition == Transition.UNPAUSE:
                self.fsm.start()
                response.success = True
            elif request.transition == Transition.PAUSE:
                self.fsm.pause()
                response.success = True
            else:
                self.get_logger().warning(
                    f"Unsupported transition '{request.transition}'. Ignoring."
                )
                response.success = False
                response.description = (
                    f"Unsupported transition '{request.transition}'. Ignoring."
                )
        except Exception as e:
            self.get_logger().error(f"Unsupported transition: {e}")
            response.success = False
            response.description = str(e)

        # to_state = STATE_TO_STRING[request.requested_state.value]

        # self.fsm.trigger(to_state)

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
        for status in msg.status:
            status: Status

            # Convert from byte to int
            level = int.from_bytes(status.level, byteorder="big")

            if status.name != "global":
                continue

            if level == Health.OK:
                self.current_health = Health.OK
            elif level == Health.WARN:
                self.current_health = Health.WARN
            elif level == Health.ERROR:
                self.current_health = Health.ERROR
            else:
                self.get_logger().error(f"Received invalid global health ID: {level}")

        if self.current_health == Health.ERROR:
            self.get_logger().error(f"Pausing due to ERROR state.")
            self.current_state = State.PAUSED

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