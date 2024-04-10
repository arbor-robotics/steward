import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from time import sleep
from transitions import Machine, State as TransitionsState
from transitions.extensions import GraphMachine

# ROS interfaces
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus as Status
from std_msgs.msg import Header, Empty
from std_srvs.srv import Trigger
from steward_msgs.msg import State as StateMsg
from steward_msgs.srv import RequestTransition, GetGlobalHealth


class State:
    PAUSED = 0
    DRIVING = 1
    PLANTING = 2


class Transition:
    PLAY = 0
    PAUSE = 1


STATE_TO_STRING = {0: "PAUSED", 1: "DRIVING", 2: "PLANTING"}
STRING_TO_STATE = {"PAUSED": 0, "DRIVING": 1, "PLANTING": 2}
TRANSITION_TO_STRING = {0: "PLAY", 1: "PAUSE"}


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

        self.initMachine()

        self.declareParams()

        # PUBLISHERS
        self.current_state_pub = self.create_publisher(
            StateMsg, "/behavior/current_state", 10
        )
        # SUBSCRIBERS
        self.create_subscription(
            DiagnosticArray, "/diagnostics_aggr", self.diagnosticCb, 10
        )
        self.create_subscription(
            Empty, "/events/waypoint_reached", self.waypointReachedCb, 10
        )

        # SERVICES
        publish_freq = self.get_parameter("publish_freq").value
        self.create_timer(1 / publish_freq, self.publishCurrentState)
        self.create_service(
            RequestTransition, "/behavior/request_transition", self.transitionRequestCb
        )

        # CLIENTS
        self.go_to_waypoint_pub = self.create_publisher(
            Empty, "/planning/go_to_waypoint", 10
        )

        self.current_health = Health.ERROR

    def initMachine(self):
        """
        Create a Finite State Machine object using `transitions`
        https://github.com/pytransitions/transitions
        """
        self.machine = GraphMachine(
            model=self,
            states=[
                TransitionsState("PAUSED"),
                TransitionsState("DRIVING", on_enter="onEnterDriving"),
                TransitionsState("PLANTING", on_enter="onEnterPlanting"),
            ],
            initial="PAUSED",
        )

        self.machine.add_transition(
            trigger="start", source="PAUSED", dest="DRIVING", conditions=["is_healthy"]
        )
        self.machine.add_transition(
            "onGoalReached", "DRIVING", "PLANTING", conditions=["is_healthy"]
        )
        self.machine.add_transition(
            "onTreePlanted", "PLANTING", "DRIVING", conditions=["is_healthy"]
        )
        self.machine.add_transition("onError", "*", "PAUSED")
        self.machine.add_transition("pause", "*", "PAUSED")

        self.get_graph().draw("my_state_diagram.png", prog="dot")

    @property
    def is_healthy(self):
        return self.current_health == Health.OK

    def onEnterDriving(self):
        self.go_to_waypoint_pub.publish(Empty())

    def onEnterPlanting(self):
        print("Planting a tree")
        sleep(1)
        self.trigger("onTreePlanted")
        pass

    def waypointReachedCb(self, msg: Empty):
        self.trigger("onGoalReached")
        print("Waypoint reached.")

    def publishCurrentState(self):
        print(self.state)
        state_msg = StateMsg(value=STRING_TO_STATE[self.state])
        self.current_state_pub.publish(state_msg)

    def transitionRequestCb(
        self, request: RequestTransition.Request, response: RequestTransition.Response
    ) -> RequestTransition.Response:

        print(f"Requested: {TRANSITION_TO_STRING[request.transition]}")

        try:
            if request.transition == Transition.PLAY:
                response.success = self.trigger("start")

                print(f"IS HEALTH? {self.is_healthy}")
                if response.success:
                    response.description = f"State is now {self.state}"
                else:
                    response.description = f"Current health: {self.current_health}"
            elif request.transition == Transition.PAUSE:
                response.success = self.trigger("pause")
                if response.success:
                    response.description = f"State is now {self.state}"
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
        """Extract the gobal status level from the aggregated diagnostic array.

        Args:
            msg (DiagnosticArray): The aggregated global diagnostic array.
        """
        for status in msg.status:
            status: Status

            # Convert from byte to int
            level = int.from_bytes(status.level, byteorder="big")

            if status.name != "global":
                continue

            if level == Health.OK:
                print("OK")
                self.current_health = Health.OK
            elif level == Health.WARN:
                print("WARN")
                self.current_health = Health.WARN
            elif level == Health.ERROR:
                print("ERROR")
                if self.current_health != Health.ERROR:
                    # We've just transitioned to an error state.
                    # Inform the FSM.
                    self.trigger("onError")
                self.current_health = Health.ERROR
            else:
                self.get_logger().error(f"Received invalid global health ID: {level}")

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
