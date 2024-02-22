import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster


# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html


class PoseToTransformBroadcaster(Node):
    def __init__(self):
        super().__init__("pose_to_transform_broadcaster")

        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # TODO: Parameterize topic name
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/gnss/pose", self.poseCb, sensor_qos_profile
        )

    def poseCb(self, msg: PoseWithCovarianceStamped):
        t = TransformStamped()

        t.header = msg.header
        t.child_frame_id = "base_link"  # TODO: Change to 'gnss'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    broadcaster = PoseToTransformBroadcaster()

    rclpy.spin(broadcaster)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
