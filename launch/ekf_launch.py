from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_se_odom",
                output="screen",
                parameters=["/home/main/steward/src/ekf/ekf_config.yaml"],
            ),
            # IMU static transform
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "imu_link"],
            ),
            # Odom static transform
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_base_link_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "base_link"],
            ),
        ]
    )
