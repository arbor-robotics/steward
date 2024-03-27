from os import name, path, environ, getcwd

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

import xacro


def generate_launch_description():

    MAP_NAME = "demoland"

    # TODO: Is there a more elegant way to do this?
    # Currently we assume that this launch file is at the root of the workspace
    steward_root = path.dirname(path.realpath(__file__))
    environ["STEWARD_ROOT"] = steward_root
    xacro_name = "steward.urdf.xacro"
    pkg_share = FindPackageShare(package="steward_description").find(
        "steward_description"
    )
    mesh_path = path.join(pkg_share, "meshes")
    map_dir = path.join(steward_root, "data", "maps", MAP_NAME)
    param_dir = path.join(steward_root, "config")

    doc = xacro.process_file(
        path.join(pkg_share, xacro_name), mappings={"mesh_path": mesh_path}
    )
    robot_desc = doc.toprettyxml(indent="   ")

    urdf_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher", executable="joint_state_publisher"
    )

    return LaunchDescription(
        [
            joint_state_publisher,
            urdf_publisher,
        ]
    )
