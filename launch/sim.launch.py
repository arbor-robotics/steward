from os import name, path, environ, getcwd

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

import xacro


def generate_launch_description():

    # TODO: Is there a more elegant way to do this?
    # Currently we assume that this launch file is at the root of the workspace
    steward_root = path.dirname(path.realpath(__file__))
    environ["STEWARD_ROOT"] = steward_root
    xacro_name = "steward.urdf.xacro"
    mesh_path = path.join(steward_root, "data", "meshes", "steward")
    map_directory = path.join(steward_root, "data", "maps", "schenley")

    doc = xacro.process_file(
        path.join(steward_root, "../config", xacro_name),
        mappings={"mesh_path": mesh_path},
    )
    robot_desc = doc.toprettyxml(indent="   ")

    urdf_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    heightmap_publisher = Node(
        package="mapping",
        executable="heightmap_publisher",
        parameters=[
            {
                "map_dir": map_directory,
                "resolution": 1.0,  # m/pixel
                "origin": [-661.07, -423.11, 0.0],
                # 'origin': [-661.07, -423.11, -43.73]
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher", executable="joint_state_publisher"
    )

    pose_to_transform_broadcaster = Node(
        package="state_estimation", executable="pose_to_transform_broadcaster"
    )

    forest_planner = Node(
        package="forest_planning",
        executable="forest_planner",
        parameters=[{"minimum_spacing": 4.0}],
    )
    route_planner = Node(package="route_planning", executable="route_planner")
    mvp_controller = Node(package="motion_control", executable="mvp_controller")

    gnss = Node(package="gnss_interface", executable="interface")
    rosbridge_server = Node(
        package="rosbridge_server", executable="rosbridge_websocket"
    )
    warthog_bridge = Node(package="web_bridge", executable="warthog_bridge")
    sim_bridge = Node(package="web_bridge", executable="sim_bridge")
    health_monitor = Node(package="health", executable="monitor")
    occ_grid = Node(package="costmaps", executable="occupancy_grid_node")
    cost_maps = Node(package="costmaps", executable="cost_map_node")
    behavior_fsm = Node(package="behavior", executable="fsm")
    plan_manager = Node(package="behavior", executable="plan_manager")

    return LaunchDescription(
        [
            # INTERFACES
            # gnss,
            rosbridge_server,
            # warthog_bridge,
            sim_bridge,
            # PERCEPTION
            # PLANNING
            plan_manager,
            occ_grid,
            cost_maps,
            # behavior_fsm,
            # trajectory_planner,
            # CONTROL
            # SAFETY
            health_monitor,
            # MISC.
            joint_state_publisher,
            urdf_publisher,
        ]
    )
