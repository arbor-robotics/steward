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

default_config_common = path.join(
    get_package_share_directory("zed_wrapper"), "config", "common.yaml"
)


def zed_launch_setup(context, *args, **kwargs):
    wrapper_dir = get_package_share_directory("zed_wrapper")

    # Launch configuration variables
    svo_path = LaunchConfiguration("svo_path")

    # use_sim_time = LaunchConfiguration("use_sim_time")
    # sim_mode = LaunchConfiguration("sim_mode")
    # sim_address = LaunchConfiguration("sim_address")
    # sim_port = LaunchConfiguration("sim_port")

    stream_address = LaunchConfiguration("stream_address")
    stream_port = LaunchConfiguration("stream_port")

    camera_name = LaunchConfiguration("camera_name")
    camera_model = LaunchConfiguration("camera_model")

    # node_name = LaunchConfiguration("node_name")
    node_name = "zed"

    config_common_path = LaunchConfiguration("config_path")

    serial_number = LaunchConfiguration("serial_number")

    # publish_urdf = LaunchConfiguration("publish_urdf")
    # publish_tf = LaunchConfiguration("publish_tf")
    # publish_map_tf = LaunchConfiguration("publish_map_tf")
    # publish_imu_tf = LaunchConfiguration("publish_imu_tf")
    xacro_path = LaunchConfiguration("xacro_path")

    default_xacro_path = path.join(
        get_package_share_directory("zed_wrapper"), "urdf", "zed_descr.urdf.xacro"
    )

    custom_baseline = LaunchConfiguration("custom_baseline")

    ros_params_override_path = LaunchConfiguration("ros_params_override_path")

    enable_gnss = LaunchConfiguration("enable_gnss")
    gnss_antenna_offset = LaunchConfiguration("gnss_antenna_offset")

    # camera_name_val = camera_name.perform(context)
    # camera_model_val = camera_model.perform(context)
    # enable_gnss_val = enable_gnss.perform(context)
    # gnss_coords = parse_array_param(gnss_antenna_offset.perform(context))
    # custom_baseline_val = custom_baseline.perform(context)

    # if camera_name_val == "":
    #     camera_name_val = "zed"

    # if camera_model_val == "virtual" and float(custom_baseline_val) <= 0:
    #     return [
    #         LogInfo(
    #             msg="Please set a positive value for the 'custom_baseline' argument when using a 'virtual' Stereo Camera with two ZED X One devices."
    #         ),
    #     ]

    config_camera_path = path.join(
        get_package_share_directory("zed_wrapper"), "config", "zed" + ".yaml"
    )

    # Xacro command with options
    xacro_command = []
    xacro_command.append("xacro")
    xacro_command.append(" ")
    xacro_command.append(default_xacro_path)
    xacro_command.append(" ")
    xacro_command.append("camera_name:=")
    xacro_command.append("camera_front")
    xacro_command.append(" ")
    xacro_command.append("camera_model:=")
    xacro_command.append("zed")
    xacro_command.append(" ")
    xacro_command.append("custom_baseline:=")
    xacro_command.append("0.0")
    # if enable_gnss_val == "true":
    #     xacro_command.append(" ")
    #     xacro_command.append("enable_gnss:=true")
    #     xacro_command.append(" ")
    #     if len(gnss_coords) == 3:
    #         xacro_command.append("gnss_x:=")
    #         xacro_command.append(gnss_coords[0])
    #         xacro_command.append(" ")
    #         xacro_command.append("gnss_y:=")
    #         xacro_command.append(gnss_coords[1])
    #         xacro_command.append(" ")
    #         xacro_command.append("gnss_z:=")
    #         xacro_command.append(gnss_coords[2])
    #         xacro_command.append(" ")

    # Robot State Publisher node

    publish_urdf = True
    publish_tf = True
    rsp_node = Node(
        package="robot_state_publisher",
        namespace="zed",
        executable="robot_state_publisher",
        name="zed_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(xacro_command)}],
    )

    node_parameters = [
        # YAML files
        config_common_path,  # Common parameters
        config_camera_path,  # Camera related parameters
        # Overriding
        {
            "use_sim_time": False,
            # "simulation.sim_enabled": sim_mode,
            # "simulation.sim_address": sim_address,
            # "simulation.sim_port": sim_port,
            # "stream.stream_address": stream_address,
            # "stream.stream_port": stream_port,
            "general.camera_name": "camera_front",
            "general.camera_model": "zed",
            "general.grab_resolution": "VGA",
            # "svo.svo_path": svo_path,
            # "general.serial_number": serial_number,
            "pos_tracking.publish_tf": publish_tf,
            "pos_tracking.publish_map_tf": False,
            "sensors.publish_imu_tf": True,
            "pos_tracking.pos_tracking_enabled": False,
            "debug.sdk_verbose": 0,
            "debug.debug_common": False,
            # "gnss_fusion.gnss_fusion_enabled": enable_gnss,
        },
    ]

    # if ros_params_override_path.perform(context) != "":
    #     node_parameters.append(ros_params_override_path)

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package="zed_wrapper",
        namespace="",
        executable="zed_wrapper",
        name=node_name,
        # output="log",
        # prefix=['xterm -e valgrind --tools=callgrind'],
        # prefix=['xterm -e gdb -ex run --args'],
        # prefix=['gdbserver localhost:3000'],
        parameters=node_parameters,
        arguments=["--ros-args", "--log-level", "WARN"],
        remappings=[("/diagnostics", "/diagnostics/zed")],
    )

    return [rsp_node, zed_wrapper_node]


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

    health_monitor = Node(package="health", executable="monitor")
    occ_grid = Node(package="costmaps", executable="occupancy_grid_node")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=TextSubstitution(text=default_config_common),
                description="Path to the YAML configuration file for the camera.",
            ),
            # INTERFACES
            OpaqueFunction(function=zed_launch_setup),  # camera
            gnss,
            rosbridge_server,
            # warthog_bridge,
            # PERCEPTION
            occ_grid,
            # PLANNING
            # forest_planner,
            # route_planner,
            # heightmap_publisher,
            # CONTROL
            # mvp_controller,
            # SAFETY
            health_monitor,
            # MISC.
            joint_state_publisher,
            pose_to_transform_broadcaster,
            urdf_publisher,
        ]
    )
