from os import name, path, environ, getcwd

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
    mesh_path = path.join(steward_root, "data", "meshes", "steward")
    map_dir = path.join(steward_root, "data", "maps", MAP_NAME)
    param_dir = path.join(steward_root, "config")

    doc = xacro.process_file(
        path.join(steward_root, "config", xacro_name), mappings={"mesh_path": mesh_path}
    )
    robot_desc = doc.toprettyxml(indent="   ")

    urdf_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    rviz = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        # arguments=['-d' + f'{steward_root}/config/steward.rviz']
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
        parameters=[{"minimum_spacing": 4.0, "plan_resolution": 0.2}],
    )
    route_planner = Node(package="route_planning", executable="route_planner")
    mvp_controller = Node(package="motion_control", executable="mvp_controller")
    camera_processor = Node(
        package="camera_processing", executable="arborsim_cam_processor"
    )

    map_loader = Node(
        package="mapping",
        executable="map_loader",
        parameters=[
            {
                "map_dir": map_dir,
                "resolution": 0.2,  # m/pixel
                "origin": [0, 0, 0.0],
                # 'origin': [-661.07, -423.11, -43.73]
            }
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                path.join(get_package_share_directory("nav2_bringup"), "launch"),
                "/bringup_launch.py",
            ]
        ),
        launch_arguments={
            "map": path.join(map_dir, "map.yaml"),
            # "use_sim_time": use_sim_time,
            "params_file": path.join(param_dir, "nav2.param.yaml"),
        }.items(),
    )

    return LaunchDescription(
        [
            camera_processor,
            forest_planner,
            joint_state_publisher,
            map_loader,
            # mvp_controller,
            nav2_bringup,
            pose_to_transform_broadcaster,
            route_planner,
            rviz,
            urdf_publisher,
        ]
    )
