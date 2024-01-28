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

    # TODO: Is there a more elegant way to do this?
    # Currently we assume that this launch file is at the root of the workspace
    steward_root = path.dirname(path.realpath(__file__))
    environ['STEWARD_ROOT'] = steward_root
    xacro_name = 'steward.urdf.xacro'
    # mesh_path = path.join(steward_root, 'data', 'meshes', 'steward')

    doc = xacro.process_file(
        path.join(steward_root, 'config', xacro_name),
        mappings={
            'mesh_path': path.join(steward_root, 'data', 'meshes', 'steward')
        }
    )
    robot_desc = doc.toprettyxml(indent='   ')

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc
        }]
        # arguments=[path.join(steward_root, 'config', 'steward.urdf')]
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d' + f'{steward_root}/config/steward.rviz']
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    pose_to_transform_broadcaster = Node(
        package='state_estimation',
        executable='pose_to_transform_broadcaster'
    )

    return LaunchDescription([
        pose_to_transform_broadcaster,

        # MISC
        joint_state_publisher,
        urdf_publisher,
        # rviz,
    ])
