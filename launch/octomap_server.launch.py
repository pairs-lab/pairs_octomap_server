#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        IfElseSubstitution,
        EnvironmentVariable,
        LaunchConfiguration,
        PathJoinSubstitution,
        PythonExpression,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "pairs_octomap_server"

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ frame ids

    # #{ world_frame

    world_frame = LaunchConfiguration('world_frame')

    ld.add_action(DeclareLaunchArgument(
        'world_frame',
        default_value=[uav_name, "/fixed_origin"],
        description='The frame id of the world frame for the mapping.'
    ))

    # #} end of world_frame

    # #{ robot_frame

    robot_frame = LaunchConfiguration('robot_frame')

    ld.add_action(DeclareLaunchArgument(
        'robot_frame',
        default_value=[uav_name, "/fcu"],
        description='The frame id of the robots body.',
    ))

    # #} end of robot_frame

    # #} end of frame ids

    # #{ subscriber topics through arguments

    # id 0

    # #{ lidar_3d_0

    lidar_3d_0 = LaunchConfiguration('lidar_3d_0')

    ld.add_action(DeclareLaunchArgument(
        'lidar_3d_0',
        default_value='~/lidar_3d_0/points_in',
        description='Lidar 3D #0 points topic'
    ))

    # #} end of lidar_3d_0

    # #{ lidar_3d_0_free

    lidar_3d_0_free = LaunchConfiguration('lidar_3d_0_free')

    ld.add_action(DeclareLaunchArgument(
        'lidar_3d_0_free',
        default_value='~/lidar_3d_0/free_points_in',
        description='Lidar 3D #0 free points topic'
    ))

    # #} end of lidar_3d_0_free

    # #{ depth_camera_0

    depth_camera_0 = LaunchConfiguration('depth_camera_0')

    ld.add_action(DeclareLaunchArgument(
        'depth_camera_0',
        default_value='~/depth_camera_0/points_in',
        description='Depth camera #0 points topic'
    ))

    # #} end of depth_camera_0

    # #{ camera_info_0

    camera_info_0 = LaunchConfiguration('camera_info_0')

    ld.add_action(DeclareLaunchArgument(
        'camera_info_0',
        default_value='~/depth_camera/camera_info_in',
        description='Depth camera #0 info'
    ))

    # #} end of camera_info_0

    # #{ depth_camera_0_free

    depth_camera_0_free = LaunchConfiguration('depth_camera_0_free')

    ld.add_action(DeclareLaunchArgument(
        'depth_camera_0_free',
        default_value='~/depth_camera_0/free_points_in',
        description='Depth camera #0 free points topic'
    ))

    # #} end of depth_camera_0_free

    # id 1

    # #{ lidar_3d_1

    lidar_3d_1 = LaunchConfiguration('lidar_3d_1')

    ld.add_action(DeclareLaunchArgument(
        'lidar_3d_1',
        default_value='~/lidar_3d_1/points_in',
        description='Lidar 3D #0 points topic'
    ))

    # #} end of lidar_3d_1

    # #{ lidar_3d_1_free

    lidar_3d_1_free = LaunchConfiguration('lidar_3d_1_free')

    ld.add_action(DeclareLaunchArgument(
        'lidar_3d_1_free',
        default_value='~/lidar_3d_1/free_points_in',
        description='Lidar 3D #0 free points topic'
    ))

    # #} end of lidar_3d_1_free

    # #{ depth_camera_1

    depth_camera_1 = LaunchConfiguration('depth_camera_1')

    ld.add_action(DeclareLaunchArgument(
        'depth_camera_1',
        default_value='~/depth_camera_1/points_in',
        description='Depth camera #0 points topic'
    ))

    # #} end of depth_camera_1

    # #{ camera_info_1

    camera_info_1 = LaunchConfiguration('camera_info_1')

    ld.add_action(DeclareLaunchArgument(
        'camera_info_1',
        default_value='~/depth_camera/camera_info_in',
        description='Depth camera #0 info'
    ))

    # #} end of camera_info_1

    # #{ depth_camera_free_1

    depth_camera_1_free = LaunchConfiguration('depth_camera_1_free')

    ld.add_action(DeclareLaunchArgument(
        'depth_camera_1_free',
        default_value='~/depth_camera_1/free_points_in',
        description='Depth camera #0 free points topic'
    ))

    # #} end of depth_camera_1_free

    # #} end of topics in

    # #{ octomap server node

    octomap_server_node = ComposableNode(
        package=pkg_name,
        plugin='pairs_octomap_server::OctomapServer',
        namespace=uav_name,
        name='octomap_server',
        parameters=[
            {"uav_name": uav_name},
            {"use_sim_time": use_sim_time},
            {"world_frame_id": world_frame},
            {"robot_frame_id": robot_frame},
            {"map_path": "/tmp"},
            {"public_config": this_pkg_path+'/config/default.yaml'},
            {'custom_config': custom_config},
        ],
        remappings=[
            # sensors #0
            ("~/lidar_3d_0/points_in", lidar_3d_0),
            ("~/lidar_3d_0/free_points_in", lidar_3d_0_free),
            ("~/depth_camera_0/points_in", depth_camera_0),
            ("~/depth_camera_0/points_free_in", depth_camera_0_free),
            ("~/depth_camera_0/camera_info_in", camera_info_0),
            # sensors #1
            ("~/lidar_3d_1/points_in", lidar_3d_1),
            ("~/lidar_3d_1/free_points_in", lidar_3d_1_free),
            ("~/depth_camera_1/points_in", depth_camera_1),
            ("~/depth_camera_1/points_free_in", depth_camera_1_free),
            ("~/depth_camera_1/camera_info_in", camera_info_1),
            # Other remappings
            ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),
            ("~/height_in", "estimation_manager/height_agl"),
            ("~/clear_box_in", "uav_pose_estimator/clear_box"),
            # topics out
            ("~/octomap_global_full_out", "~/octomap_global_full"),
            ("~/octomap_global_binary_out", "~/octomap_global_binary"),
            ("~/octomap_local_full_out", "~/octomap_local_full"),
            ("~/octomap_local_binary_out", "~/octomap_local_binary"),
            # services in
            ("~/reset_map_in", "~/reset_map"),
            ("~/save_map_in", "~/save_map"),
            ("~/load_map_in", "~/load_map"),
        ],
    )

    # #} end of waypoint flier node

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ load into container

    load_into_existing = LoadComposableNodes(
        target_container= container_name,
        composable_node_descriptions = [octomap_server_node],
        condition = UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of load into container

    # #{ standalone container

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        name= 'octomap_server_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[octomap_server_node],
        condition = IfCondition(standalone)
    ))

    # #} end of standalone container

    return ld
