import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def namespaced_frame(robot_namespace, frame_name):
    if robot_namespace:
        return f"{robot_namespace}/{frame_name}"
    return frame_name


def load_node_parameters(config_path, node_name):
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    return config.get(node_name, {}).get("ros__parameters", {})


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    aruco_share = get_package_share_directory("cirtesu_tank_aruco_localization")
    aruco_config_path = os.path.join(aruco_share, "config", "aruco_map.yaml")
    aruco_params = load_node_parameters(aruco_config_path, "aruco_map_localization")

    world_frame = LaunchConfiguration("world_frame").perform(context)
    if not world_frame:
        world_frame = namespaced_frame(robot_namespace, "map")

    aruco_overrides = {
        "world_frame": world_frame,
        "base_frame": namespaced_frame(robot_namespace, "base_link"),
        "camera_frame": namespaced_frame(robot_namespace, "camera_down/optical_frame"),
        "aruco_topic": "down_camera/aruco_detections",
        "marker_topic": "aruco/markers",
        "pose_topic": "sensors/aruco/pose_enu",
    }

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_cirtesu_tank",
            output="screen",
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "3.1416",
                "--frame-id",
                "world_ned",
                "--child-frame-id",
                "cirtesu_tank",
            ],
        ),
        Node(
            package="cirtesu_tank_aruco_localization",
            executable="aruco_map_localization_node",
            name="aruco_map_localization",
            output="screen",
            parameters=[aruco_params, aruco_overrides],
        ),
    ]


def generate_launch_description():
    sura_localization_share = get_package_share_directory("sura_localization")
    auv_localization_launch = os.path.join(
        sura_localization_share,
        "launch",
        "auv_localization.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value=""),
            DeclareLaunchArgument("environment", default_value="real"),
            DeclareLaunchArgument("config_package", default_value="sura_localization"),
            DeclareLaunchArgument("config_file", default_value="config/auv_localization.yaml"),
            DeclareLaunchArgument("map_frame", default_value=""),
            DeclareLaunchArgument("odom_frame", default_value=""),
            DeclareLaunchArgument("base_link_frame", default_value=""),
            DeclareLaunchArgument("world_frame", default_value=""),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("use_navsat", default_value="true"),
            DeclareLaunchArgument("wait_for_datum", default_value="true"),
            DeclareLaunchArgument("datum_latitude"),
            DeclareLaunchArgument("datum_longitude"),
            DeclareLaunchArgument("datum_heading"),
            DeclareLaunchArgument("convert_imu_ned_to_enu", default_value="true"),
            DeclareLaunchArgument("imu_ned_topic", default_value="sensors/imu"),
            DeclareLaunchArgument("imu_enu_topic", default_value="sensors/imu_enu"),
            DeclareLaunchArgument("imu_enu_frame", default_value=""),
            DeclareLaunchArgument("convert_pressure_to_pose", default_value="true"),
            DeclareLaunchArgument("pressure_topic", default_value="sensors/pressure"),
            DeclareLaunchArgument("pressure_pose_topic", default_value="sensors/pressure/pose"),
            DeclareLaunchArgument("output_odom_topic", default_value=""),
            DeclareLaunchArgument("output_ned_odom_topic", default_value=""),
            DeclareLaunchArgument("ned_world_frame", default_value="world_ned"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(auv_localization_launch),
                launch_arguments=[
                    ("robot_namespace", LaunchConfiguration("robot_namespace")),
                    ("environment", LaunchConfiguration("environment")),
                    ("config_package", LaunchConfiguration("config_package")),
                    ("config_file", LaunchConfiguration("config_file")),
                    ("map_frame", LaunchConfiguration("map_frame")),
                    ("odom_frame", LaunchConfiguration("odom_frame")),
                    ("base_link_frame", LaunchConfiguration("base_link_frame")),
                    ("world_frame", LaunchConfiguration("world_frame")),
                    ("publish_tf", LaunchConfiguration("publish_tf")),
                    ("use_navsat", LaunchConfiguration("use_navsat")),
                    ("wait_for_datum", LaunchConfiguration("wait_for_datum")),
                    ("datum_latitude", LaunchConfiguration("datum_latitude")),
                    ("datum_longitude", LaunchConfiguration("datum_longitude")),
                    ("datum_heading", LaunchConfiguration("datum_heading")),
                    ("convert_imu_ned_to_enu", LaunchConfiguration("convert_imu_ned_to_enu")),
                    ("imu_ned_topic", LaunchConfiguration("imu_ned_topic")),
                    ("imu_enu_topic", LaunchConfiguration("imu_enu_topic")),
                    ("imu_enu_frame", LaunchConfiguration("imu_enu_frame")),
                    ("convert_pressure_to_pose", LaunchConfiguration("convert_pressure_to_pose")),
                    ("pressure_topic", LaunchConfiguration("pressure_topic")),
                    ("pressure_pose_topic", LaunchConfiguration("pressure_pose_topic")),
                    ("output_odom_topic", LaunchConfiguration("output_odom_topic")),
                    ("output_ned_odom_topic", LaunchConfiguration("output_ned_odom_topic")),
                    ("ned_world_frame", LaunchConfiguration("ned_world_frame")),
                ],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
