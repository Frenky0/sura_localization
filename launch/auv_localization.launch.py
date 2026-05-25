import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def namespaced_config(config_file, robot_namespace):
    text = Path(config_file).read_text(encoding="utf-8")
    text = text.replace("/cirtesub/", f"/{robot_namespace}/")
    text = text.replace("cirtesub/", f"{robot_namespace}/")
    text = text.replace("/blueboat/", f"/{robot_namespace}/")
    text = text.replace("blueboat/", f"{robot_namespace}/")
    text = text.replace("/bluerov/", f"/{robot_namespace}/")
    text = text.replace("bluerov/", f"{robot_namespace}/")
    text = text.replace("blueboat/map", f"{robot_namespace}/map")
    text = text.replace("bluerov/map", f"{robot_namespace}/map")

    output_file = f"/tmp/sura_localization_{robot_namespace}_{Path(config_file).name}"
    Path(output_file).write_text(text, encoding="utf-8")
    return output_file


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    package_share = get_package_share_directory("sura_localization")
    aruco_share = get_package_share_directory("cirtesu_tank_aruco_localization")

    config_file = namespaced_config(
        os.path.join(package_share, "config", "ekf_auv.yaml"),
        robot_namespace,
    )
    aruco_config_file = namespaced_config(
        os.path.join(aruco_share, "config", "aruco_map.yaml"),
        robot_namespace,
    )

    def topic(path):
        return f"/{robot_namespace}/{path}"

    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    world_frame = LaunchConfiguration("world_frame")
    publish_tf = LaunchConfiguration("publish_tf")
    base_link_frame = LaunchConfiguration("base_link_frame").perform(context)
    if not base_link_frame:
        base_link_frame = f"{robot_namespace}/base_link"
    map_frame_value = map_frame.perform(context)
    if not map_frame_value:
        map_frame_value = f"{robot_namespace}/map"

    output_odom_topic = LaunchConfiguration("output_odom_topic").perform(context)
    if not output_odom_topic:
        output_odom_topic = topic("localization/odometry_enu")

    output_ned_odom_topic = LaunchConfiguration("output_ned_odom_topic").perform(context)
    if not output_ned_odom_topic:
        output_ned_odom_topic = topic("localization/odometry")

    datum_latitude = float(LaunchConfiguration("datum_latitude").perform(context))
    datum_longitude = float(LaunchConfiguration("datum_longitude").perform(context))
    datum_heading = float(LaunchConfiguration("datum_heading").perform(context))

    frame_overrides = {
        "map_frame": map_frame_value,
        "odom_frame": odom_frame,
        "base_link_frame": base_link_frame,
        "world_frame": world_frame,
        "publish_tf": publish_tf,
        "pose0": topic("sensors/pressure/pose"),
        "pose1": topic("sensors/aruco/pose_enu"),
        "odom0": topic("sensors/gps/odometry"),
        "twist0": topic("sensors/dvl/twist"),
        "imu0": topic("sensors/imu_enu"),
    }

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_world_enu",
            output="screen",
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "3.14159265359",
                "--pitch",
                "0.0",
                "--yaw",
                "1.57079632679",
                "--frame-id",
                "world_ned",
                "--child-frame-id",
                "world_enu",
            ],
        ),
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
            package="sura_localization",
            executable="ned_to_enu_imu",
            name="imu_ned_to_enu",
            output="screen",
            parameters=[
                {
                    "input_topic": topic("sensors/imu"),
                    "output_topic": topic("sensors/imu_enu"),
                    "frame_id": f"{robot_namespace}/IMU",
                }
            ],
        ),
        Node(
            package="cirtesub_stonefish",
            executable="pressure_to_pose.py",
            name="pressure_to_pose",
            output="screen",
            parameters=[
                {
                    "input_topic": topic("sensors/pressure"),
                    "output_topic": topic("sensors/pressure/pose"),
                    "frame_id": "world_enu",
                    "sensor_frame_id": f"{robot_namespace}/Pressure",
                    "positive_down": True,
                    "fallback_z_variance": 0.01,
                }
            ],
        ),
        Node(
            package="cirtesu_tank_aruco_localization",
            executable="aruco_map_localization_node",
            name="aruco_map_localization",
            output="screen",
            parameters=[aruco_config_file],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "wait_for_datum": True,
                    "datum": [datum_latitude, datum_longitude, datum_heading],
                },
            ],
            remappings=[
                ("gps/fix", topic("sensors/gps")),
                ("imu", topic("sensors/imu_enu")),
                ("odometry/filtered", output_odom_topic),
                ("odometry/gps", topic("sensors/gps/odometry")),
                ("gps/filtered", topic("sensors/gps/filtered")),
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config_file, frame_overrides],
            remappings=[("odometry/filtered", output_odom_topic)],
        ),
        Node(
            package="sura_localization",
            executable="enu_to_ned_odometry",
            name="gps_enu_to_ned_odometry",
            output="screen",
            parameters=[
                {
                    "input_topic": output_odom_topic,
                    "output_topic": output_ned_odom_topic,
                    "frame_id": "world_ned",
                    "child_frame_id": base_link_frame,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument("output_odom_topic", default_value=""),
            DeclareLaunchArgument("output_ned_odom_topic", default_value=""),
            DeclareLaunchArgument("map_frame", default_value=""),
            DeclareLaunchArgument("odom_frame", default_value="world_enu"),
            DeclareLaunchArgument("base_link_frame", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="world_enu"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("datum_latitude", default_value="39.9944"),
            DeclareLaunchArgument("datum_longitude", default_value="-0.0741"),
            DeclareLaunchArgument("datum_heading", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
