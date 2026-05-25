import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def namespaced_config(config_file: str, robot_namespace: str) -> str:
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
    if not robot_namespace:
        robot_namespace = "sura"

    package_share = get_package_share_directory("sura_localization")
    config_file = namespaced_config(
        os.path.join(package_share, "config", "ekf_surface_fastlio.yaml"),
        robot_namespace,
    )

    def topic(path: str) -> str:
        return f"/{robot_namespace}/{path}"

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = LaunchConfiguration("robot_description")
    fastlio_cfg = LaunchConfiguration("fastlio_cfg")
    body_frame = LaunchConfiguration("body_frame").perform(context).strip()
    sensor_frame = LaunchConfiguration("sensor_frame").perform(context).strip()
    lidar_topic = LaunchConfiguration("lidar_topic").perform(context).strip()
    imu_topic = LaunchConfiguration("imu_topic").perform(context).strip()

    if not body_frame:
        body_frame = f"{robot_namespace}/base_link_enu"
    if not sensor_frame:
        sensor_frame = f"{robot_namespace}/lidar_front"
    if not lidar_topic:
        lidar_topic = "/livox/lidar"
    if not imu_topic:
        imu_topic = topic("navigator/imu")

    fastlio_parameters = [
        fastlio_cfg,
        {"use_sim_time": use_sim_time},
        {"frames.body_frame": body_frame},
        {"frames.sensor_frame": sensor_frame},
        {"frames.base_link_enu_to_livox_T": [0.39, 0.36, 0.25]},
    ]

    robot_description_value = robot_description.perform(context).strip()
    if robot_description_value:
        fastlio_parameters.append(
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        )

    fastlio_node = Node(
        package="fast_lio",
        namespace="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=fastlio_parameters,
        remappings=[
            ("/livox/lidar", lidar_topic),
            ("/livox/imu", imu_topic),
            ("/cloud_registered_body", topic("fastlio/body_cloud")),
            ("/cloud_registered", topic("fastlio/cloud_registered")),
            ("/cloud_effected", topic("fastlio/cloud_effected")),
            ("/Laser_map", topic("fastlio/laser_map")),
            ("/path", topic("fastlio/path")),
            ("/Odometry", topic("fastlio/odometry")),
            ("/catamaran/odometry", topic("fastlio/odometry")),
        ],
    )

    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    world_frame = LaunchConfiguration("world_frame")
    publish_tf = LaunchConfiguration("publish_tf")

    base_link_frame = LaunchConfiguration("base_link_frame").perform(context)
    if not base_link_frame:
        base_link_frame = f"{robot_namespace}/base_link_enu"
    map_frame_value = map_frame.perform(context)
    if not map_frame_value:
        map_frame_value = f"{robot_namespace}/map"

    output_odom_topic = LaunchConfiguration("output_odom_topic").perform(context)
    if not output_odom_topic:
        output_odom_topic = topic("localization/odometry_enu")

    output_ned_odom_topic = LaunchConfiguration("output_ned_odom_topic").perform(context)
    if not output_ned_odom_topic:
        output_ned_odom_topic = topic("localization/odometry")

    ekf_overrides = {
        "map_frame": map_frame_value,
        "odom_frame": odom_frame,
        "base_link_frame": base_link_frame,
        "world_frame": world_frame,
        "publish_tf": publish_tf,
        "odom0": topic("fastlio/odometry"),
        "odom1": topic("sensors/gps/odometry"),
    }

    return [
        fastlio_node,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_world_enu",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "3.14159265359",
                "--pitch", "0.0",
                "--yaw", "1.57079632679",
                "--frame-id", "world_ned",
                "--child-frame-id", "world_enu",
            ],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            parameters=[
                config_file,
            ],
            remappings=[
                ("gps/fix", topic("sensors/gps/fix")),
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
            parameters=[config_file, ekf_overrides],
            remappings=[("odometry/filtered", output_odom_topic)],
        ),
        Node(
            package="sura_localization",
            executable="enu_to_ned_odometry",
            name="enu_to_ned_odometry",
            output="screen",
            parameters=[
                {
                    "input_topic": output_odom_topic,
                    "output_topic": output_ned_odom_topic,
                    "frame_id": "world_ned",
                    "child_frame_id": f"{robot_namespace}/base_link",
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("robot_description", default_value=""),
            DeclareLaunchArgument(
                "fastlio_cfg",
                default_value=PathJoinSubstitution([
                    FindPackageShare("fast_lio"),
                    "config",
                    "mid360.yaml",
                ]),
            ),
            DeclareLaunchArgument("body_frame", default_value=""),
            DeclareLaunchArgument("sensor_frame", default_value=""),
            DeclareLaunchArgument("lidar_topic", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value=""),
            DeclareLaunchArgument("output_odom_topic", default_value=""),
            DeclareLaunchArgument("output_ned_odom_topic", default_value=""),
            DeclareLaunchArgument("map_frame", default_value=""),
            DeclareLaunchArgument("odom_frame", default_value="world_enu"),
            DeclareLaunchArgument("base_link_frame", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="world_enu"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
