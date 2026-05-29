import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

    package_share = get_package_share_directory("sura_localization")
    aruco_share = get_package_share_directory("cirtesu_tank_aruco_localization")

    config_file = os.path.join(package_share, "config", "cirtesu_auv_localization.yaml")
    aruco_config_file = os.path.join(aruco_share, "config", "aruco_map.yaml")

    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    world_frame_value = LaunchConfiguration("world_frame").perform(context)
    frame_convention = LaunchConfiguration("frame_convention").perform(context)
    if frame_convention not in ("ned", "enu"):
        raise RuntimeError("Launch argument 'frame_convention' must be 'ned' or 'enu'.")

    publish_tf = LaunchConfiguration("publish_tf")
    base_link_frame = LaunchConfiguration("base_link_frame").perform(context)
    if not base_link_frame:
        base_link_frame = f"{robot_namespace}/base_link"
    map_frame_value = map_frame.perform(context)
    if not map_frame_value:
        map_frame_value = f"{robot_namespace}/map"

    output_odom_topic = LaunchConfiguration("output_odom_topic").perform(context)
    if not output_odom_topic:
        output_odom_topic = "odometry/filtered"

    output_ned_odom_topic = LaunchConfiguration("output_ned_odom_topic").perform(context)
    if not output_ned_odom_topic:
        output_ned_odom_topic = "localization/odometry"

    datum_latitude = float(LaunchConfiguration("datum_latitude").perform(context))
    datum_longitude = float(LaunchConfiguration("datum_longitude").perform(context))
    datum_heading = float(LaunchConfiguration("datum_heading").perform(context))

    frame_overrides = {
        "map_frame": map_frame_value,
        "odom_frame": odom_frame,
        "base_link_frame": base_link_frame,
        "world_frame": world_frame_value,
        "publish_tf": publish_tf,
    }
    aruco_overrides = {
        "base_frame": f"{robot_namespace}/base_link",
        "camera_frame": f"{robot_namespace}/down_camera/camera",
        "aruco_topic": "down_camera/aruco_detections",
        "marker_topic": "aruco/markers",
        "pose_topic": "sensors/aruco/pose_enu",
    }

    nodes = [
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
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_cirtesu_tank",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "3.1416",
                "--frame-id", "world_ned",
                "--child-frame-id", "cirtesu_tank",
            ],
        ),
        Node(
            package="sura_localization",
            executable="ned_to_enu_imu",
            name="imu_ned_to_enu",
            output="screen",
            parameters=[
                    {
                        "input_topic": "sensors/imu",
                        "output_topic": "sensors/imu_enu",
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
                        "input_topic": "sensors/pressure",
                        "output_topic": "sensors/pressure/pose",
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
            parameters=[aruco_config_file, aruco_overrides],
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
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config_file, frame_overrides],
        ),
    ]

    if frame_convention == "enu":
        nodes.append(
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
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace"),
            DeclareLaunchArgument("output_odom_topic", default_value=""),
            DeclareLaunchArgument("output_ned_odom_topic", default_value=""),
            DeclareLaunchArgument("map_frame", default_value=""),
            DeclareLaunchArgument("odom_frame", default_value="world_enu"),
            DeclareLaunchArgument("base_link_frame", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="world_enu"),
            DeclareLaunchArgument("frame_convention", default_value="enu"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("datum_latitude", default_value="39.9944"),
            DeclareLaunchArgument("datum_longitude", default_value="-0.0741"),
            DeclareLaunchArgument("datum_heading", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
