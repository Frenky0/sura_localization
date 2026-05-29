import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bool_launch_arg(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False

    raise RuntimeError(
        f"Unsupported value '{value}' for launch argument '{name}'. Use true or false."
    )


def namespaced_frame(robot_namespace, frame_name):
    if robot_namespace:
        return f"{robot_namespace}/{frame_name}"
    return frame_name


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    config_package = LaunchConfiguration("config_package").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    config_path = os.path.join(get_package_share_directory(config_package), config_file)

    frame_convention = LaunchConfiguration("frame_convention").perform(context)
    if frame_convention not in ("ned", "enu"):
        raise RuntimeError("Launch argument 'frame_convention' must be 'ned' or 'enu'.")

    map_frame = LaunchConfiguration("map_frame").perform(context)
    if not map_frame:
        map_frame = namespaced_frame(robot_namespace, "map")

    base_link_frame = LaunchConfiguration("base_link_frame").perform(context)
    if not base_link_frame:
        base_link_frame = namespaced_frame(robot_namespace, "base_link")

    output_odom_topic = LaunchConfiguration("output_odom_topic").perform(context)
    if not output_odom_topic:
        output_odom_topic = "odometry/filtered"

    output_ned_odom_topic = LaunchConfiguration("output_ned_odom_topic").perform(context)
    if not output_ned_odom_topic:
        output_ned_odom_topic = "localization/odometry"

    ekf_overrides = {
        "map_frame": map_frame,
        "odom_frame": LaunchConfiguration("odom_frame"),
        "base_link_frame": base_link_frame,
        "world_frame": LaunchConfiguration("world_frame"),
        "publish_tf": LaunchConfiguration("publish_tf"),
    }

    nodes = [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config_path, ekf_overrides],
        ),
    ]

    if bool_launch_arg(context, "use_navsat"):
        datum_latitude = float(LaunchConfiguration("datum_latitude").perform(context))
        datum_longitude = float(LaunchConfiguration("datum_longitude").perform(context))
        datum_heading = float(LaunchConfiguration("datum_heading").perform(context))

        nodes.append(
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "wait_for_datum": bool_launch_arg(context, "wait_for_datum"),
                        "datum": [datum_latitude, datum_longitude, datum_heading],
                    },
                ],
            )
        )

    if bool_launch_arg(context, "convert_imu_ned_to_enu"):
        nodes.append(
            Node(
                package="sura_localization",
                executable="ned_to_enu_imu",
                name="imu_ned_to_enu",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("imu_ned_topic"),
                        "output_topic": LaunchConfiguration("imu_enu_topic"),
                        "frame_id": LaunchConfiguration("imu_enu_frame"),
                    }
                ],
            )
        )

    if frame_convention == "enu":
        nodes.append(
            Node(
                package="sura_localization",
                executable="enu_to_ned_odometry",
                name="enu_to_ned_odometry",
                output="screen",
                parameters=[
                    {
                        "input_topic": output_odom_topic,
                        "output_topic": output_ned_odom_topic,
                        "frame_id": LaunchConfiguration("ned_world_frame"),
                        "child_frame_id": base_link_frame,
                    }
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value=""),
            DeclareLaunchArgument("config_package", default_value="sura_localization"),
            DeclareLaunchArgument("config_file", default_value="config/auv_localization.yaml"),
            DeclareLaunchArgument("frame_convention", default_value="enu"),
            DeclareLaunchArgument("map_frame", default_value=""),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_link_frame", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="odom"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("use_navsat", default_value="true"),
            DeclareLaunchArgument("wait_for_datum", default_value="true"),
            DeclareLaunchArgument("datum_latitude", default_value="0.0"),
            DeclareLaunchArgument("datum_longitude", default_value="0.0"),
            DeclareLaunchArgument("datum_heading", default_value="0.0"),
            DeclareLaunchArgument("convert_imu_ned_to_enu", default_value="false"),
            DeclareLaunchArgument("imu_ned_topic", default_value="sensors/imu"),
            DeclareLaunchArgument("imu_enu_topic", default_value="sensors/imu_enu"),
            DeclareLaunchArgument("imu_enu_frame", default_value="imu_link"),
            DeclareLaunchArgument("output_odom_topic", default_value=""),
            DeclareLaunchArgument("output_ned_odom_topic", default_value=""),
            DeclareLaunchArgument("ned_world_frame", default_value="world_ned"),
            OpaqueFunction(function=launch_setup),
        ]
    )
