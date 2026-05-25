import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("sura_localization")
    imu_only_config = os.path.join(package_share, "config", "ekf_imu_only.yaml")
    depth_config = os.path.join(package_share, "config", "ekf_with_depth.yaml")

    use_depth = LaunchConfiguration("use_depth")
    robot_namespace = LaunchConfiguration("robot_namespace")
    imu_topic = LaunchConfiguration("imu_topic")
    depth_odom_topic = LaunchConfiguration("depth_odom_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    publish_tf = LaunchConfiguration("publish_tf")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_link_frame = LaunchConfiguration("base_link_frame")
    world_frame = LaunchConfiguration("world_frame")

    common_parameter_overrides = {
        "publish_tf": publish_tf,
        "map_frame": map_frame,
        "odom_frame": odom_frame,
        "base_link_frame": base_link_frame,
        "world_frame": world_frame,
        "imu0": imu_topic,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument("use_depth", default_value="false"),
            DeclareLaunchArgument("imu_topic", default_value=["/", robot_namespace, "/imu/data"]),
            DeclareLaunchArgument("depth_odom_topic", default_value=["/", robot_namespace, "/depth/odometry"]),
            DeclareLaunchArgument(
                "output_odom_topic",
                default_value=["/", robot_namespace, "/localization/odometry"],
            ),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("map_frame", default_value=[robot_namespace, "/map"]),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_link_frame", default_value=[robot_namespace, "/base_link"]),
            DeclareLaunchArgument("world_frame", default_value="odom"),
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
                condition=UnlessCondition(use_depth),
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[imu_only_config, common_parameter_overrides],
                remappings=[("odometry/filtered", output_odom_topic)],
            ),
            Node(
                condition=IfCondition(use_depth),
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    depth_config,
                    {
                        **common_parameter_overrides,
                        "odom0": depth_odom_topic,
                    },
                ],
                remappings=[("odometry/filtered", output_odom_topic)],
            ),
        ]
    )
