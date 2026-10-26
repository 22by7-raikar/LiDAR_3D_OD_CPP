import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("lidar_od_ros"), "config", "default.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("points", default_value="/points", description="input PointCloud2 topic"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="lidar_od_ros",
            executable="lidar_od_node",
            name="lidar_od",
            output="screen",
            parameters=[{"config_file": LaunchConfiguration("config_file")}],
            remappings=[("points", LaunchConfiguration("points"))],
        ),
    ])
