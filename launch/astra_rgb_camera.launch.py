"""Launch the standalone Astra Pro Plus RGB publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device = LaunchConfiguration("device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device",
                default_value="",
                description="V4L2 node; empty automatically finds USB 2bc5:050f.",
            ),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument("fps", default_value="60.0"),
            Node(
                package="astra_rgb_camera",
                executable="rgb_camera_node",
                name="astra_rgb_camera",
                output="screen",
                parameters=[
                    {
                        "device": device,
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "fps": ParameterValue(fps, value_type=float),
                    }
                ],
                additional_env={"PYTHONNOUSERSITE": "1"},
            ),
        ]
    )
