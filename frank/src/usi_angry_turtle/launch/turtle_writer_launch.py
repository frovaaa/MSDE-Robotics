from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="turtlesim",
                namespace="",
                executable="turtlesim_node",
                name="sim",
                ros_arguments=["--log-level", "info"],
            ),
            Node(
                package="usi_angry_turtle",
                namespace="usi_angry_turtle",
                executable="writer_node",
                name="writer",
                ros_arguments=["--log-level", "info"],
            ),
        ]
    )
