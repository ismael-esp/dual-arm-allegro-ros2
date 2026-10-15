""" Static transform publisher: world -> camera_color_optical_frame """

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id", "world",
                "--child-frame-id", "camera_color_optical_frame",
                "--x", "-0.007763",
                "--y", "0.439065",
                "--z", "1.96248",
                # Quaternion (x, y, z, w)
                # RPY (rad):   roll = -3.027620, pitch = 0.016572, yaw = 3.131096
                # RPY (deg):   roll = -173.4699°, pitch = 0.9495°,  yaw = 179.3986°
                "--qx", "0.0057113",
                "--qy", "0.9983262",
                "--qz", "-0.0569960",
                "--qw", "0.0079736",
            ],
        ),
    ]
    return LaunchDescription(nodes)

