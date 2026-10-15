from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    la_ur_type = LaunchConfiguration("la_ur_type")
    ra_ur_type = LaunchConfiguration("ra_ur_type")

    la_robot_ip = LaunchConfiguration("la_robot_ip")
    ra_robot_ip = LaunchConfiguration("ra_robot_ip")

    la_use_mock_hardware = LaunchConfiguration("la_use_mock_hardware")
    la_mock_sensor_commands = LaunchConfiguration("la_mock_sensor_commands")
    ra_use_mock_hardware = LaunchConfiguration("ra_use_mock_hardware")
    ra_mock_sensor_commands = LaunchConfiguration("ra_mock_sensor_commands")

    headless_mode = LaunchConfiguration("headless_mode")

    la_kinematics_parameters_file = LaunchConfiguration(
        "la_kinematics_parameters_file"
    )
    ra_kinematics_parameters_file = LaunchConfiguration(
        "ra_kinematics_parameters_file"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "urdf",
                    "my_dual_robot_cell_controlled.urdf.xacro",
                ]
            ),
            " ",
            "la_robot_ip:=",
            la_robot_ip,
            " ",
            "ra_robot_ip:=",
            ra_robot_ip,
            " ",
            "la_ur_type:=",
            la_ur_type,
            " ",
            "ra_ur_type:=",
            ra_ur_type,
            " ",
            "la_use_mock_hardware:=",
            la_use_mock_hardware,
            " ",
            "ra_use_mock_hardware:=",
            ra_use_mock_hardware,
            " ",
            "la_kinematics_parameters_file:=",
            la_kinematics_parameters_file,
            " ",
            "ra_kinematics_parameters_file:=",
            ra_kinematics_parameters_file,
            " ",
            "la_mock_sensor_commands:=",
            la_mock_sensor_commands,
            " ",
            "ra_mock_sensor_commands:=",
            ra_mock_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "la_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "la_robot_ip",
            default_value="192.168.0.121",
            description="IP address by which la can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_robot_ip",
            default_value="192.168.0.120",
            description="IP address by which ra can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "la_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "left_tshape_calibration.yaml",  #la_calibration.yaml for the table config
                ]
            ),
            description="The calibration configuration of la.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "right_tshape_calibration.yaml", # ra_calibration.yaml for the table config
                ]
            ),
            description="The calibration configuration of ra.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "la_use_mock_hardware",
            default_value="false",
            description="Start la with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_use_mock_hardware",
            default_value="false",
            description="Start ra with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "la_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for la's sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for ra's sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )
