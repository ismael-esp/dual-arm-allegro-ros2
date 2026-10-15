from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup():
    # Initialize Arguments
    la_ur_type = LaunchConfiguration("la_ur_type")
    ra_ur_type = LaunchConfiguration("ra_ur_type")

    la_robot_ip = LaunchConfiguration("la_robot_ip")
    ra_robot_ip = LaunchConfiguration("ra_robot_ip")

    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    description_launchfile = LaunchConfiguration("description_launchfile")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")

    # Robot specific arguments
    la_use_mock_hardware = LaunchConfiguration("la_use_mock_hardware")
    la_mock_sensor_commands = LaunchConfiguration("la_mock_sensor_commands")
    la_initial_joint_controller = LaunchConfiguration("la_initial_joint_controller")
    la_activate_joint_controller = LaunchConfiguration("la_activate_joint_controller")
    la_launch_dashboard_client = LaunchConfiguration("la_launch_dashboard_client")

    ra_use_mock_hardware = LaunchConfiguration("ra_use_mock_hardware")
    ra_mock_sensor_commands = LaunchConfiguration("ra_mock_sensor_commands")
    ra_initial_joint_controller = LaunchConfiguration("ra_initial_joint_controller")
    ra_activate_joint_controller = LaunchConfiguration("ra_activate_joint_controller")
    ra_launch_dashboard_client = LaunchConfiguration("ra_launch_dashboard_client")
    
    # Single controller manager comprising of controllers for both arms
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
            # We use the tf_prefix as substitution in there, so that's why we keep it as an
            # argument for this launchfile
        ],
        remappings=[
        ('/joint_states', '/ur_joint_states'),  # <--- Remap here
        ],
        output="screen",
    )

    la_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(la_launch_dashboard_client) and UnlessCondition(la_use_mock_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        namespace="la",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": la_robot_ip}],
    )

    ra_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(ra_launch_dashboard_client) and UnlessCondition(ra_use_mock_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        namespace="ra",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": ra_robot_ip}],
    )

    la_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="la",
        parameters=[{"robot_ip": la_robot_ip}],
        output="screen",
        condition=UnlessCondition(la_use_mock_hardware),
    )

    ra_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="ra",
        parameters=[{"robot_ip": ra_robot_ip}],
        output="screen",
        condition=UnlessCondition(ra_use_mock_hardware),
    )

    la_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        namespace="la",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(la_use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": la_activate_joint_controller},
            {
                "consistent_controllers": [
                    "joint_state_broadcaster",
                    "la_io_and_status_controller",
                    "ra_io_and_status_controller",
                    "la_force_torque_sensor_broadcaster",
                    "ra_force_torque_sensor_broadcaster",
                    "la_speed_scaling_state_broadcaster",
                    "ra_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    ra_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        namespace="ra",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(ra_use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": ra_activate_joint_controller},
            {
                "consistent_controllers": [
                    "joint_state_broadcaster",
                    "la_io_and_status_controller",
                    "ra_io_and_status_controller",
                    "la_force_torque_sensor_broadcaster",
                    "ra_force_torque_sensor_broadcaster",
                    "la_speed_scaling_state_broadcaster",
                    "ra_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "la_io_and_status_controller",
        "ra_io_and_status_controller",
        "la_speed_scaling_state_broadcaster",
        "ra_speed_scaling_state_broadcaster",
        "la_force_torque_sensor_broadcaster",
        "ra_force_torque_sensor_broadcaster",
    ]
    controllers_inactive = [
        "la_forward_position_controller",
        "ra_forward_position_controller",
    ]

    controller_spawners = [controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    # There may be other controllers of the joints, but this is the initially-started one
    la_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            la_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(la_activate_joint_controller),
    )
    ra_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            ra_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(ra_activate_joint_controller),
    )
    la_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            la_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(la_activate_joint_controller),
    )
    ra_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            ra_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(ra_activate_joint_controller),
    )

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "la_robot_ip": la_robot_ip,
            "ra_robot_ip": ra_robot_ip,
            "la_ur_type": la_ur_type,
            "ra_ur_type": ra_ur_type,
        }.items(),
    )
    
    # --- Allegro Hands (left & right) ---
    allegro_pkg = FindPackageShare("allegro_hand_controllers")

    la_allegro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([allegro_pkg, "launch", "allegro_hand_adapted.launch.py"])
        ),
        launch_arguments={
            "NUM": "1",
            "CAN_DEVICE": "can1",
            "HAND": "left",
            "RVIZ": "false",
            "NS": "la_"
        }.items()
    )

    ra_allegro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([allegro_pkg, "launch", "allegro_hand_adapted.launch.py"])
        ),
        launch_arguments={
            "NUM": "0",
            "CAN_DEVICE": "can0",
            "HAND": "right",
            "RVIZ": "false",
            "NS": "ra_"
        }.items()
    )

    # --- Joint State Merger (publishes unified /joint_states) ---
    merger_node = Node(
        package="my_dual_robot_cell_control",
        executable="joint_state_merger",
        name="joint_state_merger",
        output="screen"
    )

    nodes_to_start = [
        control_node,
        la_dashboard_client_node,
        ra_dashboard_client_node,
        la_controller_stopper_node,
        ra_controller_stopper_node,
        la_urscript_interface,
        ra_urscript_interface,
        rsp,
        # Allegro hands
        la_allegro_launch,
        ra_allegro_launch,
        # Merger
        TimerAction(period=2.0, actions=[merger_node]),
        rviz_node,
        la_initial_joint_controller_spawner_stopped,
        ra_initial_joint_controller_spawner_stopped,
        la_initial_joint_controller_spawner_started,
        ra_initial_joint_controller_spawner_started,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
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
            default_value="ur3",
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

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "combined_controllers.yaml",
                ]
            ),
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "launch",
                    "rsp.launch.py",
                ]
            ),
            description="Launchfile (absolute path) providing the description. "
            "The launchfile has to start a robot_state_publisher node that "
            "publishes the description topic.",
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
            description="Enable headless mode for robot control for both arms.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "la_initial_joint_controller",
            default_value="la_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the la robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_initial_joint_controller",
            default_value="ra_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the ra robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "la_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the la robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the ra robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("my_dual_robot_cell_description"), "rviz", "urdf.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "la_launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("my_dual_robot_cell_control"),
                        "config",
                    ]
                ),
                "/",
                "update_rate.yaml",
            ],
        )
    )
    return LaunchDescription(declared_arguments + launch_setup())
