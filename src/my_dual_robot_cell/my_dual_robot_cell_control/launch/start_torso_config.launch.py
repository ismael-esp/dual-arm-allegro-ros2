from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

def launch_setup():
    # ---------------- Existing robot args ----------------
    la_ur_type = LaunchConfiguration("la_ur_type")
    ra_ur_type = LaunchConfiguration("ra_ur_type")
    la_robot_ip = LaunchConfiguration("la_robot_ip")
    ra_robot_ip = LaunchConfiguration("ra_robot_ip")

    controllers_file = LaunchConfiguration("controllers_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    description_launchfile = LaunchConfiguration("description_launchfile")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")

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

    # ---------------- NEW: camera args ----------------
    cam1_serial      = LaunchConfiguration("cam1_serial")
    cam1_namespace   = LaunchConfiguration("cam1_namespace")
    cam1_name        = LaunchConfiguration("cam1_name")
    cam1_child_frame = LaunchConfiguration("cam1_child_frame")  # usually cam1_color_optical_frame

    cam2_serial      = LaunchConfiguration("cam2_serial")
    cam2_namespace   = LaunchConfiguration("cam2_namespace")
    cam2_name        = LaunchConfiguration("cam2_name")
    cam2_child_frame = LaunchConfiguration("cam2_child_frame")  # usually cam2_color_optical_frame
    
    cam3_serial      = LaunchConfiguration("cam3_serial")
    cam3_namespace   = LaunchConfiguration("cam3_namespace")
    cam3_name        = LaunchConfiguration("cam3_name")
    cam3_child_frame = LaunchConfiguration("cam3_child_frame")  # usually cam3_color_optical_frame

    # Static TF params (world -> camera*_color_optical_frame)
    cam1_x = LaunchConfiguration("cam1_x")
    cam1_y = LaunchConfiguration("cam1_y")
    cam1_z = LaunchConfiguration("cam1_z")
    cam1_qx = LaunchConfiguration("cam1_qx")
    cam1_qy = LaunchConfiguration("cam1_qy")
    cam1_qz = LaunchConfiguration("cam1_qz")
    cam1_qw = LaunchConfiguration("cam1_qw")

    cam2_x = LaunchConfiguration("cam2_x")
    cam2_y = LaunchConfiguration("cam2_y")
    cam2_z = LaunchConfiguration("cam2_z")
    cam2_qx = LaunchConfiguration("cam2_qx")
    cam2_qy = LaunchConfiguration("cam2_qy")
    cam2_qz = LaunchConfiguration("cam2_qz")
    cam2_qw = LaunchConfiguration("cam2_qw")

    cam3_x = LaunchConfiguration("cam3_x")
    cam3_y = LaunchConfiguration("cam3_y")
    cam3_z = LaunchConfiguration("cam3_z")
    cam3_qx = LaunchConfiguration("cam3_qx")
    cam3_qy = LaunchConfiguration("cam3_qy")
    cam3_qz = LaunchConfiguration("cam3_qz")
    cam3_qw = LaunchConfiguration("cam3_qw")

    # ---------------- Existing nodes ----------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
        ],
        remappings=[('/joint_states', '/ur_joint_states')],
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
            ] + inactive_flags + controllers,
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
    ]

    controller_spawners = [controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    la_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            la_initial_joint_controller, "-c", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ],
        condition=IfCondition(la_activate_joint_controller),
    )
    ra_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            ra_initial_joint_controller, "-c", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ],
        condition=IfCondition(ra_activate_joint_controller),
    )
    la_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            la_initial_joint_controller, "-c", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout, "--inactive",
        ],
        condition=UnlessCondition(la_activate_joint_controller),
    )
    ra_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            ra_initial_joint_controller, "-c", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout, "--inactive",
        ],
        condition=UnlessCondition(ra_activate_joint_controller),
    )

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(LaunchConfiguration("description_launchfile")),
        launch_arguments={
            "la_robot_ip": la_robot_ip,
            "ra_robot_ip": ra_robot_ip,
            "la_ur_type": la_ur_type,
            "ra_ur_type": ra_ur_type,
        }.items(),
    )
    
    # =-===== New driver allegro hand launch files ======
    # ---------------- NEW: Allegro Hands (Updated Driver) ----------------
    # Path to the new driver package
    allegro_pkg = FindPackageShare("allegro_hand_ros")
    allegro_launch_file = PathJoinSubstitution([allegro_pkg, "launch", "hand.launch.py"])

    # LEFT Allegro Hand (P249)
    la_allegro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(allegro_launch_file),
        launch_arguments={
            "yaml_file": "/home/dev/ros2_ws/src/allegro_hand_cust/allegro_hand/conf/configuration_files/V4P249.yaml",
            "channel": "32", #33
            "hand": "left",
            "rviz": "false",
            "slave": "false",
            "link_prefix": "la_",
            "node_namespace": "la_",
            "initial_controller": "JointPositionPID", #GravityCompensator
            "fake_hand": "false",
            "tip": "default"
        }.items()
    )

    # RIGHT Allegro Hand (P231)
    ra_allegro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(allegro_launch_file),
        launch_arguments={
            "yaml_file": "/home/dev/ros2_ws/src/allegro_hand_cust/allegro_hand/conf/configuration_files/V4P231.yaml",
            "channel": "33", #32
            "hand": "right",    
            "rviz": "false",
            "slave": "false",
            "link_prefix": "ra_",
            "node_namespace": "ra_",
            "initial_controller": "JointPositionPID", #JointPositionPID
            "fake_hand": "false",
            "tip": "default"
        }.items()
    )

    # --- Joint State Merger ---
    merger_node = Node(
        package="my_dual_robot_cell_control",
        executable="joint_state_merger_new",
        name="joint_state_merger_new",
        output="screen"
    )

    # ---------------- NEW: RealSense cameras ----------------
    realsense_pkg = FindPackageShare("realsense2_camera")

    cam1_color_info_url = LaunchConfiguration("cam1_color_info_url")
    cam2_color_info_url = LaunchConfiguration("cam2_color_info_url")
    cam3_color_info_url = LaunchConfiguration("cam3_color_info_url")

    cam1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([realsense_pkg, "launch", "rs_launch.py"])),
        launch_arguments={
            "serial_no": cam1_serial,
            "camera_namespace": cam1_namespace,   # topics
            "camera_name": cam1_name,             # TF frame prefix (e.g., cam1_color_optical_frame)
            "enable_rgbd": "true",
            "enable_sync": "true",
            "align_depth.enable" : "true",
            "enable_color": "true",
            "enable_depth":"true",
            "pointcloud.enable": "true",
            "rgb_camera.color_profile": "1280x720x30", # 424x240x60 2026-06-12: 848x480@60 (policy v2; D415 color max 60fps res) — was 424x240x60
            "depth_module.depth_profile":"1280x720x30", # 424x240x60 2026-06-12: match color profile — was 424x240x60
            "color.camera_info_url": cam1_color_info_url,
        }.items()
    )

    cam2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([realsense_pkg, "launch", "rs_launch.py"])),
        launch_arguments={
            "serial_no": cam2_serial,
            "camera_namespace": cam2_namespace,
            "camera_name": cam2_name,
            "enable_rgbd": "true",
            "enable_sync": "true",
            "align_depth.enable" : "true",
            "enable_color": "true",
            "enable_depth":"true",
            "pointcloud.enable": "true",
            "rgb_camera.color_profile": "1280x720x30", # must match the resolution used during intrinsic calibration
            "depth_module.depth_profile":"1280x720x30",
            "color.camera_info_url": cam2_color_info_url,
        }.items()
    )

    cam3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([realsense_pkg, "launch", "rs_launch.py"])),
        launch_arguments={
            "serial_no": cam3_serial,
            "camera_namespace": cam3_namespace,
            "camera_name": cam3_name,
            "enable_rgbd": "true",
            "enable_sync": "true",
            "align_depth.enable" : "true",
            "enable_color": "true",
            "enable_depth":"true",
            "pointcloud.enable": "true",
            "rgb_camera.color_profile": "1280x720x30", # 424x240x60 2026-06-12: 848x480x60 (policy v2; D415 color max 60fps res) — was 424x240x60
            "depth_module.depth_profile":"1280x720x30", # 424x240x60 2026-06-12: match color profile — was 424x240x60
            "color.camera_info_url": cam3_color_info_url,
        }.items()
    )

    # ---------------- NEW: Static TFs (world -> *_link) ----------------
    cam1_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", cam1_child_frame,
            "--x", cam1_x, "--y", cam1_y, "--z", cam1_z,
            "--qx", cam1_qx, "--qy", cam1_qy, "--qz", cam1_qz, "--qw", cam1_qw,
        ],
    )

    cam2_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id", "ra_base_link",
            "--child-frame-id", cam2_child_frame,
            "--x", cam2_x, "--y", cam2_y, "--z", cam2_z,
            "--qx", cam2_qx, "--qy", cam2_qy, "--qz", cam2_qz, "--qw", cam2_qw,
        ],
    )
    
    cam3_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id", "world",
            "--child-frame-id", cam3_child_frame,
            "--x", cam3_x, "--y", cam3_y, "--z", cam3_z,
            "--qx", cam3_qx, "--qy", cam3_qy, "--qz", cam3_qz, "--qw", cam3_qw,
        ],
    )

    # ---------------- Final node list ----------------
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
        # RealSense cams + TFs
        cam1_launch, cam2_launch, cam3_launch,
        cam1_tf_node, cam3_tf_node, cam2_tf_node, 
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

    # --------------- Existing robot arg declarations (unchanged) ---------------
    declared_arguments += [
        DeclareLaunchArgument(
            "la_ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3","ur3e","ur5","ur5e","ur10","ur10e","ur16e","ur20","ur30"],
            default_value="ur3e",
        ),
        DeclareLaunchArgument(
            "ra_ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3","ur3e","ur5","ur5e","ur10","ur10e","ur16e","ur20","ur30"],
            default_value="ur3",
        ),
        DeclareLaunchArgument("la_robot_ip", default_value="192.168.0.121"),
        DeclareLaunchArgument("ra_robot_ip", default_value="192.168.0.120"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("my_dual_robot_cell_control"), "config", "combined_controllers.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [FindPackageShare("my_dual_robot_cell_control"), "launch", "rsp.launch.py"]
            ),
        ),
        DeclareLaunchArgument("la_use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("ra_use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("la_mock_sensor_commands", default_value="false"),
        DeclareLaunchArgument("ra_mock_sensor_commands", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="10"),
        DeclareLaunchArgument("la_initial_joint_controller", default_value="la_forward_position_controller"),
        DeclareLaunchArgument("ra_initial_joint_controller", default_value="ra_forward_position_controller"),
        DeclareLaunchArgument("la_activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("ra_activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("my_dual_robot_cell_control"), "config", "config_dual_system.rviz"]
            ),
        ),
        DeclareLaunchArgument("la_launch_dashboard_client", default_value="true"),
        DeclareLaunchArgument("ra_launch_dashboard_client", default_value="true"),
        DeclareLaunchArgument(
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution([FindPackageShare("my_dual_robot_cell_control"), "config"]),
                "/", "update_rate.yaml",
            ],
        ),
    ]

    # --------------- NEW: camera arg declarations ---------------
    declared_arguments += [
        # Camera 1 (front)
        DeclareLaunchArgument("cam1_serial",     default_value="_839512060149",
                              description="RealSense cam1 serial number"),
        DeclareLaunchArgument("cam1_namespace",  default_value="front_right",
                              description="ROS namespace for cam1 topics"),
        DeclareLaunchArgument("cam1_name",       default_value="cam1",
                              description="Frame prefix / camera_name for cam1"),
        DeclareLaunchArgument("cam1_child_frame", default_value="cam1_link", #cam1_color_optical_frame
                              description="TF child frame for cam1 static transform"),
        # Camera 2 (head)
        DeclareLaunchArgument("cam2_serial",     default_value="_121622061488",
                              description="RealSense cam2 serial number"),
        DeclareLaunchArgument("cam2_namespace",  default_value="extra", #head
                              description="ROS namespace for cam2 topics"),
        DeclareLaunchArgument("cam2_name",       default_value="cam2",
                              description="Frame prefix / camera_name for cam2"),
        DeclareLaunchArgument("cam2_child_frame", default_value="cam2_link", #cam2_color_optical_frame
                              description="TF child frame for cam2 static transform"),
        
        # Camera 3 (back)
        DeclareLaunchArgument("cam3_serial",     default_value="_217222067476",
                                description="RealSense cam3 serial number"),
        DeclareLaunchArgument("cam3_namespace",  default_value="front_left",
                                description="ROS namespace for cam3 topics"),
        DeclareLaunchArgument("cam3_name",       default_value="cam3",
                                description="Frame prefix / camera_name for cam3"),
        DeclareLaunchArgument("cam3_child_frame", default_value="cam3_link", #cam
                                description="TF child frame for cam3 static transform"),

        # --- Per-camera color intrinsic calibration YAML (camera_info_url) ---
        # Empty string = use factory firmware intrinsics. Set to
        # file:///abs/path/to/yaml to override. Resolution in the YAML must
        # match the streamed rgb_camera.color_profile or override is ignored.
        DeclareLaunchArgument("cam1_color_info_url", default_value="",
                              description="file:// URL to cam1 color intrinsics YAML (optional)"),
        DeclareLaunchArgument("cam2_color_info_url",
                              default_value="file:///home/dev/ros2_ws/src/realsense-ros/cam2_color_intrinsics.yaml",
                              description="file:// URL to cam2 color intrinsics YAML"),
        DeclareLaunchArgument("cam3_color_info_url", default_value="",
                              description="file:// URL to cam3 color intrinsics YAML (optional)"),
    ]

    # --------------- NEW: static TF parameter declarations ---------------
    # Matches the numbers you provided (top & front). Adjust here if you re-calibrate.
    declared_arguments += [
        # cam1 (front_right)
        DeclareLaunchArgument("cam1_x",  default_value="-0.21841603"),
        DeclareLaunchArgument("cam1_y",  default_value="1.49491771"),
        DeclareLaunchArgument("cam1_z",  default_value="1.54374578"),
        DeclareLaunchArgument("cam1_qx", default_value="0.10440205"),
        DeclareLaunchArgument("cam1_qy", default_value="0.08496433"),
        DeclareLaunchArgument("cam1_qz", default_value="-0.65939017"),
        DeclareLaunchArgument("cam1_qw", default_value="0.73965254"),

        # cam2 (head)
        # DeclareLaunchArgument("cam2_x",  default_value="-0.02101440"),
        # DeclareLaunchArgument("cam2_y",  default_value="0.13125413"),
        # DeclareLaunchArgument("cam2_z",  default_value="1.62182867"),
        # DeclareLaunchArgument("cam2_qx", default_value="-0.41312574"),
        # DeclareLaunchArgument("cam2_qy", default_value="0.38114708"),
        # DeclareLaunchArgument("cam2_qz", default_value="0.58194579"),
        # DeclareLaunchArgument("cam2_qw", default_value="0.58770156"),

        # cam2 (extra pose)
        DeclareLaunchArgument("cam2_x",  default_value="0.282625"),
        DeclareLaunchArgument("cam2_y",  default_value="1.189474"),
        DeclareLaunchArgument("cam2_z",  default_value="0.555863"),
        DeclareLaunchArgument("cam2_qx", default_value="0.765963"),
        DeclareLaunchArgument("cam2_qy", default_value="-0.611474"),
        DeclareLaunchArgument("cam2_qz", default_value="-0.122388"),
        DeclareLaunchArgument("cam2_qw", default_value="0.156523"),
        
        # cam3 (front_left)
        DeclareLaunchArgument("cam3_x",  default_value="0.01841603"),
        DeclareLaunchArgument("cam3_y",  default_value="1.49491771"),
        DeclareLaunchArgument("cam3_z",  default_value="1.54374578"),
        DeclareLaunchArgument("cam3_qx", default_value="0.10440205"),
        DeclareLaunchArgument("cam3_qy", default_value="0.08496433"),
        DeclareLaunchArgument("cam3_qz", default_value="-0.65939017"),
        DeclareLaunchArgument("cam3_qw", default_value="0.73965254"),
    ]

    return LaunchDescription(declared_arguments + launch_setup())
