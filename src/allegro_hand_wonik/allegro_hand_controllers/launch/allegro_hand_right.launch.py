from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # New launch argument to set the namespace prefix.
    
    allegro_hand_bringup = get_package_share_directory('allegro_hand_controllers')
    allegro_hand_desc = get_package_share_directory('allegro_hand_description')
    
    declare_ns_arg = DeclareLaunchArgument(
        'NS',
        default_value='',
        description='Namespace prefix for the hand, e.g. left_hand or right_hand'
    )
    
    # The rest of the original launch arguments remain the same.
    declare_visualize_arg = DeclareLaunchArgument(
        'RVIZ',
        default_value='false',
        description='Flag to enable/disable visualization'
    )

    declare_gui_arg = DeclareLaunchArgument(
        'GUI',
        default_value='false',
        description='Flag to enable/disable GUI'
    )

    declare_hand_arg = DeclareLaunchArgument(
        'HAND',
        default_value='right',
        description='Specify which hand to use: right or left'
    )

    declare_polling_arg = DeclareLaunchArgument(
        'POLLING',
        default_value='true',
        description='true, false for polling the CAN communication'
    )

    declare_can_device_arg = DeclareLaunchArgument(
        'CAN_DEVICE',
        default_value='can1',
        description='Specify CAN port for control multi devices'
    )

    declare_num_arg = DeclareLaunchArgument(
        'NUM',
        default_value='1',
        description='Specify AH num for control multi devices'
    )

    declare_keyboards_arg = DeclareLaunchArgument(
        'KEYBOARDS',
        default_value='true',
        description='Specify AH num for control multi devices'
    )

    declare_controller_arg = DeclareLaunchArgument(
        'CONTROLLER',
        default_value="'grasp'",
        description='Specify which controller to use: grasp, pd, velsat, torque'
    )

    # Function to setup the CAN device.
    def setup_can(context):
        can_port = context.launch_configurations['CAN_DEVICE']
        commands = [
            f"sudo ip link set {can_port} down",
            f"sudo ip link set {can_port} type can bitrate 1000000",
            f"sudo ip link set {can_port} up"
        ]

        for cmd in commands:
            result = os.system(cmd)
            if result != 0:
                print(f"Command failed: {cmd}")
                return []
        print(f'{can_port} setup completed')
        return []

    hand = LaunchConfiguration('HAND')
    # # Build the path to the appropriate URDF based on the hand argument.
    urdf_path = PythonExpression([
        '"', allegro_hand_desc, '/urdf/allegro_hand_description_', hand, '.urdf"'
    ])

    # Include the RViz launch file if visualization is enabled.
    include_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(allegro_hand_bringup, "launch", "allegro_rviz_right.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration('RVIZ'))
    )

    controller = LaunchConfiguration('CONTROLLER')
    # Dynamically build the executable name for the controller node.
    executable_name = PythonExpression([
        "'allegro_node_' + ", controller
    ])

    
    return LaunchDescription([
        declare_visualize_arg,
        declare_polling_arg,
        declare_can_device_arg,
        declare_num_arg,
        declare_hand_arg,
        declare_gui_arg,
        declare_keyboards_arg,
        declare_controller_arg,
        include_rviz,
        OpaqueFunction(function=setup_can),

        Node(
            package='allegro_hand_controllers',
            executable=executable_name,
            output='screen',
            parameters=[
                {'~hand_info/which_hand': LaunchConfiguration('HAND')},
                {'~comm/CAN_CH': LaunchConfiguration('CAN_DEVICE')}
            ],
            arguments=[LaunchConfiguration('POLLING')],
            remappings=[
                ('allegroHand/lib_cmd', PythonExpression(["'allegroHand_", LaunchConfiguration('NUM'), "/lib_cmd'"])),
                ('allegroHand/joint_cmd', PythonExpression(["'allegroHand_", LaunchConfiguration('NUM'), "/joint_cmd'"])),
                ('allegroHand/envelop_torque', PythonExpression(["'allegroHand_", LaunchConfiguration('NUM'), "/envelop_torque'"])),
                ('allegroHand/joint_states', PythonExpression(["'allegroHand_", LaunchConfiguration('NUM'), "/joint_states'"]))
            ]
        ),
        
        Node(
            package='robot_state_publisher',
            output='screen',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
            remappings=[
                ('joint_states', PythonExpression(["'allegroHand_", LaunchConfiguration('NUM'), "/joint_states'"]))
            ]
        )
    ])