import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Path of the Allegro hand description package with robot URDF/XACRO files.
    description_package = FindPackageShare('allegro_hand_des')
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="allegro_hand_des.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "link_prefix",
            default_value='',
            description="Prefix of the link names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tip",
            default_value="''",
            description="Type of tip",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand",
            default_value='right',
            description="Right or left.",
            choices=[
                "right",
                "left",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace_jointstates",
            default_value="''",
            description="Namespace of the joint_states. We will remap them",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "rviz_config_file_path",
    #         default_value=PathJoinSubstitution([description_package, "rviz", "display_allegro.rviz"]),
    #         description="Path of the rviz configuration file",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_fixed_frame",
            default_value="base_link",
            description="Fixed frame name of the Rviz",
        )
    )

    description_file = LaunchConfiguration("description_file")
    joint_prefix = LaunchConfiguration("joint_prefix")
    link_prefix = LaunchConfiguration("link_prefix")
    tip = LaunchConfiguration("tip")
    hand = LaunchConfiguration("hand")
    namespace_jointstates = LaunchConfiguration("namespace_jointstates")


    # Conditional assignment of rviz fixed frame link_prefix/base_link or base_link
    conditional_rviz_fixed_frame = [link_prefix, LaunchConfiguration('rviz_fixed_frame')] if LaunchConfiguration('link_prefix') != '' else LaunchConfiguration('rviz_fixed_frame')

    # Create paths to the rviz files
    rviz_file_default = PathJoinSubstitution([description_package, "rviz", "display_allegro.rviz"])
    rviz_file_weiss = PathJoinSubstitution([description_package, "rviz", "display_allegro_weiss.rviz"])
    # rviz_config_file_path = rviz_file_default if tip != "weiss" else rviz_file_weiss 
    
    # Path of the urdf file 
    urdf_path = PathJoinSubstitution([description_package, "urdf", description_file])

    # Robot description and passing launch arguments to the xacro.
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path, ' hand:=', hand, ' tip:=', tip, ' joint_prefix:=', joint_prefix, ' link_prefix:=', link_prefix]), value_type=str)
    robot_description = {"robot_description": robot_description_content}


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ('/joint_states', ['/', LaunchConfiguration("namespace_jointstates"), '/joint_states'])
        ] 
    )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file_path, "-f", conditional_rviz_fixed_frame],
    #     # arguments=["-d", LaunchConfiguration("rviz_file"), "-f", conditional_rviz_fixed_frame],
    # )
     # Define the conditional logic for the RViz config file
    rviz_node_default = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file_default, "-f", conditional_rviz_fixed_frame],
        condition=IfCondition(
            PythonExpression(
                ["'", tip, "' != 'weiss'"]
            )
        )

    )

    rviz_node_weiss = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file_weiss, "-f", conditional_rviz_fixed_frame],
        condition=IfCondition(
            PythonExpression(
                ["'", tip, "' == 'weiss' "]
            )
        ),
    )
    
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     output="log",
    # )

    nodes_to_start = [
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # rviz_node,
        rviz_node_weiss,
        rviz_node_default
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
