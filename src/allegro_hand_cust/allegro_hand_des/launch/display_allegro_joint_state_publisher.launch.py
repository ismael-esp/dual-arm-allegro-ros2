from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
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
            default_value='""',
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
            default_value='left',
            description="Right or left.",
        )
    )
    
    # Pass the rviz file name. The file should be located inside the allegro_hand_des package
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file_name",
            default_value= 'display_allegro.rviz',
            description="File name of the rviz configuration file",
        )
    )
    # # Alternative: define an argument with the whole path towards the rviz configuration file. (If so comment line 80 and uncomment 81)
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "rviz_config_file_path",
    #         default_value=PathJoinSubstitution([description_package, "rviz", "display_allegro.rviz"]),
    #         description="Path of the rviz configuration file",
    #     )
    # )

    description_file = LaunchConfiguration("description_file")
    joint_prefix = LaunchConfiguration("joint_prefix")
    link_prefix = LaunchConfiguration("link_prefix")
    tip = LaunchConfiguration("tip")
    hand = LaunchConfiguration("hand")
    rviz_config_file_name = LaunchConfiguration("rviz_config_file_name")
    
    # Path of the rviz configuration file 
    rviz_config_file_path = PathJoinSubstitution([description_package, "rviz", rviz_config_file_name])
    # rviz_config_file_path = LaunchConfiguration("rviz_config_file_path")
    
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
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file_path],
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="log",
    )

    nodes_to_start = [
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)