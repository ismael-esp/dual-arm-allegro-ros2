# launch where all the parameters are not read from a yaml file
import os
from launch.substitutions import PythonExpression
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction,  LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ## Launch the allegro_hand_des launch file
    # Path to the allegro_hand_des package
    allegro_hand_des_pkg = FindPackageShare('allegro_hand_des')
    # Path to display_allegro.launch.py
    display_allegro_launch_file = PathJoinSubstitution([allegro_hand_des_pkg,'launch', 'display_allegro.launch.py'])

    # Launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "yaml_file",
            # default_value="src/allegro_hand/conf/configuration_files/V4R284.yaml",
            # default_value="src/allegro_hand/conf/configuration_files/SAH020AL011.yaml",
            # default_value="SAH020BR014.yaml",
            default_value="/home/dev/ros2_ws/src/allegro_hand_cust/allegro_hand/conf/configuration_files/V4P231.yaml",
            description="Allegro hand configuration yaml file. Can be the directory or the yaml file name (with or without the extension .yaml).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "path_pcan",
            default_value="",
            description="pcan path",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "channel",
            default_value = '0',
            description="Communications channel",
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
            default_value='Allegro_namespace',
            description="Prefix of the link names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tip",
            default_value='default',
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_hand",
            default_value='false',
            description="Fake hand argument.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value='true',
            description="Argument to launch or not the rviz: true or false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=PathJoinSubstitution([allegro_hand_des_pkg, "rviz", "display_allegro.rviz"]),
            description="Path of the rviz configuration ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slave",
            default_value='false',
            description="Slave: true or false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "node_namespace",
            default_value='Allegro_namespace',
            description="Node namespace",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_controller",
            default_value="GravityCompensator",
            description="Initial allegro hand controller.",
            choices=[
                "GravityCompensator",
                "Fake",
                "Torque",
                "JointPositionPID",
                "JointPositionPD",
            ],
        )
    )

    conditional_link_prefix = [LaunchConfiguration('link_prefix'), TextSubstitution(text='/')] if LaunchConfiguration('link_prefix') != '' else TextSubstitution(text='')
    conditional_joint_prefix = [LaunchConfiguration('joint_prefix'), TextSubstitution(text='/')] if LaunchConfiguration('joint_prefix') != '' else TextSubstitution(text='')

    allegro_node = Node(
        package ='allegro_hand_ros',
        executable ='allegro_node',
        name = LaunchConfiguration('hand'),
        namespace =LaunchConfiguration('node_namespace'),
        output ='screen',
        parameters = [{
            'yaml_file': LaunchConfiguration('yaml_file'),
            'path_pcan': LaunchConfiguration('path_pcan'),
            'channel': LaunchConfiguration('channel'),
            'link_prefix': conditional_link_prefix,
            'fake_hand': LaunchConfiguration('fake_hand'),
            'slave': LaunchConfiguration('slave'),
            'initial_controller': LaunchConfiguration('initial_controller'),
            'hand': LaunchConfiguration('hand'),
            'tip': LaunchConfiguration('tip'),
            'joint_prefix': LaunchConfiguration('joint_prefix'),
            }],
    )

    # Display in rviz using the allegro_hand_des package
    display_allegro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_allegro_launch_file),
        condition=IfCondition(LaunchConfiguration("rviz")),
        launch_arguments={
            'hand': LaunchConfiguration('hand'),
            'tip': LaunchConfiguration('tip'),
            'joint_prefix': LaunchConfiguration('joint_prefix'),
            'link_prefix': conditional_link_prefix,
            'namespace_jointstates': LaunchConfiguration('node_namespace'),
            'rviz_config_file_path': LaunchConfiguration('rvizconfig'),
        }.items(),
    )

    nodes_to_start = [
        allegro_node,
        # display_allegro_launch,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
