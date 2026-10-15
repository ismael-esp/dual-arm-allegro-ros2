from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    package_name = 'my_dual_robot_cell_control'
    package_share = FindPackageShare(package_name)

    # 1. Main Driver Launch (UR + Allegro + Cameras)
    start_torso_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'start_torso_config.launch.py'])
        )
    )

    # 2. Hand Service Commanders
    hand_commander_left = Node(
        package=package_name,
        executable='hand_service_commander_left',
        name='hand_service_commander_left',
        output='screen'
    )

    hand_commander_right = Node(
        package=package_name,
        executable='hand_service_commander_right',
        name='hand_service_commander_right',
        output='screen'
    )

    # 3. Dual Arm Controller (new_driver_real)
    new_driver_real = Node(
        package=package_name,
        executable='new_driver_real',
        name='dual_arm_controller',
        output='screen'
    )

    # 4. Joint Republishers (Bridges from Simulation/Isaac)
    jointrepub_left = Node(
        package=package_name,
        executable='jointrepub_left',
        name='joint_republisher_left',
        output='screen'
    )

    jointrepub_right = Node(
        package=package_name,
        executable='jointrepub_right',
        name='joint_republisher_right',
        output='screen'
    )

    return LaunchDescription([
        start_torso_config_launch,
        # We can add small delays if needed to ensure previous nodes are ready
        TimerAction(period=3.0, actions=[
            hand_commander_left,
            hand_commander_right,
            new_driver_real,
            jointrepub_left,
            jointrepub_right
        ])
    ])
