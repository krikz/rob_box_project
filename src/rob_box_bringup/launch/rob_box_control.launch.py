#!/usr/bin/env python3
"""
rob_box_control.launch.py

Launch file для запуска ros2_control с VESC Nexus hardware interface
и diff_drive_controller для Rob Box робота.

Компоненты:
- robot_state_publisher: публикует TF дерево из URDF
- ros2_control_node: запускает VESC hardware interface
- diff_drive_controller: управление движением
- joint_state_broadcaster: публикация состояния джоинтов
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ═══════════════════════════════════════════════════════════════
    # Launch Arguments
    # ═══════════════════════════════════════════════════════════════
    
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="diff_drive_controller",
            description="Robot controller to start.",
        )
    )

    # ═══════════════════════════════════════════════════════════════
    # Configuration
    # ═══════════════════════════════════════════════════════════════
    
    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    robot_controller = LaunchConfiguration("robot_controller")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rob_box_description"),
                    "urdf",
                    "rob_box_main.xacro",
                ]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Controller manager configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rob_box_bringup"),
            "config",
            "robot_controller.yaml",
        ]
    )

    # ═══════════════════════════════════════════════════════════════
    # Nodes
    # ═══════════════════════════════════════════════════════════════

    # ros2_control node (controller_manager)
    # Запускает VESC hardware interface
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {"use_sim_time": use_sim_time}],
        output="both",
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    # robot_state_publisher
    # Публикует TF дерево робота из URDF
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # RViz2 (опционально)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rob_box_description"), "rviz", "rob_box.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ═══════════════════════════════════════════════════════════════
    # Controller Spawners
    # ═══════════════════════════════════════════════════════════════

    # Joint State Broadcaster
    # Должен запуститься первым
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Diff Drive Controller
    # Запускается после joint_state_broadcaster
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # ═══════════════════════════════════════════════════════════════
    # Build Launch Description
    # ═══════════════════════════════════════════════════════════════

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
