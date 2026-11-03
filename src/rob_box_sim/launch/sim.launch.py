"""РОББОКС в Gazebo Fortress: мир, робот, мост ros_gz, контроллеры ros2_control.

Шаг 1 плана docs/research/2026-11-04-gazebo-sim-navigation.md §6.

Что поднимается:
  * ``ign gazebo`` без окна (``-s``), с ``--headless-rendering`` для gpu_lidar;
  * robot_state_publisher с URDF ``rob_box.xacro sim:=true`` и use_sim_time;
  * spawn робота (``ros_gz_sim create``) из топика /robot_description;
  * ros_gz_bridge: /clock, /scan, /camera/imu/data, /sim/ground_truth/odom;
  * спаунеры joint_state_broadcaster и diff_drive_controller — controller_manager
    живёт внутри плагина gz_ros2_control и читает ТОТ ЖЕ controller_manager.yaml,
    что контейнер ros2-control на роботе;
  * relay /cmd_vel → /diff_drive_controller/cmd_vel_unstamped. На роботе это
    делает twist_mux (docker/main/scripts/twist_mux/start_twist_mux.sh:72);
    в сим twist_mux с боевым конфигом приходит на шаге 2, тогда relay уходит.

Пример:
  ros2 launch rob_box_sim sim.launch.py
  ros2 launch rob_box_sim sim.launch.py headless_rendering:=false   # программный рендер
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("rob_box_description")
    sim_share = get_package_share_directory("rob_box_sim")

    world = LaunchConfiguration("world")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    headless_rendering = LaunchConfiguration("headless_rendering")

    # package://rob_box_description/meshes/... в URDF: Gazebo ищет модель
    # rob_box_description в каталогах ресурсов, поэтому кладём туда родителя share.
    resource_path = os.pathsep.join(
        p
        for p in (
            os.path.dirname(description_share),
            os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
        )
        if p
    )

    # gz_ros2_control-system лежит в /opt/ros/humble/lib: Fortress ищет системные
    # плагины только в IGN_GAZEBO_SYSTEM_PLUGIN_PATH (иначе «couldn't find shared
    # library»), поэтому отдаём ему LD_LIBRARY_PATH из окружения ROS.
    plugin_path = os.pathsep.join(
        p
        for p in (
            os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
            os.environ.get("LD_LIBRARY_PATH", ""),
        )
        if p
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(description_share, "urdf", "rob_box.xacro"),
                " sim:=true controllers_yaml:=",
                controllers_yaml,
            ]
        ),
        value_type=str,
    )

    gz_headless_rendering = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", "--headless-rendering", "-v", "3", world],
        output="screen",
        condition=IfCondition(headless_rendering),
    )
    gz_plain = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", "-v", "3", world],
        output="screen",
        condition=UnlessCondition(headless_rendering),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "rob_box",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", "0.05",
            "-Y", LaunchConfiguration("yaw"),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(sim_share, "config", "bridge.yaml"),
                "use_sim_time": True,
            }
        ],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            parameters=[{"use_sim_time": True}],
        )

    jsb = spawner("joint_state_broadcaster")
    ddc = spawner("diff_drive_controller")

    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="sim_cmd_vel_relay",
        output="screen",
        arguments=["/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([sim_share, "worlds", "polygon.sdf"]),
            description="SDF-мир",
        ),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value="/config/controllers/controller_manager.yaml",
            description="controller_manager.yaml робота (docker/main/config/controllers/)",
        ),
        DeclareLaunchArgument(
            "headless_rendering",
            default_value="true",
            description="--headless-rendering (EGL, нужен /dev/dri); false — без него",
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", plugin_path),
        gz_headless_rendering,
        gz_plain,
        robot_state_publisher,
        bridge,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[ddc])),
        cmd_vel_relay,
    ])
