"""РОББОКС в Gazebo Fortress: мир, spawn робота, мост ros_gz, контроллеры ros2_control.

План: docs/research/2026-11-04-gazebo-sim-navigation.md §6 (шаги 1–2).

Что поднимается:
  * ``ign gazebo`` без окна (``-s``), с ``--headless-rendering`` для gpu_lidar;
  * spawn робота (``ros_gz_sim create``) из топика /robot_description;
  * ros_gz_bridge: /clock, /scan, /camera/imu/data, /sim/ground_truth/odom;
  * спаунеры joint_state_broadcaster и diff_drive_controller — controller_manager
    живёт внутри плагина gz_ros2_control и читает ТОТ ЖЕ controller_manager.yaml,
    что контейнер ros2-control на роботе.

Чего здесь нет (шаг 2): robot_state_publisher и twist_mux запускаются отдельными
сервисами docker/sim/compose.yaml БОЕВЫМИ скриптами docker/main/scripts/**
(``URDF_SIM=true``, ``USE_SIM_TIME=true``). Поэтому /robot_description публикует
сервис robot-state-publisher, а /cmd_vel → /diff_drive_controller/cmd_vel_unstamped
ведёт боевой twist_mux. Временный relay шага 1 удалён.

Пример (внутри compose-профиля):
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("rob_box_description")
    sim_share = get_package_share_directory("rob_box_sim")

    world = LaunchConfiguration("world")
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([sim_share, "worlds", "polygon.sdf"]),
            description="SDF-мир",
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
        bridge,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[ddc])),
    ])
