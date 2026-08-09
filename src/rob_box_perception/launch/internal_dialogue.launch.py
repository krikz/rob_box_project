#!/usr/bin/env python3
"""
internal_dialogue.launch.py - Launch perception pipeline (dev).

Phase 6 v2 / W11. Perception package is now a thin sensor bridge — no LLM,
no reflection node, no vision stub. The dialogue shell (src/rob_box_voice)
owns all reasoning via DialogCore + MemoryStore.

Launches:
1. perception_bridge - UART sensor bridge -> /sensors/data + /perception/health
2. context_aggregator - aggregates topics into /perception/context_update
3. health_monitor - watches /rosout, publishes on /voice/sound/trigger

Data flow (current — Phase 6 v2):
  [Sensor board (UART)] -> perception_bridge -> /sensors/data
                          (orphan publisher, Phase 7+ wiring)
                          perception_bridge -> /perception/health
                          (orphan publisher)
  [9 ROS2 topics] ---> context_aggregator -> /perception/context_update
                          (vision, pose, odom, joint_states, rosout,
                           voice/stt/result,
                           voice/dialogue/response,
                           voice/command/intent, voice/command/feedback)
                                     |
                                     v
        /perception/context_update -> mcp_server.py (harness MCP-bridge)
                                     -> DialogCore -> LLM

Data flow (target — Phase 7+, после готовности sensor board firmware):
  [Sensor board (UART)] -> perception_bridge -> /sensors/data
                          \
  [Other ROS2 topics] ------------------------------> /
                                                      \
                                                                          |
                                                                          v
                                              /perception/context_update
                                              -> dialogue_node

Config file is loaded from /config/perception/context_aggregator.yaml if
present, otherwise inline defaults are used.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = '/config/perception/context_aggregator.yaml'
    use_config_file = os.path.exists(config_file)

    if use_config_file:
        context_aggregator_params = [config_file]
    else:
        context_aggregator_params = [{
            'publish_rate': 2.0,
            'memory_window': 60,
            'timezone': 'Europe/Moscow',
        }]

    return LaunchDescription([
        # Perception Bridge - UART sensor reader -> /sensors/data
        Node(
            package='rob_box_perception',
            executable='perception_bridge',
            name='perception_bridge',
            output='screen',
            parameters=[{
                'sensor_read_period': 0.1,   # 10 Hz
                'health_period': 1.0,        # 1 Hz
            }],
        ),

        # Context Aggregator - collects topics, publishes
        # /perception/context_update
        Node(
            package='rob_box_perception',
            executable='context_aggregator',
            name='context_aggregator',
            output='screen',
            parameters=context_aggregator_params,
        ),

        # Health Monitor - watches /rosout, publishes status sounds
        Node(
            package='rob_box_perception',
            executable='health_monitor',
            name='health_monitor',
            output='screen',
            parameters=[{
                'check_rate': 1.0,
                'error_window': 30,
                'degraded_threshold': 5,
                'critical_threshold': 10,
                'enable_sounds': True,
            }],
        ),
    ])
