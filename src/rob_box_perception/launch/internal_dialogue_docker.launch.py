#!/usr/bin/env python3
"""
internal_dialogue_docker.launch.py - Launch perception pipeline inside Docker.

Phase 6 v2 / W11. Perception package is now a thin sensor bridge — no LLM,
no reflection node, no vision stub, no startup greeting. The dialogue shell
(src/rob_box_voice) owns all reasoning.

Launches:
1. perception_bridge - UART sensor bridge -> /sensors/data + /perception/health
2. context_aggregator - aggregates subscribed topics into /perception/context_update
3. health_monitor - watches /rosout, publishes status on /voice/sound/trigger

UART port and baud come from SENSOR_UART_PORT / SENSOR_UART_BAUD env vars
(falling back to /dev/ttyAMA0 @ 115200). Context aggregator params are
loaded from /config/perception/context_aggregator.yaml (mounted via docker)
with optional overrides from CONTEXT_PUBLISH_RATE / MEMORY_WINDOW / TIMEZONE.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Context Aggregator config file path (mounted via docker)
    config_file = '/config/perception/context_aggregator.yaml'

    # Environment variable overrides for context_aggregator
    env_overrides = {}
    if 'CONTEXT_PUBLISH_RATE' in os.environ:
        env_overrides['publish_rate'] = float(os.environ['CONTEXT_PUBLISH_RATE'])
    if 'MEMORY_WINDOW' in os.environ:
        env_overrides['memory_window'] = int(os.environ['MEMORY_WINDOW'])
    if 'TIMEZONE' in os.environ:
        env_overrides['timezone'] = os.environ['TIMEZONE']

    return LaunchDescription([
        # Perception Bridge - UART sensor reader -> /sensors/data
        Node(
            package='rob_box_perception',
            executable='perception_bridge',
            name='perception_bridge',
            output='screen',
            parameters=[{
                'sensor_read_period': 0.1,
                'health_period': 1.0,
            }],
        ),

        # Context Aggregator - publishes aggregated /perception/context_update
        Node(
            package='rob_box_perception',
            executable='context_aggregator',
            name='context_aggregator',
            output='screen',
            parameters=[
                config_file,
                env_overrides,
            ],
        ),

        # Health Monitor - watches /rosout, publishes status
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
