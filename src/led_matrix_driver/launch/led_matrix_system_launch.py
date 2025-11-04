#!/usr/bin/env python3
"""
LED Matrix System Launch File

Запускает LED matrix драйвер и compositor для Rob Box.
Compositor использует hardcoded конфигурацию, драйвер использует YAML конфиг.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Конфиг драйвера - приоритет: volume > install
    driver_config_volume = '/config/led_matrix/led_matrix_driver.yaml'
    driver_config_install = '/ws/install/led_matrix_driver/share/led_matrix_driver/config/led_matrix_driver.yaml'
    driver_config = driver_config_volume if os.path.exists(driver_config_volume) else driver_config_install

    return LaunchDescription([
        # Запускаем драйвер с конфигом
        Node(
            package='led_matrix_driver',
            executable='led_matrix_driver',
            name='led_matrix_driver',
            output='screen',
            parameters=[driver_config] if os.path.exists(driver_config) else []
        ),

        # Запускаем композитор без параметров (использует hardcoded config)
        # Compositor.py имеет встроенную конфигурацию панелей
        Node(
            package='led_matrix_compositor',
            executable='led_matrix_compositor',
            name='led_matrix_compositor',
            output='screen'
        )
    ])
