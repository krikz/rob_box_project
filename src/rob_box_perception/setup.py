"""Setup script for the rob_box_perception ROS2 package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rob_box_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Установка launch файлов
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Prompts
        (os.path.join('share', package_name, 'prompts'),
            glob('prompts/*.txt')),
    ],
    install_requires=['setuptools', 'pytz'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='kukoreken@rob-box.local',
    description=(
        'Internal Dialogue Agent - Perception and Reflection for Rob Box'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'context_aggregator = '
            'rob_box_perception.context_aggregator_node:main',
            'health_monitor = '
            'rob_box_perception.health_monitor:main',
            'perception_bridge = '
            'rob_box_perception.perception_bridge:main',
            # ADR-0089 Phase 1: AI HAT+ inference node. Stub publishes
            # deterministic test events when no HEF is configured; real
            # HEF loading is gated by the hailo_enabled launch parameter.
            'vision_hailo = '
            'rob_box_perception.vision_hailo_node:main',
            # ADR-0089 Phase 2 (issue #2599 PR-A): face detection node
            # (RetinaFace), отдельный процесс на том же контракте.
            'vision_face = '
            'rob_box_perception.vision_face_node:main',
        ],
    },
)
