from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rob_box_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')),
        # Prompts
        (os.path.join('share', package_name, 'prompts'),
            glob('prompts/*.txt') + glob('prompts/*.yaml')),
        # Service definitions
        (os.path.join('share', package_name, 'srv'),
            glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krikz',
    maintainer_email='kukoreken@rob-box.local',
    description='AI Voice Assistant for ROBBOX autonomous rover with ReSpeaker Mic Array v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_node = rob_box_voice.audio_node:main',
            'led_node = rob_box_voice.led_node:main',
            # Реализованные ноды (Phase 2)
            'dialogue_node = rob_box_voice.dialogue_node:main',
            'tts_node = rob_box_voice.tts_node:main',
            # TODO: Реализовать в Phase 3-6
            # 'stt_node = rob_box_voice.stt_node:main',
            # 'sound_node = rob_box_voice.sound_node:main',
            # 'command_node = rob_box_voice.command_node:main',
        ],
    },
)
