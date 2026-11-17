from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rob_box_voice'


def existing(*patterns):
    """Файлы по шаблонам glob — только существующие.

    CI собирает с ``--symlink-install`` и восстанавливает build/ из кэша: после удаления файла из src (PR-15 15p,
    #3711: yaml-конфиг ноды v1) в build/ остаётся висячая ссылка, glob её находит, и setup.py падал
    «can't copy …: doesn't exist or not a regular file».
    """
    return [path for pattern in patterns for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']) + ['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            existing('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            existing('config/*.yaml', 'config/*.json')),
        # Пресеты стиля речи (voice_presets.yaml + voice_presets/*.txt) — в профиле робота docker/sound/config
        # (ADR-0167 Ш1а-2), в share пакета их больше нет.
        # Prompts
        (os.path.join('share', package_name, 'prompts'),
            existing('prompts/*.txt', 'prompts/*.yaml')),
        # Service definitions
        (os.path.join('share', package_name, 'srv'),
            existing('srv/*.srv')),
    ],
    install_requires=[
        'setuptools',
        # ADR-0167 Ш3: ядро диалога (rob_box_core, rob_box_harness и другие модули ядра)
        # ставится pip-пакетом rob-box-dialog-core из krikz/rob_box_dialog_core по тегу,
        # а не colcon'ом из src/ (см. docker/sound/voice_assistant/Dockerfile).
        'rob-box-dialog-core>=0.7.0',
    ],
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
            # Диалог: хост v2 (ADR-0150 PR-8 #3702); нода v1 удалена в PR-15 (#3711, часть 15p)
            'dialog_node = rob_box_voice.dialog_node:main',
            'tts_node = rob_box_voice.tts_node:main',
            # Phase 3: STT (Speech-to-Text)
            'stt_node = rob_box_voice.stt_node:main',
            # Phase 4: Sound Effects
            'sound_node = rob_box_voice.sound_node:main',
            # Phase 5: Command Recognition
            'command_node = rob_box_voice.command_node:main',
            'speaker_id_node = rob_box_voice.speaker_id_node:main',
            # Utilities
            'configure_respeaker_aec = scripts.configure_respeaker_aec:main',
        ],
    },
)
