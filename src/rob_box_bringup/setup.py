import glob
import os
from setuptools import setup

package_name = 'rob_box_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Добавляем launch-файлы
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description=(
        'Meta-package for launching the entire rob_box system. '
        'This package is responsible for organizing and launching all components of the rob_box project.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # Entry points can be added here if needed
    },
)