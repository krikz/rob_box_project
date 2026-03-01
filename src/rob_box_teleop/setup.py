from setuptools import setup
import os
from glob import glob

package_name = 'rob_box_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rob Box Team',
    maintainer_email='krikz@rob-box.ru',
    description='Teleoperation package for Rob Box robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_control_node = rob_box_teleop.joystick_control_node:main',
        ],
    },
)
