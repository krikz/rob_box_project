from setuptools import setup

package_name = 'led_matrix_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/led_matrix_driver_launch.py']),
        ('share/' + package_name + '/config', ['config/led_matrix_driver.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS 2 driver for LED matrix displays with flexible configuration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_matrix_driver = led_matrix_driver.led_matrix_driver:main',
        ],
    },
)