from setuptools import setup

package_name = 'rob_box_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Meta-package for launching the entire rob_box system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
