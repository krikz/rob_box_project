from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rob_box_mcp_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rob Box Team',
    maintainer_email='your_email@example.com',
    description='MCP-like tool system for LLM integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcp_server = rob_box_mcp_tools.mcp_server:main',
        ],
    },
)
