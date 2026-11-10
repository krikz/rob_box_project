from setuptools import find_packages, setup

package_name = "rob_box_dialog_host"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=False,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description="Dialog turn host glue without ROS (ADR-0167 Sh1g): command bridge, dialog ports, voice memory",
    license="MIT",
    tests_require=["pytest"],
)
