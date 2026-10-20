from setuptools import setup, find_packages

package_name = "rob_box_dialog"

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
    description="Pure-Python dialog v2 core (ADR-0150): knowledge table, word matching",
    license="MIT",
    tests_require=["pytest"],
)
