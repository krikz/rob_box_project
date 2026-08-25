from setuptools import find_packages, setup

package_name = "rob_box_quest"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description="WebXR / Meta Quest telepresence service for Rob Box",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Phase 1.3 — populated in Phase 1.3 PR.
            # 'quest_node = rob_box_quest.quest_node:main',
        ],
    },
)
