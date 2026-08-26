from setuptools import setup

package_name = "rob_box_quest"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rob Box Team",
    maintainer_email="krikz@rob-box.ru",
    description="WebXR / Meta Quest telepresence service for Rob Box (Phase 2+)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "quest_node = rob_box_quest.quest_node:main",
            "quest_perf_logger = rob_box_quest.perf.logger_node:main",
        ],
    },
)