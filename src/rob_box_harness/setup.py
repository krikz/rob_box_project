from setuptools import setup, find_packages

package_name = "rob_box_harness"

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
        # rob_box_harness depends on rob_box_llm because the LLMProvider
        # port is re-exported from there. Symlink / install rob_box_llm
        # into the same env (colcon handles this for ROS2 builds).
        "rob_box_llm>=0.2.1",
        "PyYAML>=6.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.4",
            "pytest-asyncio>=0.21",
            "pytest-cov>=4.0",
        ],
    },
    zip_safe=True,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description=(
        "Harness Framework per ADR-0001: lifecycle, ports, registry, "
        "config loader, and entry point for dialog / persistent / "
        "telegram harnesses."
    ),
    license="MIT",
    tests_require=["pytest", "pytest-asyncio"],
)
