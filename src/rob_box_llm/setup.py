from setuptools import setup, find_packages

package_name = "rob_box_llm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "openai>=1.0", "httpx>=0.27"],
    zip_safe=True,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description="Shared LLM provider abstraction for rob_box nodes (ADR-0001 P0 foundation)",
    license="MIT",
    tests_require=["pytest", "pytest-asyncio"],
)
