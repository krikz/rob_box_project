from setuptools import setup, find_packages

package_name = "rob_box_llm"

setup(
    name=package_name,
    version="0.2.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "openai>=1.0", "httpx>=0.27"],
    extras_require={
        # Dev-time test dependencies. ``pip install -e .[dev]`` is the
        # recommended way to bootstrap a contributor's venv — keeps
        # runtime deps separate from the testing stack so production
        # images don't ship with pytest/respx installed.
        # openai<3.0 pinned here because respx doesn't support httpx2 yet
        # (https://github.com/lundberg/respx/pull/317). Production code
        # is NOT affected — it uses openai>=1.0 without upper bound.
        "dev": [
            "openai>=1.0,<3.0",
            "pytest>=7.4",
            "pytest-asyncio>=0.21",
            "pytest-cov>=4.0",
            "respx>=0.21",
            "faker>=18.0",
        ],
    },
    zip_safe=True,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description="Shared LLM provider abstraction for rob_box nodes (ADR-0001 P0 foundation)",
    license="MIT",
    tests_require=["pytest", "pytest-asyncio"],
)
