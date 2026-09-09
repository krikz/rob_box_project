from setuptools import setup, find_packages

package_name = "rob_box_harness"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    # confirmation_policy.yaml лежит ВНУТРИ пакета
    # (rob_box_harness/core/data/) и читается через
    # importlib.resources.files("rob_box_harness.core.data") — см.
    # core/confirmation_policy.py:426. Значит доставка — package_data,
    # а не data_files: find_packages() собирает только *.py, а
    # ament-share из importlib.resources не виден. Прод-образ
    # собирается БЕЗ --symlink-install
    # (docker/vision/voice_assistant/Dockerfile:239), то есть пакет
    # реально устанавливается — без этой строки YAML в
    # install-дерево не попадает. Четвёртый случай этого
    # класса в репозитории: prompts/ (6bb0f999), wake_words.yaml
    # (#2022), slice_policy.yaml (#1998), теперь этот.
    package_data={
        "rob_box_harness.core.data": ["*.yaml"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "rob_box_core>=0.1.0",
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
    # Пакет читает свои ресурсы с диска (confirmation_policy.yaml) — из
    # zip-egg это работает не на всех путях установки.
    zip_safe=False,
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
