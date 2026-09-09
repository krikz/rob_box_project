from setuptools import setup, find_packages

package_name = "rob_box_supervisor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # issue #1988 — промпт оператора (ТАРС) + фрагменты срезов
        # operator.speech / operator.control. Читаются через ament share
        # (supervisor_node._resolve_prompts_dir).
        (
            "share/" + package_name + "/prompts",
            ["prompts/operator_system_prompt.txt"],
        ),
        (
            "share/" + package_name + "/prompts/skills",
            [
                "prompts/skills/operator.speech.txt",
                "prompts/skills/operator.control.txt",
            ],
        ),
    ],
    install_requires=["setuptools", "msgpack>=1.0"],
    zip_safe=True,
    maintainer="krikz",
    maintainer_email="kukoreken@rob-box.local",
    description=(
        "Avatar Supervisor per ADR-0028. Phase 1: monitor-only "
        "publishes /avatar/state. Full FSM/lock-manager is Phase 2 (AV-6)."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "supervisor_node = rob_box_supervisor.supervisor_node:main",
            # ADR-0051 §2.2 / issue #1987 — арбитр floor/FSM без LLM.
            "avatar_arbiter = rob_box_supervisor.arbiter_node:main",
        ],
    },
)
