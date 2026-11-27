"""Regression checks for the Vision Pi LED launch and container healthcheck."""

import runpy
import shlex
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]


class Node(SimpleNamespace):
    @property
    def node_package(self) -> str:
        return self.package


@pytest.fixture
def launch_entities(monkeypatch: pytest.MonkeyPatch) -> list:
    """Load the launch file on unit-test hosts without ROS installed."""
    modules = {
        "ament_index_python.packages": SimpleNamespace(get_package_share_directory=lambda package: package),
        "launch": SimpleNamespace(LaunchContext=SimpleNamespace, LaunchDescription=list),
        "launch.actions": SimpleNamespace(
            EmitEvent=SimpleNamespace,
            IncludeLaunchDescription=lambda source: SimpleNamespace(source=source),
            RegisterEventHandler=lambda handler: SimpleNamespace(handler=handler),
        ),
        "launch.event_handlers": SimpleNamespace(OnProcessExit=SimpleNamespace),
        "launch.events": SimpleNamespace(Shutdown=SimpleNamespace),
        "launch.events.process": SimpleNamespace(ProcessExited=SimpleNamespace),
        "launch.launch_description_sources": SimpleNamespace(PythonLaunchDescriptionSource=str),
        "launch_ros.actions": SimpleNamespace(Node=Node),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    launch_file = ROOT / "src/rob_box_animations/launch/face_led_matrix.launch.py"
    return runpy.run_path(str(launch_file))["generate_launch_description"]()


@pytest.mark.parametrize("package", ["led_matrix_driver", "led_matrix_compositor"])
def test_oom_kill_of_a_required_node_shuts_down_led_launch(launch_entities: list, package: str) -> None:
    handlers = [entity.handler for entity in launch_entities if hasattr(entity, "handler")]
    assert len(handlers) == 1

    event = SimpleNamespace(action=Node(package=package), process_name=f"{package}-1", returncode=-9)
    actions = handlers[0].on_exit(event, SimpleNamespace(is_shutdown=False))

    assert len(actions) == 1
    assert package in actions[0].event.reason
    assert "-9" in actions[0].event.reason


def test_face_exit_and_manual_shutdown_do_not_request_another_shutdown(launch_entities: list) -> None:
    handlers = [entity.handler for entity in launch_entities if hasattr(entity, "handler")]
    assert len(handlers) == 1

    face = next(entity for entity in launch_entities if isinstance(entity, Node))
    assert face.respawn
    face_exit = SimpleNamespace(action=face, process_name="face_node.py-3", returncode=-9)
    assert not handlers[0].on_exit(face_exit, SimpleNamespace(is_shutdown=False))

    matrix = Node(package="led_matrix_driver")
    matrix_exit = SimpleNamespace(action=matrix, process_name="led_matrix_driver-1", returncode=-15)
    assert not handlers[0].on_exit(matrix_exit, SimpleNamespace(is_shutdown=True))


def test_healthcheck_requires_driver_and_compositor_processes() -> None:
    compose = yaml.safe_load((ROOT / "docker/vision/docker-compose.yaml").read_text(encoding="utf-8"))
    led_matrix = compose["services"]["led-matrix"]
    assert led_matrix["restart"] == "unless-stopped"
    assert "test" not in led_matrix["healthcheck"]

    dockerfile = (ROOT / "docker/vision/led_matrix/Dockerfile").read_text(encoding="utf-8")
    healthcheck = dockerfile.split("HEALTHCHECK ", 1)[1]
    command = healthcheck.split("CMD ", 1)[1].splitlines()[0]
    checks = [shlex.split(check.split(">", 1)[0]) for check in command.split("&&")]
    assert checks == [["pgrep", "-x", "led_matrix_driv"], ["pgrep", "-x", "led_matrix_comp"]]
