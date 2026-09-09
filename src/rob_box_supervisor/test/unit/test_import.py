"""Smoke-import test for rob_box_supervisor package (AV-2 acceptance).

Verifies the package can be imported as a normal Python module and
exposes ``__version__`` per AV-2 acceptance criterion. The
``supervisor_node`` module is also import-checked so the
``ros2 run rob_box_supervisor supervisor_node`` console-script entry
point is guaranteed to resolve under colcon/pytest.
"""

import importlib

import pytest

import rob_box_supervisor


def test_import_module() -> None:
    """``import rob_box_supervisor`` resolves without ImportError (AV-2)."""
    assert rob_box_supervisor is not None


def test_package_has_version() -> None:
    """``rob_box_supervisor`` exposes ``__version__`` as a non-empty string."""
    assert hasattr(rob_box_supervisor, "__version__")
    assert isinstance(rob_box_supervisor.__version__, str)
    assert rob_box_supervisor.__version__


def test_supervisor_node_module_importable() -> None:
    """``rob_box_supervisor.supervisor_node`` is importable (entry-point target).

    The entry-point module unconditionally imports ``rclpy`` (the module
    body launches a ROS 2 node), so this test only runs on machines where
    ``rclpy`` is available — typically inside the ROS 2 container or after
    ``source /opt/ros/<distro>/setup.bash``. On a plain CI runner without
    ROS 2 installed we ``importorskip`` rather than fail, so the AV-2
    smoke-import suite stays green everywhere.
    """
    pytest.importorskip("rclpy")
    mod = importlib.import_module("rob_box_supervisor.supervisor_node")
    assert mod is not None


@pytest.mark.parametrize("symbol", ["Node", "rclpy"])
def test_rclpy_symbols_available_when_rclpy_present(symbol: str) -> None:
    """The package entry point depends on rclpy but the smoke-import test
    itself must remain runnable even when rclpy is unavailable (CI lint job).

    We attempt the import and only assert presence; absence is a soft skip
    so this test never blocks AV-2 acceptance on a CI runner without ROS2.
    """
    rclpy = pytest.importorskip("rclpy")
    assert hasattr(rclpy, symbol)
