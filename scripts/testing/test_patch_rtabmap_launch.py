from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def load_module():
    module_path = Path(__file__).resolve().parents[2] / "docker/main/scripts/patch_rtabmap_launch.py"
    spec = spec_from_file_location("patch_rtabmap_launch", module_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_patch_injects_optimizer_and_localization_params(tmp_path):
    module = load_module()
    launch_file = tmp_path / "rtabmap.launch.py"
    launch_file.write_text(
        'Node(parameters=[{\n'
        '    "Mem/IncrementalMemory": LaunchConfiguration("Mem/IncrementalMemory"),\n'
        '}])\n',
        encoding="utf-8",
    )

    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True

    patched = launch_file.read_text(encoding="utf-8")
    assert '"Optimizer/Strategy": "1"' in patched
    assert '"RGBD/OptimizeFromGraphEnd": "true"' in patched
    assert '"Mem/IncrementalMemory": "false"' in patched
