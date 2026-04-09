from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RTABMAP_INI = REPO_ROOT / "docker/main/config/rtabmap/rtabmap.ini"


def load_module():
    module_path = REPO_ROOT / "docker/main/scripts/patch_rtabmap_launch.py"
    spec = spec_from_file_location("patch_rtabmap_launch", module_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_rtabmap_localization_tuning_avoids_proximity_links():
    ini_text = RTABMAP_INI.read_text(encoding="utf-8")

    assert "NeighborLinkRefining=false" in ini_text
    assert "ProximityBySpace=false" in ini_text
    assert "NeighborLinkRefining=true" not in ini_text
    assert "ProximityBySpace=true" not in ini_text


def test_launch_patch_does_not_force_proximity_links(tmp_path):
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
    assert '"RGBD/ProximityBySpace": "true"' not in patched
    assert '"RGBD/NeighborLinkRefining": "true"' not in patched