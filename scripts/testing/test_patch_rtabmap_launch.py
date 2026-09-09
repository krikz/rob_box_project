from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def load_module():
    module_path = Path(__file__).resolve().parents[2] / "docker/main/scripts/patch_rtabmap_launch.py"
    spec = spec_from_file_location("patch_rtabmap_launch", module_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_patch_injects_required_rtabmap_params_without_map_reanchoring(tmp_path):
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
    assert '"Grid/Sensor": "0"' in patched
    assert '"Optimizer/Strategy": "1"' in patched
    assert '"Mem/IncrementalMemory": "false"' in patched
    assert '"RGBD/OptimizeFromGraphEnd": "true"' not in patched


def test_patch_migrates_legacy_tag_detections_remap(tmp_path):
    """Issue #670: rtabmap warns that \"tag_detections\" is deprecated.

    Patch must remap the CURRENT internal subscription
    (\"apriltag/detections\") instead of the legacy one (\"tag_detections\"),
    so the data flows through the non-deprecated subscription and the warning
    disappears.
    """
    module = load_module()
    launch_file = tmp_path / "rtabmap.launch.py"
    launch_file.write_text(
        'Node(\n'
        '    remappings=[\n'
        '        ("tag_detections", LaunchConfiguration(\'tag_topic\')),\n'
        '        ("scan", LaunchConfiguration(\'scan_topic\')),\n'
        '    ],\n'
        '    parameters=[{\n'
        '    "Mem/IncrementalMemory": LaunchConfiguration("Mem/IncrementalMemory"),\n'
        '    }])\n',
        encoding="utf-8",
    )

    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True

    patched = launch_file.read_text(encoding="utf-8")
    # Legacy remap replaced with the current internal name.
    assert '("apriltag/detections", LaunchConfiguration(\'tag_topic\'))' in patched
    assert '("tag_detections",' not in patched
    # Other remappings must be preserved.
    assert '("scan", LaunchConfiguration(\'scan_topic\'))' in patched


def test_patch_remap_is_idempotent(tmp_path):
    """Re-running the patch on an already-migrated launch must be a no-op
    (returns True and does not duplicate the remap line)."""
    module = load_module()
    launch_file = tmp_path / "rtabmap.launch.py"
    already = (
        'Node(\n'
        '    remappings=[\n'
        '        ("apriltag/detections", LaunchConfiguration(\'tag_topic\')),\n'
        '    ],\n'
        '    parameters=[{\n'
        '        "Grid/Sensor": "0",\n'
        '    "Mem/IncrementalMemory": LaunchConfiguration("Mem/IncrementalMemory"),\n'
        '    }])\n'
    )
    launch_file.write_text(already, encoding="utf-8")

    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True
    # No duplicates of the migrated remap.
    patched = launch_file.read_text(encoding="utf-8")
    assert patched.count('("apriltag/detections",') == 1
