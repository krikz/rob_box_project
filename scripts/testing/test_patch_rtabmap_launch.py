from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def load_module():
    module_path = Path(__file__).resolve().parents[2] / "docker/main/scripts/rtabmap/patch_rtabmap_launch.py"
    spec = spec_from_file_location("patch_rtabmap_launch", module_path)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_patch_injects_required_rtabmap_params_without_map_reanchoring(tmp_path):
    module = load_module()
    launch_file = tmp_path / "rtabmap.launch.py"
    launch_file.write_text(
        "Node(parameters=[{\n" '    "Mem/IncrementalMemory": LaunchConfiguration("Mem/IncrementalMemory"),\n' "}])\n",
        encoding="utf-8",
    )

    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True

    patched = launch_file.read_text(encoding="utf-8")
    assert '"Grid/Sensor": "0"' in patched
    assert '"Optimizer/Strategy": "1"' in patched
    assert '"RGBD/OptimizeFromGraphEnd": "true"' not in patched
    # #4495: Mem/IncrementalMemory ставит сам launch из localization:= (дубль ключа в dict был мёртвым),
    # Odom/* узел rtabmap стирает (CoreWrapper) — в его dict их нет.
    assert '"Mem/IncrementalMemory": "false"' not in patched
    assert '"Odom/ResetCountdown"' not in patched


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
        "Node(\n"
        "    remappings=[\n"
        "        (\"tag_detections\", LaunchConfiguration('tag_topic')),\n"
        "        (\"scan\", LaunchConfiguration('scan_topic')),\n"
        "    ],\n"
        "    parameters=[{\n"
        '    "Mem/IncrementalMemory": LaunchConfiguration("Mem/IncrementalMemory"),\n'
        "    }])\n",
        encoding="utf-8",
    )

    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True

    patched = launch_file.read_text(encoding="utf-8")
    # Legacy remap replaced with the current internal name.
    assert "(\"apriltag/detections\", LaunchConfiguration('tag_topic'))" in patched
    assert '("tag_detections",' not in patched
    # Other remappings must be preserved.
    assert "(\"scan\", LaunchConfiguration('scan_topic'))" in patched


def test_patch_remap_is_idempotent(tmp_path):
    """Re-running the patch on an already-migrated launch must be a no-op
    (returns True and does not duplicate the remap line)."""
    module = load_module()
    launch_file = tmp_path / "rtabmap.launch.py"
    already = (
        "Node(\n"
        "    remappings=[\n"
        "        (\"apriltag/detections\", LaunchConfiguration('tag_topic')),\n"
        "    ],\n"
        "    parameters=[{\n"
        '        "Grid/Sensor": "0",\n'
        '    "Mem/IncrementalMemory": LaunchConfiguration("Mem/IncrementalMemory"),\n'
        "    }])\n"
    )
    launch_file.write_text(already, encoding="utf-8")

    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True
    # No duplicates of the migrated remap.
    patched = launch_file.read_text(encoding="utf-8")
    assert patched.count('("apriltag/detections",') == 1


_ICP_AND_RTABMAP_NODES = (
    "Node(\n"
    "    package='rtabmap_odom', executable='icp_odometry', name=\"icp_odometry\",\n"
    "    parameters=[{\n"
    "        \"frame_id\": LaunchConfiguration('frame_id'),\n"
    "    }]),\n"
    "Node(\n"
    "    package='rtabmap_slam', executable='rtabmap', name=\"rtabmap\",\n"
    "    parameters=[{\n"
    '    "Mem/IncrementalMemory": LaunchConfiguration("Mem/IncrementalMemory"),\n'
    "    }])\n"
)


def _icp_block(text):
    start = text.index("executable='icp_odometry'")
    return text[start : text.index("Node(", start)]


def test_patch_sets_icp_odometry_reset_countdown_explicitly(tmp_path):
    """#4495: явный параметр узла icp_odometry (rtabmap.ini не применялся: HIL 06.11, ros2 param get = 0).

    Без сброса одометрия после одной неудачной регистрации осталась lost на весь прогон (8309 кадров
    из 8946) и стек ехал на колёсах. Параметр должен стоять явным параметром узла icp_odometry.
    """
    module = load_module()
    launch_file = tmp_path / "rtabmap.launch.py"
    launch_file.write_text(_ICP_AND_RTABMAP_NODES, encoding="utf-8")
    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True

    patched = launch_file.read_text(encoding="utf-8")
    assert '"Odom/ResetCountdown": "1"' in _icp_block(patched)
    # Patch 1 по-прежнему в узле rtabmap
    assert '"Grid/Sensor": "0"' in patched[patched.index("executable='rtabmap'") :]


def test_icp_patch_is_idempotent(tmp_path):
    module = load_module()
    launch_file = tmp_path / "rtabmap.launch.py"
    launch_file.write_text(_ICP_AND_RTABMAP_NODES, encoding="utf-8")
    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True
    once = launch_file.read_text(encoding="utf-8")
    assert module.patch() is True
    assert launch_file.read_text(encoding="utf-8") == once
    assert _icp_block(once).count('"Odom/ResetCountdown"') == 1


def test_icp_patch_skips_launch_without_icp_node():
    module = load_module()
    content = 'Node(parameters=[{\n    "Mem/IncrementalMemory": LaunchConfiguration("Mem/IncrementalMemory"),\n}])\n'
    patched, changed = module.patch_icp_odometry(content)
    assert changed is False and patched == content


def test_rendered_params_match_the_dicts(tmp_path):
    """Что в dict модуля, то и в launch: каждая пара ключ–значение ровно в своём узле."""
    module = load_module()
    launch_file = tmp_path / "rtabmap.launch.py"
    launch_file.write_text(_ICP_AND_RTABMAP_NODES, encoding="utf-8")
    module.LAUNCH_FILE = str(launch_file)

    assert module.patch() is True

    patched = launch_file.read_text(encoding="utf-8")
    icp, rtabmap = _icp_block(patched), patched[patched.index("executable='rtabmap'") :]
    for key, value in module.RTABMAP_PARAMS.items():
        assert f'"{key}": "{value}"' in rtabmap and f'"{key}"' not in icp, key
    for key, value in module.ICP_ODOMETRY_PARAMS.items():
        assert f'"{key}": "{value}"' in icp and f'"{key}"' not in rtabmap, key
