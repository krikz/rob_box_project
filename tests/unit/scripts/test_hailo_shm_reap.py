"""Регресс-тесты для docker/vision/scripts/vision-hailo/hailo_shm_reap.py.

Issue #4277: /dev/shm хоста Vision Pi забит буферами HailoRT прошлых
запусков (38 тыс. файлов, 3.9 ГБ из 3.9 ГБ), vision-hailo и vision-face
падают SIGBUS (exit code -7) на первом кадре. Уборщик при старте
контейнера удаляет буферы только своих моделей.

Имена файлов в тестах — дословно с робота (ls /dev/shm, 03.11.2026).
"""
from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
HAILO_SCRIPTS = REPO_ROOT / "docker" / "vision" / "scripts" / "vision-hailo"
REAP = HAILO_SCRIPTS / "hailo_shm_reap.py"

_spec = importlib.util.spec_from_file_location("hailo_shm_reap", REAP)
reap_mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(reap_mod)

YOLO = [
    "yolov8n_conv52_97_4_628_628",
    "yolov8n_input_layer1_0_46_46",
]
FACE = [
    "retinaface_mobilenet_v1_input_layer1_7_53_53",
    "retinaface_mobilenet_v1_conv43_2_251_251",
]
ARC = ["arcface_mobilefacenet_fc1_0_101_101"]
FOREIGN = [
    "SuperColliderServer_57110",
    "jack-shm-registry",
    "yolov8n_notes",          # префикс свой, но не формат HailoRT
    "yolov8nx_conv1_0_1_1",   # чужая модель с похожим именем
]


def _populate(d: Path, names: list[str]) -> None:
    for n in names:
        (d / n).write_bytes(b"x" * 4096)


def _left(d: Path) -> set[str]:
    return {p.name for p in d.iterdir()}


def test_stems_from_hef_paths() -> None:
    assert reap_mod.model_stems(
        ["/opt/rob_box/models/retinaface_mobilenet_v1.hef", "",
         "/opt/rob_box/models/arcface_mobilefacenet.hef",
         "/opt/rob_box/models/retinaface_mobilenet_v1.hef"]
    ) == ["retinaface_mobilenet_v1", "arcface_mobilefacenet"]


def test_vision_hailo_reaps_only_yolo(tmp_path: Path) -> None:
    _populate(tmp_path, YOLO + FACE + ARC + FOREIGN)
    removed, freed, errors = reap_mod.reap(str(tmp_path), ["yolov8n"])
    assert errors == []
    assert removed == len(YOLO)
    assert freed > 0
    assert _left(tmp_path) == set(FACE + ARC + FOREIGN)


def test_vision_face_reaps_retinaface_and_arcface(tmp_path: Path) -> None:
    _populate(tmp_path, YOLO + FACE + ARC + FOREIGN)
    removed, _, errors = reap_mod.reap(
        str(tmp_path), ["retinaface_mobilenet_v1", "arcface_mobilefacenet"])
    assert errors == []
    assert removed == len(FACE + ARC)
    assert _left(tmp_path) == set(YOLO + FOREIGN)


def test_no_stems_touches_nothing(tmp_path: Path) -> None:
    _populate(tmp_path, YOLO)
    assert reap_mod.reap(str(tmp_path), []) == (0, 0, [])
    assert _left(tmp_path) == set(YOLO)


def test_directories_and_symlinks_are_skipped(tmp_path: Path) -> None:
    (tmp_path / "yolov8n_dir_0_1_1").mkdir()
    target = tmp_path / "keep_me"
    target.write_bytes(b"x")
    link = tmp_path / "yolov8n_link_0_2_2"
    try:
        os.symlink(target, link)
    except (OSError, NotImplementedError):
        pytest.skip("symlink недоступен на этой ФС")
    removed, _, _ = reap_mod.reap(str(tmp_path), ["yolov8n"])
    assert removed == 0
    assert target.exists() and link.is_symlink()


def test_missing_dir_is_not_fatal(tmp_path: Path) -> None:
    removed, freed, errors = reap_mod.reap(str(tmp_path / "nope"), ["yolov8n"])
    assert (removed, freed) == (0, 0)
    assert len(errors) == 1


def test_cli_logs_and_exits_zero(tmp_path: Path) -> None:
    _populate(tmp_path, YOLO + FACE)
    proc = subprocess.run(
        [sys.executable, str(REAP), "--shm-dir", str(tmp_path),
         "--tag", "[t]", "--min-free-mb", "0",
         "/opt/rob_box/models/yolov8n.hef"],
        capture_output=True, text=True, timeout=30,
    )
    assert proc.returncode == 0, proc.stderr
    assert "[t] shm reap" in proc.stdout
    assert "models=yolov8n removed=2" in proc.stdout
    assert _left(tmp_path) == set(FACE)


def test_cli_warns_when_shm_still_low(tmp_path: Path) -> None:
    if not hasattr(os, "statvfs"):
        pytest.skip("os.statvfs нет на этой ОС")
    proc = subprocess.run(
        [sys.executable, str(REAP), "--shm-dir", str(tmp_path),
         "--min-free-mb", "1e12", "/x/yolov8n.hef"],
        capture_output=True, text=True, timeout=30,
    )
    assert proc.returncode == 0
    assert "SIGBUS" in proc.stderr and "#4277" in proc.stderr


def test_start_scripts_call_reaper_before_launch() -> None:
    for name, args in (
        ("start_vision_hailo.sh", '"${HEF_PATH}" || true'),
        ("start_vision_face.sh", '"${HEF_PATH}" "${ARCFACE_HEF_PATH}" || true'),
    ):
        text = (HAILO_SCRIPTS / name).read_text(encoding="utf-8")
        assert "hailo_shm_reap.py" in text, name
        assert args in text, name
        assert text.index("hailo_shm_reap.py") < text.index("exec ros2 launch"), name
