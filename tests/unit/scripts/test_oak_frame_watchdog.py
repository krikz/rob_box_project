"""Регресс #4532: healthcheck oak-d зелёный при X_LINK_ERROR.

Чистая логика сторожа кадров (docker/vision/scripts/oak-d/oak_frame_watchdog.py)
на подменах времени и файлов: свежие кадры -> 0, нет кадров дольше порога -> 1,
первые секунды после старта -> 0.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
OAK = REPO_ROOT / "docker" / "vision" / "scripts" / "oak-d"

_spec = importlib.util.spec_from_file_location("oak_frame_watchdog", OAK / "oak_frame_watchdog.py")
wd = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(wd)

STALE = wd.DEFAULT_STALE_SEC
GRACE = wd.DEFAULT_GRACE_SEC
T0 = 1_000_000.0


def test_fresh_frames_are_ok() -> None:
    assert wd.verdict(T0 + 500, last_frame=T0 + 498, started=T0) == wd.OK


def test_no_frames_longer_than_threshold_is_stale() -> None:
    assert wd.verdict(T0 + 500, last_frame=T0 + 500 - STALE - 1, started=T0) == wd.STALE


def test_first_seconds_after_start_are_not_stale() -> None:
    assert wd.verdict(T0 + 5, last_frame=None, started=T0) == wd.STARTING


def test_no_frames_after_grace_is_stale() -> None:
    assert wd.verdict(T0 + GRACE + 1, last_frame=None, started=T0) == wd.STALE


def test_old_heartbeat_from_previous_run_does_not_count() -> None:
    # Контейнер при рестарте сохраняет /tmp: кадр до старта сторожа свежим не считается.
    assert wd.verdict(T0 + GRACE + 1, last_frame=T0 - 1, started=T0) == wd.STALE


def test_watchdog_not_started_is_stale() -> None:
    assert wd.verdict(T0, last_frame=T0, started=None) == wd.STALE


def test_clock_going_backwards_is_not_freshness() -> None:
    assert wd.verdict(T0 - 100, last_frame=T0, started=T0 - 200) == wd.STALE


def test_xlink_error_after_start_is_stale_even_with_fresh_frame() -> None:
    # Кадр моложе порога, но драйвер уже записал X_LINK_ERROR в этом запуске — не healthy.
    assert wd.verdict(T0 + 200, last_frame=T0 + 199, started=T0, xlink_error_at=T0 + 150) == wd.STALE


def test_xlink_error_during_grace_is_stale() -> None:
    assert wd.verdict(T0 + 5, last_frame=None, started=T0, xlink_error_at=T0 + 4) == wd.STALE


def test_xlink_error_from_previous_run_does_not_count() -> None:
    assert wd.verdict(T0 + 5, last_frame=T0 + 4, started=T0, xlink_error_at=T0 - 1) == wd.OK


def test_xlink_marker_in_the_future_is_not_this_run() -> None:
    assert wd.verdict(T0 + 5, last_frame=T0 + 4, started=T0, xlink_error_at=T0 + 50) == wd.OK


@pytest.fixture
def paths(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> tuple[Path, Path, Path]:
    hb, st, xlink = tmp_path / "hb", tmp_path / "started", tmp_path / "xlink"
    monkeypatch.setenv("OAK_HEARTBEAT_PATH", str(hb))
    monkeypatch.setenv("OAK_STARTED_PATH", str(st))
    monkeypatch.setenv("OAK_XLINK_PATH", str(xlink))
    return hb, st, xlink


def _stamp(path: Path, at: float) -> None:
    path.write_text("")
    os.utime(path, (at, at))


def test_check_exit_codes_with_files(paths: tuple[Path, Path, Path]) -> None:
    hb, st, xlink = paths
    assert wd.check(now=T0) == 1, "сторожа нет"
    _stamp(st, T0)
    assert wd.check(now=T0 + 3) == 0, "льготные секунды старта"
    assert wd.check(now=T0 + GRACE + 5) == 1, "кадров так и нет"
    _stamp(hb, T0 + 200)
    assert wd.check(now=T0 + 205) == 0, "кадры идут"
    assert wd.check(now=T0 + 200 + STALE + 1) == 1, "кадры пропали"
    _stamp(hb, T0 + 400)
    _stamp(xlink, T0 + 401)
    assert wd.check(now=T0 + 402) == 1, "X_LINK_ERROR при свежем кадре — нездоров"


def test_note_xlink_error_marks_only_the_token(tmp_path: Path) -> None:
    log = tmp_path / "driver.log"
    marker = tmp_path / "xlink"
    log.write_bytes(b"Camera ready!\n")
    offset = wd.note_xlink_error(str(log), 0, str(marker))
    assert offset == log.stat().st_size
    assert not marker.exists(), "обычный лог отметку не ставит"
    with log.open("ab") as handle:
        handle.write(b"Couldn't read data from stream: 'sys_logger_queue' (X_LINK_ERROR)\n")
    offset = wd.note_xlink_error(str(log), offset, str(marker))
    assert marker.is_file(), "X_LINK_ERROR в новом хвосте — отметка"
    assert offset == log.stat().st_size
    # Повторное чтение того же хвоста отметку не требует заново и offset не откатывает.
    again = wd.note_xlink_error(str(log), offset, str(marker))
    assert again == offset


def test_note_xlink_error_missing_log_keeps_offset(tmp_path: Path) -> None:
    assert wd.note_xlink_error(str(tmp_path / "nope.log"), 7, str(tmp_path / "xlink")) == 7


def test_start_script_feeds_driver_log_to_watchdog() -> None:
    start = (OAK / "start_oak_d.sh").read_text(encoding="utf-8")
    assert "/tmp/oak_driver.log" in start
    assert "ros2 launch oakd_with_apriltag.launch.py >> /tmp/oak_driver.log 2>&1 &" in start


def test_compose_healthcheck_uses_frames_and_covers_grace() -> None:
    svc = yaml.safe_load((REPO_ROOT / "docker/vision/docker-compose.yaml").read_text(encoding="utf-8"))["services"]
    oak = svc["oak-d"]
    assert "oak_frame_watchdog.py check" in oak["healthcheck"]["test"][-1]
    assert oak["restart"] == "unless-stopped"
    start_period = int(str(oak["healthcheck"]["start_period"]).rstrip("s"))
    assert start_period >= GRACE
    # Выход сторожа обязан ронять контейнер, иначе restart policy не сработает.
    start = (OAK / "start_oak_d.sh").read_text(encoding="utf-8")
    assert "oak_frame_watchdog.py run" in start and "wait -n" in start
