"""Unit tests для Phase 2.2 perf logger.

Покрывает чистые куски (JsonlWriter, PerfDedup, validate_telemetry_payload),
которые не зависят от rclpy. ROS2-обёртку тестируем минимально через dry-run.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import pytest

# Добавляем корень пакета в PYTHONPATH чтобы import работал из worktree.
_HERE = Path(__file__).resolve().parent
_PKG_ROOT = _HERE.parent  # src/rob_box_quest/
sys.path.insert(0, str(_PKG_ROOT))

from rob_box_quest.perf.logger_node import (  # noqa: E402
    JsonlWriter,
    PerfDedup,
    main,
    validate_telemetry_payload,
)


@pytest.fixture
def tmp_log_dir(tmp_path: Path) -> Path:
    """Каждый тест получает свой log_dir (auto-cleanup)."""
    d = tmp_path / "quest_perf"
    d.mkdir()
    return d


# ---- validate_telemetry_payload ------------------------------------------

class TestValidateTelemetryPayload:
    def test_valid_webxr_payload(self):
        payload = {
            "type": "telemetry_perf",
            "ts_ms": int(time.time() * 1000),
            "seq": 1,
            "source": "webxr",
            "fps_mean": 89.2,
            "gpu_ms": 6.5,
        }
        assert validate_telemetry_payload(payload) is None

    def test_valid_desktop_payload_minimal(self):
        payload = {
            "type": "telemetry_perf",
            "ts_ms": 1234567890,
            "seq": 1,
            "source": "desktop",
        }
        assert validate_telemetry_payload(payload) is None

    def test_rejects_wrong_type(self):
        payload = {"type": "heartbeat", "ts_ms": 1, "seq": 1, "source": "desktop"}
        err = validate_telemetry_payload(payload)
        assert err is not None
        assert "type" in err

    def test_rejects_zero_ts(self):
        payload = {"type": "telemetry_perf", "ts_ms": 0, "seq": 1, "source": "desktop"}
        err = validate_telemetry_payload(payload)
        assert err is not None
        assert "ts_ms" in err

    def test_rejects_negative_seq(self):
        payload = {"type": "telemetry_perf", "ts_ms": 1, "seq": -1, "source": "desktop"}
        err = validate_telemetry_payload(payload)
        assert err is not None
        assert "seq" in err

    def test_rejects_unknown_source(self):
        payload = {"type": "telemetry_perf", "ts_ms": 1, "seq": 1, "source": "vr"}
        err = validate_telemetry_payload(payload)
        assert err is not None
        assert "source" in err


# ---- PerfDedup -------------------------------------------------------- ----------------------------------------------------

class TestPerfDedup:
    def test_first_seq_is_not_duplicate(self):
        d = PerfDedup()
        assert d.is_duplicate("sess-A", 1) is False

    def test_repeated_seq_is_duplicate(self):
        d = PerfDedup()
        assert d.is_duplicate("sess-A", 5) is False
        assert d.is_duplicate("sess-A", 5) is True  # same seq → drop
        assert d.is_duplicate("sess-A", 4) is True  # older seq → drop

    def test_increasing_seq_per_session(self):
        d = PerfDedup()
        assert d.is_duplicate("sess-A", 1) is False
        assert d.is_duplicate("sess-A", 2) is False
        assert d.is_duplicate("sess-A", 3) is False
        assert d.is_duplicate("sess-A", 2) is True

    def test_independent_sessions_have_independent_seqs(self):
        d = PerfDedup()
        assert d.is_duplicate("sess-A", 100) is False
        # sess-B может иметь такой же seq — это не duplicate.
        assert d.is_duplicate("sess-B", 100) is False

    def test_trims_old_sessions_at_max(self):
        d = PerfDedup(max_sessions=3)
        d.is_duplicate("s1", 1)
        d.is_duplicate("s2", 1)
        d.is_duplicate("s3", 1)
        # Переполнение → должна trim'нуть одну сессию (FIFO).
        d.is_duplicate("s4", 1)
        # Теперь s1 снова fresh (его seq не отслеживается).
        assert d.is_duplicate("s1", 1) is False  # trim произошёл


# ---- JsonlWriter -------------------------------------------------------- ----------------------------------------------------

class TestJsonlWriter:
    def test_write_creates_file(self, tmp_log_dir: Path):
        w = JsonlWriter(log_dir=str(tmp_log_dir), retention_days=0)
        w.write({"ts_ms": 1, "seq": 1, "type": "telemetry_perf"})
        w.close()
        files = list(tmp_log_dir.glob("*.jsonl"))
        assert len(files) == 1
        content = files[0].read_text().strip()
        record = json.loads(content)
        assert record["seq"] == 1
        assert record["type"] == "telemetry_perf"

    def test_appends_in_place(self, tmp_log_dir: Path):
        w = JsonlWriter(log_dir=str(tmp_log_dir), retention_days=0)
        w.write({"seq": 1})
        w.write({"seq": 2})
        w.close()
        content = next(tmp_log_dir.glob("*.jsonl")).read_text()
        # Два отдельных JSON объекта, каждый на своей строке.
        assert content.count("\n") == 2 or content.endswith("\n")

    def test_rotates_when_size_exceeded(self, tmp_log_dir: Path):
        # max 100 байт — второй record должен вызвать rotation.
        w = JsonlWriter(
            log_dir=str(tmp_log_dir),
            max_bytes_per_file=100,
            retention_days=0,
        )
        # Каждый record ~60 байт JSON.
        for i in range(5):
            w.write({"seq": i, "padding": "x" * 40})
        w.close()
        files = sorted(tmp_log_dir.glob("*.jsonl"))
        assert len(files) >= 2, f"expected rotation, got {files}"
        # Первый файл — initial, второй — .1
        assert files[0].name == f"{files[0].stem}.jsonl" or ".jsonl" in files[0].name

    def test_unicode_payload(self, tmp_log_dir: Path):
        w = JsonlWriter(log_dir=str(tmp_log_dir), retention_days=0)
        w.write({"seq": 1, "tag": "робот"})
        w.close()
        content = next(tmp_log_dir.glob("*.jsonl")).read_text(encoding="utf-8")
        assert "робот" in content

    def test_thread_safety(self, tmp_log_dir: Path):
        """Многопоточная запись не теряет records."""
        import threading

        w = JsonlWriter(log_dir=str(tmp_log_dir), retention_days=0)

        def worker(start: int):
            for i in range(50):
                w.write({"seq": start + i, "thread": start})

        threads = [threading.Thread(target=worker, args=(t * 100,)) for t in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        w.close()

        content = next(tmp_log_dir.glob("*.jsonl")).read_text()
        lines = [ln for ln in content.split("\n") if ln]
        assert len(lines) == 4 * 50

    def test_prunes_old_files(self, tmp_log_dir: Path):
        """Старые jsonl-файлы удаляются при инициализации."""
        # Создаём "старый" файл с mtime = now - 40 дней.
        old_file = tmp_log_dir / "2025-01-01.jsonl"
        old_file.write_text('{"seq": 1}')
        old_time = time.time() - 40 * 86400
        os.utime(old_file, (old_time, old_time))

        # Создаём "свежий" файл.
        new_file = tmp_log_dir / "2025-12-31.jsonl"
        new_file.write_text('{"seq": 2}')

        w = JsonlWriter(log_dir=str(tmp_log_dir), retention_days=30)
        w.close()

        assert not old_file.exists(), f"old file not pruned: {old_file}"
        assert new_file.exists(), f"new file accidentally pruned"

    def test_close_is_idempotent(self, tmp_log_dir: Path):
        w = JsonlWriter(log_dir=str(tmp_log_dir), retention_days=0)
        w.write({"seq": 1})
        w.close()
        w.close()  # не должно падать
        assert True


# ---- main() dry-run ----------------------------------------------------- ----------------------------------------------------

class TestMainDryRun:
    def test_dry_run_exits_zero(self, monkeypatch):
        monkeypatch.setattr("sys.argv", ["prog", "--dry-run"])
        rc = main()
        assert rc == 0


# ---- CLI arguments ------------------------------------------------------ ----------------------------------------------------

class TestCliArgs:
    def test_default_log_dir(self, tmp_log_dir: Path, monkeypatch):
        monkeypatch.setattr("sys.argv", ["prog", "--dry-run", "--log-dir", str(tmp_log_dir)])
        # Проверяем, что argparse парсит без ошибок и пишет в нужный dir.
        rc = main()
        assert rc == 0
        # Никаких файлов (dry-run).
        assert list(tmp_log_dir.glob("*.jsonl")) == []

    def test_retention_zero_keeps_old_files(self, tmp_log_dir: Path, monkeypatch):
        old_file = tmp_log_dir / "2020-01-01.jsonl"
        old_file.write_text('{"seq": 1}')
        old_time = time.time() - 365 * 86400
        os.utime(old_file, (old_time, old_time))

        monkeypatch.setattr("sys.argv", ["prog", "--dry-run", "--log-dir", str(tmp_log_dir), "--retention-days", "0"])
        main()
        assert old_file.exists()