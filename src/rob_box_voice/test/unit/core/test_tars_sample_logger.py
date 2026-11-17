"""Юнит-тесты для ADR-0076 — side-channel сборщик STT-семплов «ТАРС» с шлема.

Что проверяем:

1. По умолчанию (без env) — no-op, файл не создаётся, никаких ошибок.
2. При ROBBOX_STT_COLLECT=1 — пишется JSONL с ожидаемыми полями.
3. Является идемпотентным по env: повторный вызов без env — молчит.
4. ``build_attempts_snapshot`` корректно снимает снимок с duck-type
   объектов и с пустого списка.
5. ``append_sample`` НЕ падает при ошибке записи (FS read-only).
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from rob_box_voice.core.tars_sample_logger import (
    DEFAULT_LOG_PATH,
    append_sample,
    build_attempts_snapshot,
)


@pytest.fixture
def collect_on(monkeypatch, tmp_path):
    """Включить сбор и подменить путь по умолчанию на tmp_path."""
    monkeypatch.setenv("ROBBOX_STT_COLLECT", "1")
    log = tmp_path / "stt_tars_samples.jsonl"
    yield log
    monkeypatch.delenv("ROBBOX_STT_COLLECT", raising=False)


@pytest.fixture
def collect_off(monkeypatch):
    """Явно выключить сбор (на случай если env выставлен в shell)."""
    monkeypatch.delenv("ROBBOX_STT_COLLECT", raising=False)


class TestKillSwitch:
    def test_default_is_noop(self, collect_off, tmp_path):
        """Без env файл не создаётся и возвращается False."""
        out = append_sample(
            raw_text="арс расскажи мне анекдот",
            has_operator_wake=False,
            duration_s=1.2,
            attempts=[],
            operator_wake_words=("тарс", "tars"),
            path=tmp_path / "nope.jsonl",
        )
        assert out is False
        assert not (tmp_path / "nope.jsonl").exists()

    def test_env_other_values_do_not_enable(self, collect_off, monkeypatch, tmp_path):
        """Только строго '1', никаких 'true'/'yes'/'on'."""
        for val in ("true", "TRUE", "yes", "on", "1 ", " 1"):
            monkeypatch.setenv("ROBBOX_STT_COLLECT", val)
            out = append_sample(
                raw_text="арс",
                has_operator_wake=False,
                duration_s=1.0,
                attempts=[],
                operator_wake_words=("тарс",),
                path=tmp_path / "n.jsonl",
            )
            assert out is False, f"unexpected enable for value {val!r}"
            assert not (tmp_path / "n.jsonl").exists()


class TestAppendEnabled:
    def test_writes_one_json_line_per_call(self, collect_on):
        log = collect_on
        for i in range(3):
            append_sample(
                raw_text=f"арс {i}",
                has_operator_wake=False,
                duration_s=0.8 + i * 0.1,
                attempts=[
                    {"provider": "yandex", "ok": False, "reason": "low_confidence"},
                ],
                operator_wake_words=("тарс", "tars"),
                path=log,
            )
        assert log.exists()
        text = log.read_text(encoding="utf-8").strip().splitlines()
        assert len(text) == 3
        for line in text:
            rec = json.loads(line)
            assert set(rec.keys()) == {
                "ts_ms", "source", "duration_s", "raw_text",
                "has_operator_wake", "operator_wake_words", "attempts",
            }
            assert rec["source"] == "wake"
            assert rec["has_operator_wake"] is False
            assert rec["operator_wake_words"] == ["тарс", "tars"]

    def test_raw_text_none_for_empty_wake_segment(self, collect_on):
        log = collect_on
        append_sample(
            raw_text=None,
            has_operator_wake=False,
            duration_s=0.5,
            attempts=[],
            operator_wake_words=("тарс",),
            path=log,
        )
        rec = json.loads(log.read_text(encoding="utf-8").strip())
        assert rec["raw_text"] is None

    def test_creates_parent_directory(self, tmp_path, monkeypatch):
        monkeypatch.setenv("ROBBOX_STT_COLLECT", "1")
        nested = tmp_path / "a" / "b" / "c" / "log.jsonl"
        out = append_sample(
            raw_text="арс",
            has_operator_wake=False,
            duration_s=0.4,
            attempts=[],
            operator_wake_words=("тарс",),
            path=nested,
        )
        assert out is True
        assert nested.exists()

    def test_readonly_filesystem_does_not_raise(self, tmp_path, monkeypatch):
        """ADR-0076 §2.2: side-channel не должен ронять STT-ноду.

        Любой OSError на записи (read-only FS, EROFS, ENOSPC, EPERM...)
        приводит к возврату False, а не к исключению.

        В CI pytest крутится от ``--user root`` (см. .github/workflows/
        G-Run Tests.yml unit-tests job), и root на Linux игнорирует
        read-only биты. Поэтому вместо ``chmod 0o444`` подменяем
        ``builtins.open`` на версию, которая кидает OSError для нашего
        целевого файла — детерминированно в любых условиях.
        """
        monkeypatch.setenv("ROBBOX_STT_COLLECT", "1")
        target = tmp_path / "log.jsonl"

        import builtins as _bi
        original_open = _bi.open

        def boom_open(*a, **kw):
            # Блокируем запись только для нашего целевого файла; всё
            # остальное (включая чтение) пробрасываем в оригинал, чтобы
            # не сломать internals pytest.
            if (
                a
                and len(a) >= 1
                and isinstance(a[0], (str, os.PathLike))
                and str(a[0]) == str(target)
            ):
                raise OSError(30, "Read-only file system (simulated)")
            return original_open(*a, **kw)

        monkeypatch.setattr(_bi, "open", boom_open)

        out = append_sample(
            raw_text="арс",
            has_operator_wake=False,
            duration_s=0.3,
            attempts=[],
            operator_wake_words=("тарс",),
            path=target,
        )
        assert out is False
        # target НЕ должен появиться на диске.
        assert not target.exists()


class TestSnapshotBuilder:
    def test_empty_list(self):
        assert build_attempts_snapshot([]) == []
        assert build_attempts_snapshot(None) == []

    def test_ducktyped_attempt(self):
        class _Attempt:
            provider = "yandex"
            reason = "ok"
            latency_ms = 42.5

        assert build_attempts_snapshot([_Attempt()]) == [
            {"provider": "yandex", "ok": True, "reason": "ok", "latency_ms": 42}
        ]

    def test_unknown_failure_recorded(self):
        class _Broken:
            def __getattr__(self, name):
                raise RuntimeError("boom")

            # hasattr-friendly defaults
            provider = "broken"
            reason = "x"
            latency_ms = 0

        # Не должно бросать — пусть фолбек-объект будет в снапшоте.
        snap = build_attempts_snapshot([_Broken()])
        assert isinstance(snap, list)
        assert len(snap) == 1
        assert "provider" in snap[0]


class TestDefaultLogPath:
    def test_default_under_data_dir(self):
        # ADR-0076 §2.2: data/ уже в .gitignore — случайный запуск не утечёт.
        assert DEFAULT_LOG_PATH.parent.name == "data"
        assert DEFAULT_LOG_PATH.name == "stt_tars_samples.jsonl"
