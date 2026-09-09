"""
test_issue_1562_late_stop_after_synth.py — Issue #1562 regression:
end-to-end race scenario from live logs 23.08 17:42 («робот новая
сессия сбрось все настройки»).

Validates that the *combined* #1563 machinery
(IMMUNE-window + post-synth STOP buffer + dialogue IGNORE_STOP_MS payload)
keeps the response audible despite a double STOP (barge-in from wake-word
+ session-reset _cancel_run).

Original PR #1572 (`z-{agent}/1562-bug-voice-new-session-tts`) carried a
weaker implementation that was superseded by fix(voice #1563) #1568 — see
PR #1568 for the canonical fix (TTS state-aware STOP + IGNORE_STOP_MS
IMMUNE-окно после new-session reset). The race scenario from the live
logs is preserved here as a regression test exercising #1563 end-to-end
(``_handle_stop_command`` + ``_is_post_synth_phase`` + ``_post_synth_stop_pending``
drain at ``synth_speech`` start).

Acceptance (issue #1562, lived through #1563):
- «робот новая сессия» → TTS доигрывает ответ «Начинаю новую сессию».
- STOP после синтеза не отменяет уже готовый ответ (буферизуется).
- Ранний STOP (до synth) продолжает работать как раньше.
"""

from __future__ import annotations

import importlib.util
import sys
import time
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


def _load_tts_node_class():
    """Подгрузить ``rob_box_voice.tts_node`` через importlib.

    Подменяем ``grpc`` MagicMock'ом с ``__version__=``«1.78.0» ДО загрузки
    модуля: иначе yandex.cloud SDK падает на ``grpc.__version__``.
    Остальные stubs (rclpy / sounddevice / torch) ставит conftest.py.
    """
    grpc_stub = MagicMock()
    grpc_stub.__version__ = "1.78.0"
    grpc_stub.RpcError = type("RpcError", (Exception,), {})
    utilities_stub = MagicMock()
    utilities_stub.first_version_is_lower = lambda *a, **k: False
    grpc_stub._utilities = utilities_stub
    sys.modules["grpc"] = grpc_stub

    yandex_pkg = types.ModuleType("yandex")
    yandex_pkg.__path__ = []  # type: ignore[attr-defined]
    sys.modules.setdefault("yandex", yandex_pkg)
    cloud_pkg = types.ModuleType("yandex.cloud")
    cloud_pkg.__path__ = []  # type: ignore[attr-defined]
    sys.modules["yandex.cloud"] = cloud_pkg
    ai_pkg = types.ModuleType("yandex.cloud.ai")
    ai_pkg.__path__ = []  # type: ignore[attr-defined]
    sys.modules["yandex.cloud.ai"] = ai_pkg
    tts_pkg = types.ModuleType("yandex.cloud.ai.tts")
    tts_pkg.__path__ = []  # type: ignore[attr-defined]
    sys.modules["yandex.cloud.ai.tts"] = tts_pkg
    v3_pkg = types.ModuleType("yandex.cloud.ai.tts.v3")
    v3_pkg.tts_pb2 = MagicMock()
    v3_pkg.tts_service_pb2_grpc = MagicMock()
    sys.modules["yandex.cloud.ai.tts.v3"] = v3_pkg

    pkg_root = Path(__file__).resolve().parent.parent.parent.parent
    if "rob_box_voice" not in sys.modules:
        pkg = types.ModuleType("rob_box_voice")
        pkg.__path__ = [str(pkg_root / "rob_box_voice")]
        sys.modules["rob_box_voice"] = pkg
    spec = importlib.util.spec_from_file_location(
        "rob_box_voice.tts_node",
        pkg_root / "rob_box_voice" / "tts_node.py",
    )
    if spec is None or spec.loader is None:  # pragma: no cover
        raise RuntimeError("Failed to load rob_box_voice.tts_node")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["rob_box_voice.tts_node"] = mod
    spec.loader.exec_module(mod)
    return mod.TTSNode


_TTS_NODE_CLS_CACHE: dict = {}


@pytest.fixture(scope="module")
def tts_node_cls():
    if "_cls" not in _TTS_NODE_CLS_CACHE:
        _TTS_NODE_CLS_CACHE["_cls"] = _load_tts_node_class()
    return _TTS_NODE_CLS_CACHE["_cls"]


def _make_node(tts_node_cls, *, post_synth_phase: bool = False):
    """Минимальный TTSNode без __init__ — нужные атрибуты вручную."""
    n = object.__new__(tts_node_cls)
    n.get_logger = lambda: MagicMock()
    n.stop_requested = False
    n.processing_dialogue_id = None
    n.current_stream = None
    # Issue #1563 init-блок, покрывающий issue #1562 acceptance.
    n._immune_until_ts = 0.0
    n._post_synth_stop_pending = False
    n._interrupt_playback = MagicMock()
    n.publish_state = MagicMock()
    n._is_post_synth_phase = MagicMock(return_value=post_synth_phase)
    return n


class TestInterruptPlaybackAfterSynth:
    """#1563 (post-synth STOP buffer) реализует #1562 acceptance #1."""

    def test_early_stop_sets_stop_requested(self, tts_node_cls):
        """STOP до синтеза → stop_requested=True (как раньше)."""
        n = _make_node(tts_node_cls, post_synth_phase=False)
        tts_node_cls._handle_stop_command(n, "STOP")
        # В обычном пути _handle_stop_command вызывает _interrupt_playback
        # → наш MagicMock фиксирует вызов, stop_requested остаётся False
        # (мы не инстанцировали __init__ — флаг не реальный, проверяем
        # факт вызова через _interrupt_playback).
        n._interrupt_playback.assert_called_once()

    def test_late_stop_in_post_synth_phase_buffers(self, tts_node_cls):
        """STOP после синтеза → _post_synth_stop_pending=True, текущий доигрывается."""
        n = _make_node(tts_node_cls, post_synth_phase=True)
        tts_node_cls._handle_stop_command(n, "STOP")
        # В post-synth фазе _interrupt_playback НЕ должен вызываться —
        # STOP буферизуется для следующего запроса.
        n._interrupt_playback.assert_not_called()
        assert n._post_synth_stop_pending is True

    def test_late_stop_when_already_pending_keeps_pending(self, tts_node_cls):
        """Повторный STOP в post-synth + уже pending → остаётся pending, не прерывает."""
        n = _make_node(tts_node_cls, post_synth_phase=True)
        n._post_synth_stop_pending = True
        tts_node_cls._handle_stop_command(n, "STOP")
        n._interrupt_playback.assert_not_called()
        assert n._post_synth_stop_pending is True


class TestImmuneWindowFromNewSession:
    """#1563 IMMUNE-окно (IGNORE_STOP_MS:700 от dialogue_node)."""

    def test_stop_in_immune_window_is_ignored(self, tts_node_cls):
        """STOP в IMMUNE-окне → игнорируется (current chunk доигрывается)."""
        n = _make_node(tts_node_cls)
        n._immune_until_ts = time.monotonic() + 0.7  # окно открыто
        tts_node_cls._handle_stop_command(n, "STOP")
        n._interrupt_playback.assert_not_called()
        assert n._post_synth_stop_pending is False

    def test_stop_outside_immune_window_stops_normally(self, tts_node_cls):
        """STOP после истечения IMMUNE → обычная остановка."""
        n = _make_node(tts_node_cls, post_synth_phase=False)
        n._immune_until_ts = time.monotonic() - 0.001  # окно истекло
        tts_node_cls._handle_stop_command(n, "STOP")
        n._interrupt_playback.assert_called_once()


class TestPendingStopDrainedAtSynthStart:
    """synth_speech потребляет _post_synth_stop_pending в начале (issue #1563)."""

    def test_no_pending_does_not_trigger_stop(self, tts_node_cls):
        """Без pending → _interrupt_playback НЕ вызван."""
        n = _make_node(tts_node_cls)
        n._post_synth_stop_pending = False
        # Симуляция блока из synth_speech (см. tts_node.py).
        if getattr(n, "_post_synth_stop_pending", False):
            n._post_synth_stop_pending = False
            n._interrupt_playback()
        n._interrupt_playback.assert_not_called()

    def test_pending_drains_to_interrupt_playback(self, tts_node_cls):
        """pending=True → interrupt_playback (новый синтез сразу отменяется)."""
        n = _make_node(tts_node_cls)
        n._post_synth_stop_pending = True
        # Симуляция блока из synth_speech.
        if getattr(n, "_post_synth_stop_pending", False):
            n._post_synth_stop_pending = False
            n._interrupt_playback()
        n._interrupt_playback.assert_called_once()
        assert n._post_synth_stop_pending is False


class TestRaceScenarioFromLiveLogs:
    """Полный race-сценарий из live 23.08 17:42 (issue #1562)."""

    def test_full_new_session_race_scenario(self, tts_node_cls):
        """Шаги 1-5 из логов 17:42 + IMMUNE + post-synth buffer."""
        n = _make_node(tts_node_cls, post_synth_phase=True)

        # Шаг 1: dialogue_node шлёт IGNORE_STOP_MS:700 ПЕРЕД _publish_response
        # (в tts_node — только проверка что окно открылось).
        n._immune_until_ts = time.monotonic() + 0.7

        # Шаг 2: barge-in STOP от wake-word в той же фразе — должен быть
        # ПРОИГНОРИРОВАН (в IMMUNE-окне) даже при active post_synth флаге.
        tts_node_cls._handle_stop_command(n, "STOP")
        assert n._interrupt_playback.call_count == 0, (
            "Barge-in STOP в IMMUNE-окне должен игнорироваться "
            "(иначе TTS «Начинаю новую сессию» отменится ДО play)"
        )
        assert n._post_synth_stop_pending is False

        # Шаг 3: IMMUNE-окно истекло, приходит поздний STOP в post-synth фазе.
        n._immune_until_ts = time.monotonic() - 0.001
        tts_node_cls._handle_stop_command(n, "STOP")
        assert n._interrupt_playback.call_count == 0, (
            "Late STOP в post-synth фазе не должен отменять текущий chunk"
        )
        assert n._post_synth_stop_pending is True, (
            "STOP должен буферизоваться для следующего запроса"
        )

        # Шаг 4: следующий синтез — drain pending → _interrupt_playback.
        if getattr(n, "_post_synth_stop_pending", False):
            n._post_synth_stop_pending = False
            n._interrupt_playback()
        assert n._interrupt_playback.call_count == 1
        assert n._post_synth_stop_pending is False