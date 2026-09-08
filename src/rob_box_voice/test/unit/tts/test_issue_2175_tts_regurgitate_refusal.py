"""
test_issue_2175_tts_regurgitate_refusal.py — TTS chokepoint drops
MiniMax regurgitated ``<system>...</system>`` template (defense-in-depth).

Issue #2175 (live 08.09, vision-pi 14:52): три запроса подряд после
``set_voice`` + multi-voice user_input + новая DJ-skill context дали
в ``spoken`` кусок СИСТЕМНОГО промпта::

    <system>
    [получатель ответа забыл указать антропоморфные атрибуты]
    </system>

TTS озвучивал эту метаинструкцию через Yandex→MiniMax fallback, юзер
слышал «получатель ответа забыл указать антропоморфные атрибуты»
поверх только что сменённого голоса.

Первая линия защиты — ``dialogue_node._check_system_template_regurgitate_and_retry``
(см. test_issue_2175_dialogue_node_system_regurgitate_guard.py). Она
отправляет ОДИН CRITICAL-ретрай и обычно спасает.

Вторая линия защиты (defense-in-depth, эти тесты) — ``TTSNode.dialogue_callback``
ОТКАЗЫВАЕТСЯ синтезировать regurgitated template даже если dialogue_node
guard почему-то пропустил (например, пришёл запрос от MCP ``speak_text``
напрямую, минуя dialogue_node; или fallback-цепочка dialogue_node→LLM
проскочила guard в нестандартной ситуации). Результат:

* ``_submit_synthesis`` НЕ вызывается;
* публикуется ``/voice/tts/finished`` с ``success=False`` и
  ``error="system_template_regurgitated"`` — чтобы ``SpeakTextTool``
  не висел в ожидании (тот же контракт, что для issue #1709);
* в лог уходит полный regurgitated текст для forensic (как в
  issue #1709 / #1564).

Паттерн загрузки ``TTSNode`` (importlib + grpc/yandex stubs) и
``_make_node`` (без __init__) — точно как в
``test_issue_1709_unicode_guard.py``.
"""
from __future__ import annotations

import importlib.util
import json
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


# ---------------------------------------------------------------------------
# Загрузка TTSNode (как в test_issue_1709_unicode_guard.py)
# ---------------------------------------------------------------------------


def _load_tts_node_class():
    """Загрузить ``rob_box_voice.tts_node`` и вернуть класс ``TTSNode``."""
    _touched = (
        "grpc",
        "yandex",
        "yandex.cloud",
        "yandex.cloud.ai",
        "yandex.cloud.ai.tts",
        "yandex.cloud.ai.tts.v3",
        "rob_box_voice.tts_node",
    )
    saved = {k: sys.modules.get(k) for k in _touched}

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
    v3_pkg.tts_pb2 = MagicMock()  # type: ignore[attr-defined]
    v3_pkg.tts_service_pb2_grpc = MagicMock()  # type: ignore[attr-defined]
    sys.modules["yandex.cloud.ai.tts.v3"] = v3_pkg

    pkg_root = Path(__file__).resolve().parent.parent.parent.parent
    if "rob_box_voice" not in sys.modules:
        pkg = types.ModuleType("rob_box_voice")
        pkg.__path__ = [str(pkg_root / "rob_box_voice")]  # type: ignore[attr-defined]
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
    return mod.TTSNode, saved


@pytest.fixture(scope="module")
def tts_node_cls():
    """TTSNode class; sys.modules-подмены откатываются после модуля."""
    cls, saved = _load_tts_node_class()
    try:
        yield cls
    finally:
        for key, value in saved.items():
            if value is None:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = value


# ---------------------------------------------------------------------------
# Scaffolding (идентично test_issue_1709_unicode_guard.py)
# ---------------------------------------------------------------------------


class _CapturingLogger:
    def __init__(self) -> None:
        self.info_msgs: list = []
        self.warn_msgs: list = []
        self.error_msgs: list = []
        self.debug_msgs: list = []

    def info(self, msg):
        self.info_msgs.append(msg)

    def warn(self, msg):
        self.warn_msgs.append(msg)

    def warning(self, msg):
        self.warn_msgs.append(msg)

    def error(self, msg):
        self.error_msgs.append(msg)

    def debug(self, msg):
        self.debug_msgs.append(msg)


class _CapturingPublisher:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _make_node(tts_node_cls):
    n = object.__new__(tts_node_cls)
    logger = _CapturingLogger()
    n._logger = logger
    n.get_logger = lambda: logger
    n.current_speech_id = None
    n.current_dialogue_id = None
    n.processing_dialogue_id = None
    n.finished_pub = _CapturingPublisher()
    n.batch_complete_pub = _CapturingPublisher()
    n._submit_synthesis = MagicMock()
    return n


def _msg(payload: dict):
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


def _finished_payloads(node) -> list:
    return [json.loads(m.data) for m in node.finished_pub.messages]


# ---------------------------------------------------------------------------
# Regurgitated system-template → НЕ синтезируется
# ---------------------------------------------------------------------------


class TestRegurgitatedSystemTemplateDropped:
    """Issue #2175 — defense-in-depth в tts_node."""

    def test_canonical_regurgitate_is_not_synthesized(self, tts_node_cls):
        """Канонический пример из живого лога 08.09 14:52."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": (
                    "<speak><system>\n"
                    "[получатель ответа забыл указать антропоморфные "
                    "атрибуты]\n"
                    "</system></speak>"
                ),
                "speech_id": "sid-2175-0001",
            })
        )
        node._submit_synthesis.assert_not_called()

    def test_minimal_system_block_is_not_synthesized(self, tts_node_cls):
        """Минимальный regurgitates (без whitespace) тоже дропается."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": "<speak><system>x</system></speak>",
                "speech_id": "sid-2175-0002",
            })
        )
        node._submit_synthesis.assert_not_called()

    def test_dropped_chunk_publishes_finished_failure(self, tts_node_cls):
        """``SpeakTextTool`` не должен висеть: finished(success=False)
        с error="system_template_regurgitated" — тот же контракт, что
        для issue #1709 unicode-guard."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": (
                    "<speak><system>\n[foo bar]\n</system></speak>"
                ),
                "speech_id": "sid-2175-0003",
            })
        )
        payloads = _finished_payloads(node)
        assert payloads, "finished не опубликован — SpeakTextTool повиснет"
        assert payloads[0]["speech_id"] == "sid-2175-0003"
        assert payloads[0]["success"] is False
        assert payloads[0]["error"] == "system_template_regurgitated"

    def test_dropped_chunk_preserves_batch_metadata(self, tts_node_cls):
        """batch_id/index/total пробрасываются — батч закрывается корректно."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": "<speak><system>foo</system></speak>",
                "speech_id": "sid-2175-0004",
                "batch_id": "batch-1",
                "batch_index": 2,
                "batch_total": 2,
            })
        )
        payloads = _finished_payloads(node)
        assert payloads[0]["batch_id"] == "batch-1"
        assert payloads[0]["batch_index"] == 2
        assert payloads[0]["batch_total"] == 2

    def test_dropped_chunk_logs_full_text(self, tts_node_cls):
        """Acceptance: полный regurgitated текст в WARNING логе для
        forensic (как issue #1709 для чужих письменностей)."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": (
                    "<speak><system>\n"
                    "[получатель ответа забыл указать антропоморфные "
                    "атрибуты]\n"
                    "</system></speak>"
                ),
                "speech_id": "sid-2175-0005",
                "voice": "Russian_ReliableMan",
            })
        )
        warnings = "\n".join(node._logger.warn_msgs)
        assert "issue 2175" in warnings
        assert "regurgitated" in warnings
        # Полный текст regurgitates в логе (forensic)
        assert "получатель ответа забыл указать антропоморфные атрибуты" in warnings
        assert "voice=Russian_ReliableMan" in warnings


# ---------------------------------------------------------------------------
# Back-compat: нормальная речь и серединные ссылки НЕ блокируются
# ---------------------------------------------------------------------------


class TestNormalSpeechUnaffected:
    """Issue #2175 — guard не должен ломать back-compat."""

    def test_russian_text_is_synthesized(self, tts_node_cls):
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": "<speak>Привет! Я РОББОКС, поехали.</speak>",
                "speech_id": "sid-2175-back-0001",
            })
        )
        node._submit_synthesis.assert_called_once()
        assert _finished_payloads(node) == []

    def test_embedded_system_tag_is_synthesized(self, tts_node_cls):
        """Если LLM упомянула <system> в переносном смысле
        («согласно <system>инструкции</system>») — НЕ regurgitates,
        синтез идёт как обычно."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": "<speak>Согласно <system>инструкции</system>, отвечу.</speak>",
                "speech_id": "sid-2175-back-0002",
            })
        )
        node._submit_synthesis.assert_called_once()
        # finished не публикуется с ошибкой (если только TTS не упал в самой
        # логике — но в stub'е submit_synthesis не публикует ничего)
        for payload in _finished_payloads(node):
            assert payload.get("error") != "system_template_regurgitated"

    def test_unclosed_system_tag_is_synthesized(self, tts_node_cls):
        """Неполный тег (``<system>foo`` без закрывающего) — НЕ
        regurgitates (это неполный блок, не полный pattern)."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": "<speak>Пишу <system>размышление вслух</speak>",
                "speech_id": "sid-2175-back-0003",
            })
        )
        # Этот случай _не_ regurgitates (нет closing), но имеет
        # «встроенный» <system> — guard НЕ должен сработать. Однако
        # извлечённый текст будет «Пишу размышление вслух», и он не
        # regurgitates. Поэтому _submit_synthesis ДОЛЖЕН быть вызван.
        node._submit_synthesis.assert_called_once()


# ---------------------------------------------------------------------------
# Параметризация для основного паттерна
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "ssml_inner",
    [
        "<system>x</system>",
        "<system>\n[foo bar]\n</system>",
        # Канонический пример из issue body
        "<system>\n[получатель ответа забыл указать антропоморфные атрибуты]\n</system>",
        # Mixed case
        "<SYSTEM>foo</SYSTEM>",
    ],
)
def test_regurgitate_is_blocked(ssml_inner: str, tts_node_cls):
    """Параметризованная проверка — все варианты regurgitates дропаются."""
    node = _make_node(tts_node_cls)
    node.dialogue_callback(
        _msg({
            "ssml": f"<speak>{ssml_inner}</speak>",
            "speech_id": f"sid-2175-param-{abs(hash(ssml_inner))}",
        })
    )
    node._submit_synthesis.assert_not_called()
    payloads = _finished_payloads(node)
    assert any(p.get("error") == "system_template_regurgitated" for p in payloads)
