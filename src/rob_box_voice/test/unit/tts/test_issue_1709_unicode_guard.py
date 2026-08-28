"""
test_issue_1709_unicode_guard.py — TTS chokepoint drops foreign-script text.

Issue #1709 (live 28.08): юзер услышал «бормотание на хинди», а в логе
``voice-assistant`` текста этого чанка не было — только ``speech_id`` и
2 failed TTS. Причина: LLM периодически вставляет в реплику символы
неподдерживаемых письменностей (CJK / деванагари / арабица), провайдер
читает их нечитаемо.

``TTSNode.dialogue_callback`` — ЕДИНСТВЕННЫЙ чокпоинт, через который
проходят ВСЕ TTS-запросы (и ``/voice/dialogue/response`` от
dialogue_node, и ``/voice/tts/request`` от MCP ``speak_text``) — тот же
чокпоинт, где живёт markdown-санитизация issue #988. Эти тесты
проверяют, что guard стоит именно там:

* текст с >10% чужих букв НЕ уходит в синтез (``_submit_synthesis``
  не вызывается) — робот молчит, а не бормочет;
* публикуется ``/voice/tts/finished`` с ``success=False`` и
  ``error="unsupported_script"``, чтобы MCP ``speak_text`` /
  dialogue_node не висели в ожидании навсегда (батч закрывается);
* нормальная русская речь проходит без изменений (back-compat);
* лог несёт ПОЛНЫЙ текст (acceptance issue #1709), а не префикс.

Паттерн загрузки ``TTSNode`` (importlib + grpc/yandex stubs) —
как в ``test_issue_1563_stop_immune.py``.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest


def _load_tts_node_class():
    """Загрузить ``rob_box_voice.tts_node`` и вернуть класс ``TTSNode``."""
    grpc_stub = MagicMock()
    grpc_stub.__version__ = "1.78.0"
    grpc_stub.RpcError = type("RpcError", (Exception,), {})
    utilities_stub = MagicMock()
    utilities_stub.first_version_is_lower = lambda *a, **k: False
    grpc_stub._utilities = utilities_stub
    sys.modules["grpc"] = grpc_stub

    import types

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
    return mod.TTSNode


_TTS_NODE_CLS_CACHE: dict = {}


@pytest.fixture(scope="module")
def tts_node_cls():
    if "_cls" not in _TTS_NODE_CLS_CACHE:
        _TTS_NODE_CLS_CACHE["_cls"] = _load_tts_node_class()
    return _TTS_NODE_CLS_CACHE["_cls"]


class _CapturingLogger:
    """Минимальный ROS-logger: tts_node зовёт .info/.warn/.warning/.error.

    Списки называются ``*_msgs``, чтобы не затенять одноимённые методы
    (``self.info = []`` сделал бы ``logger.info(...)`` невызываемым).
    """

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


class _FakeString:
    def __init__(self) -> None:
        self.data = ""


def _make_node(tts_node_cls):
    """TTSNode без __init__ — только то, что нужно dialogue_callback."""
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


def _all_logs(node) -> str:
    logger = node._logger
    return "\n".join(
        logger.info_msgs + logger.warn_msgs + logger.error_msgs
    )


# ── Guard блокирует чужие письменности ────────────────────────────────


class TestForeignScriptDropped:
    def test_hieroglyph_text_is_not_synthesized(self, tts_node_cls):
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({"ssml": "<speak>你好世界，加油加油</speak>",
                  "speech_id": "sid-cjk-0001"})
        )
        node._submit_synthesis.assert_not_called()

    def test_devanagari_text_is_not_synthesized(self, tts_node_cls):
        """«Хинди» из репорта юзера."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({"ssml": "<speak>नमस्ते दुनिया</speak>",
                  "speech_id": "sid-dev-0001"})
        )
        node._submit_synthesis.assert_not_called()

    def test_dropped_chunk_publishes_finished_failure(self, tts_node_cls):
        """MCP speak_text не должен висеть: приходит finished(success=False).

        Без этого ``pending_speeches`` в SpeakTextTool никогда не
        закрывается, ``batch_complete`` не публикуется и dialogue_node
        не делает music_cleanup (issue #980 контракт).
        """
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({"ssml": "<speak>你好世界，加油加油</speak>",
                  "speech_id": "sid-cjk-0002"})
        )
        payloads = _finished_payloads(node)
        assert payloads, "finished не опубликован — speak_text повиснет"
        assert payloads[0]["speech_id"] == "sid-cjk-0002"
        assert payloads[0]["success"] is False
        assert payloads[0]["error"] == "unsupported_script"

    def test_dropped_chunk_preserves_batch_metadata(self, tts_node_cls):
        """batch_id/index/total пробрасываются — батч закрывается корректно."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({
                "ssml": "<speak>加油加油加油加油</speak>",
                "speech_id": "sid-cjk-0003",
                "batch_id": "batch-1",
                "batch_index": 2,
                "batch_total": 2,
            })
        )
        payloads = _finished_payloads(node)
        assert payloads[0]["batch_id"] == "batch-1"
        assert payloads[0]["batch_index"] == 2
        assert payloads[0]["batch_total"] == 2
        # Последний чанк батча → batch_complete тоже улетает.
        assert node.batch_complete_pub.messages, (
            "последний чанк батча должен закрыть батч даже когда текст "
            "отброшен guard'ом"
        )

    def test_dropped_chunk_logs_full_text_and_diagnostics(self, tts_node_cls):
        """Acceptance: полный текст + доля/письменности в WARNING."""
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({"ssml": "<speak>Слушай: 加油加油加油加油加油加油</speak>",
                  "speech_id": "sid-cjk-0004",
                  "voice": "anton"})
        )
        warnings = "\n".join(node._logger.warn_msgs)
        assert "issue 1709" in warnings
        assert "加油加油加油加油加油加油" in warnings
        assert "foreign_ratio=" in warnings
        assert "cjk" in warnings
        assert "voice=anton" in warnings


# ── Back-compat: нормальная речь не задета ────────────────────────────


class TestNormalSpeechUnaffected:
    def test_russian_text_is_synthesized(self, tts_node_cls):
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({"ssml": "<speak>Привет! Я РОББОКС, поехали.</speak>",
                  "speech_id": "sid-ru-0001"})
        )
        node._submit_synthesis.assert_called_once()
        # Никакого finished-провала — синтез просто ушёл в работу.
        assert _finished_payloads(node) == []

    def test_single_hieroglyph_in_long_russian_still_synthesized(
        self, tts_node_cls
    ):
        """Порог 10%: вкрапление не глушит фразу (иначе робот замолчит)."""
        node = _make_node(tts_node_cls)
        text = (
            "Китайская идиома звучит как вай го, а пишется она "
            "вот таким единственным символом 加 в самом конце фразы"
        )
        node.dialogue_callback(
            _msg({"ssml": f"<speak>{text}</speak>", "speech_id": "sid-mix-1"})
        )
        node._submit_synthesis.assert_called_once()

    def test_emoji_and_digits_pass_through(self, tts_node_cls):
        node = _make_node(tts_node_cls)
        node.dialogue_callback(
            _msg({"ssml": "<speak>Погнали 2024! 🎶🦝 128 bpm!</speak>",
                  "speech_id": "sid-emoji-1"})
        )
        node._submit_synthesis.assert_called_once()

    def test_normal_log_carries_full_text_not_prefix(self, tts_node_cls):
        """Acceptance: лог TTS должен позволять восстановить любую фразу.

        До фикса лог обрезался на 50 символов (``text[:50]``) — длинные
        реплики (рэп/сказка) восстановить по логу было нельзя.
        """
        node = _make_node(tts_node_cls)
        text = (
            "Жил да был енотик полосатый, он катался по дороге и мурлыкал "
            "песенку про то, как хорошо быть енотиком весёлым"
        )
        node.dialogue_callback(
            _msg({"ssml": f"<speak>{text}</speak>", "speech_id": "sid-long-1"})
        )
        logs = _all_logs(node)
        assert text in logs, "полный текст обязан быть в логе (issue #1709)"
