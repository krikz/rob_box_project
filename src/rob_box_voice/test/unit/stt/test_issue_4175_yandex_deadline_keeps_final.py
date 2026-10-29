"""#4175 — Yandex STT выдал готовый final с EOU, но не закрыл стрим до дедлайна: фраза больше не выбрасывается.

Марафон-2, n301 (utterance 10a56723a3a9): Yandex прислал ``F#0 'робот мы тут сейчас немн…'`` и EOU, стрим не закрылся
до 5 с, ``DEADLINE_EXCEEDED`` выбросил собранное, фраза ушла в minimax → vosk-small, который «Робот» потерял.
Решение Шифу 28.10 18:49 UTC (вариант А): при дедлайне брать собранный непустой final, если фраза закрыта EOU.

Проверяем поведение, а не текст промпта:

* чистая функция ``text_kept_on_deadline`` (сегменты + тип ошибки → текст или ``None``);
* ``_recognize_yandex_phase`` на потоке с настоящим сборщиком сегментов;
* цепочка ``_recognize_with_fallback``: фраза принимается Yandex-ом (``ok`` + пометка ``deadline_final``) даже при
  ``elapsed > timeout_s`` (дедлайн gRPC срабатывает ровно на ``timeout_s``), фолбек не зовётся.

Без ROS: rclpy/std_msgs/vosk/grpc — заглушки, как в ``test_issue_4135_stt_hang_failover.py``.
"""

from __future__ import annotations

import importlib
import logging
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_voice.core.yandex_stt_segments import YandexSegmentCollector, text_kept_on_deadline
from rob_box_voice.stt_fallback import (
    NOTE_DEADLINE_FINAL,
    STTAttempt,
    STTDeadlineFinal,
    STTTimeoutError,
    summarize_attempts,
)

install_dialog_msgs_stub()

AUDIO = b"\x10\x00" * 1600
PHRASE = "робот мы тут сейчас немного пошумим"


# ── события потока ────────────────────────────────────────────────────────────────────────────────────────────────


def _event(kind: str, text: str = "", index: int = 0, start: int = 0, end: int = 0):
    """Ответ стрима v3 (поля как в stt.proto): ``partial`` / ``final`` / ``eou_update``."""
    resp = SimpleNamespace(
        WhichOneof=lambda _oneof: kind,
        audio_cursors=SimpleNamespace(final_index=index),
    )
    if kind in ("partial", "final"):
        alts = [SimpleNamespace(text=text, start_time_ms=start, end_time_ms=end)] if text else []
        setattr(resp, kind, SimpleNamespace(alternatives=alts))
    return resp


def _collector(*events) -> YandexSegmentCollector:
    segments = YandexSegmentCollector()
    for event in events:
        segments.feed(event, event.WhichOneof("Event"))
    return segments


# ── чистая функция: сегменты + тип ошибки → текст или None ──────────────────────────────────────────────────────────


def test_deadline_with_final_and_eou_keeps_text():
    """Итог n301: partial'ы, final, EOU, пустой хвостовой final — стрим не закрылся. Берём фразу."""
    segments = _collector(
        _event("partial", PHRASE, start=0, end=9920),
        _event("final", PHRASE, index=0, start=0, end=8696),
        _event("eou_update"),
        _event("final", "", index=1),
    )
    assert text_kept_on_deadline(segments, deadline_exceeded=True) == PHRASE


def test_deadline_with_only_partial_falls_back():
    """Final нет, только partial: фраза не подтверждена сервером - фолбек, как раньше."""
    segments = _collector(_event("partial", "робот"), _event("partial", PHRASE))
    assert text_kept_on_deadline(segments, deadline_exceeded=True) is None


def test_deadline_with_empty_final_and_eou_falls_back():
    """Пустой final + EOU: подтверждённого текста нет, partial сборщик подставил сам - не считается."""
    segments = _collector(_event("partial", PHRASE), _event("final", "", index=0), _event("eou_update"))
    assert segments.text() == PHRASE  # текст сборщик собрал бы...
    assert text_kept_on_deadline(segments, deadline_exceeded=True) is None  # ...но сервер его не подтвердил


def test_deadline_with_final_but_no_eou_falls_back():
    """Final без EOU: может быть сегмент посреди фразы (пауза), за которым идёт речь. Обрезанную фразу не берём."""
    segments = _collector(_event("partial", "робот"), _event("final", "робот", index=0, start=0, end=700))
    assert text_kept_on_deadline(segments, deadline_exceeded=True) is None


def test_deadline_with_eou_before_final_text_falls_back():
    """EOU пришёл раньше любого final с текстом - он ничего не закрывал."""
    segments = _collector(_event("eou_update"), _event("final", PHRASE, index=0, start=0, end=5000))
    assert text_kept_on_deadline(segments, deadline_exceeded=True) is None


def test_new_nonempty_final_after_eou_reopens_utterance():
    """final, EOU, потом ещё один непустой final без своего EOU: фраза продолжилась - не закрыта."""
    segments = _collector(
        _event("final", "робот привет", index=0, start=0, end=1500),
        _event("eou_update"),
        _event("final", "как дела", index=1, start=2000, end=3000),
    )
    assert text_kept_on_deadline(segments, deadline_exceeded=True) is None


def test_two_closed_segments_joined():
    segments = _collector(
        _event("final", "робот привет", index=0, start=0, end=1500),
        _event("eou_update"),
        _event("final", "как дела", index=1, start=2000, end=3000),
        _event("eou_update"),
    )
    assert text_kept_on_deadline(segments, deadline_exceeded=True) == "робот привет как дела"


def test_other_grpc_error_keeps_old_behaviour():
    """Не дедлайн (UNAVAILABLE и т.п.): даже закрытая фраза не берётся - прежнее поведение."""
    segments = _collector(_event("final", PHRASE, index=0, start=0, end=5000), _event("eou_update"))
    assert text_kept_on_deadline(segments, deadline_exceeded=False) is None


def test_empty_stream_gives_none():
    assert text_kept_on_deadline(YandexSegmentCollector(), deadline_exceeded=True) is None


def test_attempt_note_is_in_summary_and_log_dict():
    attempt = STTAttempt(provider="yandex", reason="ok", latency_ms=5029, text=PHRASE, note=NOTE_DEADLINE_FINAL)
    assert summarize_attempts([attempt]).startswith("yandex:ok(5029ms deadline_final '")
    assert attempt.to_log_dict()["note"] == "deadline_final"
    assert "note" not in STTAttempt(provider="yandex", reason="ok", latency_ms=1, text="робот").to_log_dict()


# ── stt_node: настоящая проводка ──────────────────────────────────────────────────────────────────────────────────


class _RosNode:
    def __init__(self, *a, **kw):
        pass


class _RpcError(Exception):
    def __init__(self, code: str):
        super().__init__(code)
        self._code = code

    def code(self):
        return self._code

    def details(self):
        return "Deadline Exceeded"


def _forget_stt_node() -> None:
    import rob_box_voice

    sys.modules.pop("rob_box_voice.stt_node", None)
    if hasattr(rob_box_voice, "stt_node"):
        delattr(rob_box_voice, "stt_node")


@pytest.fixture
def stt_node_module(monkeypatch):
    grpc_stub = MagicMock()
    grpc_stub.RpcError = _RpcError
    grpc_stub.StatusCode = SimpleNamespace(
        DEADLINE_EXCEEDED="DEADLINE_EXCEEDED",
        UNAUTHENTICATED="UNAUTHENTICATED",
        PERMISSION_DENIED="PERMISSION_DENIED",
        RESOURCE_EXHAUSTED="RESOURCE_EXHAUSTED",
        UNAVAILABLE="UNAVAILABLE",
    )
    stubs = {
        "rclpy": SimpleNamespace(init=None, shutdown=None, spin=None, ok=lambda: False),
        "rclpy.node": SimpleNamespace(Node=_RosNode),
        "rclpy.qos": MagicMock(),
        "std_msgs": MagicMock(),
        "std_msgs.msg": MagicMock(),
        "audio_common_msgs": MagicMock(),
        "audio_common_msgs.msg": MagicMock(),
        "vosk": MagicMock(),
        "grpc": grpc_stub,
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    _forget_stt_node()
    yield importlib.import_module("rob_box_voice.stt_node")
    _forget_stt_node()


class _Log(logging.Handler):
    def __init__(self):
        super().__init__()
        self.lines: list[str] = []

    def emit(self, record):
        self.lines.append(record.getMessage())


@pytest.fixture
def log_lines():
    logger = logging.getLogger("test_issue_4175_stt_node")
    logger.setLevel(logging.INFO)
    handler = _Log()
    logger.addHandler(handler)
    yield logger, handler.lines
    logger.removeHandler(handler)


def _stub(events=(), error=None, delay_s: float = 0.0):
    """Стаб Yandex: стрим отдаёт ``events``, ждёт ``delay_s`` (дедлайн «не закрыл стрим») и падает ``error``."""

    def streaming(gen, metadata=None, timeout=None):
        def _iter():
            yield from events
            if delay_s:
                time.sleep(delay_s)
            if error is not None:
                raise error

        return _iter()

    return SimpleNamespace(RecognizeStreaming=MagicMock(side_effect=streaming))


def _node(module, logger, *, chain, minimax_text=None, yandex_timeout_s=5.0):
    """STTNode без ROS: настоящая проводка цепочки, Yandex — на стабе."""
    node = module.STTNode.__new__(module.STTNode)
    params = {
        "stt_provider_chain": chain,
        "minimax_stt_enabled": minimax_text is not None,
        "minimax_stt_api_key": "FAKE" if minimax_text is not None else "",
        "minimax_stt_api_key_env": "MINIMAX_API_KEY_UNSET_FOR_TESTS_4175",
        "minimax_stt_timeout_s": 1.0,
        "minimax_stt_max_retries": 0,
        "provider_dead_ttl_s": 300.0,
        "provider_dead_ttl_transient_s": 30.0,
        "stt_phrase_budget_s": 20.0,
        "provider_state_file": "",
    }
    node.get_parameter = lambda name: SimpleNamespace(value=params.get(name))
    node.get_logger = lambda: logger
    node._init_provider_chain_params()
    node.yandex_timeout_s, node.yandex_max_retries, node.retry_backoff_s = yandex_timeout_s, 0, 0.0
    node.min_text_chars, node.sample_rate = 3, 16000
    node.yandex_api_key, node.yandex_model, node.yandex_language = "FAKE", "general", "ru-RU"
    node.eou_profile, node.eou_profiles = "balanced", {"balanced": {"type": 0, "max_pause_ms": 1000}}
    node.recognizer, node._vosk_available, node._last_speaker_tag = None, False, None
    node._offer_interrupt = lambda text: None
    node.yandex_channel = SimpleNamespace(close=lambda: None)
    node.initialize_yandex = lambda: None  # шов создания gRPC-канала: сеть в тесте не нужна
    if minimax_text is not None:
        node._minimax_calls = []
        node._recognize_minimax = lambda audio_bytes: node._minimax_calls.append(1) or minimax_text
    return node


def _closed_phrase_events():
    return [
        _event("partial", PHRASE, start=0, end=9920),
        _event("final", PHRASE, index=0, start=0, end=8696),
        _event("eou_update"),
        _event("final", "", index=1),
    ]


def test_phase_raises_deadline_final_with_text_when_utterance_closed(stt_node_module, log_lines):
    logger, lines = log_lines
    node = _node(stt_node_module, logger, chain=["yandex"])
    node.yandex_stub = _stub(_closed_phrase_events(), error=_RpcError("DEADLINE_EXCEEDED"))

    with pytest.raises(STTDeadlineFinal) as caught:
        node._recognize_yandex_phase(AUDIO, phase="REAL_TIME", enable_speech_analysis=True)

    assert caught.value.text == PHRASE
    assert isinstance(caught.value, STTTimeoutError)  # для кода, не знающего про #4175, это всё тот же таймаут
    # Канал по-прежнему пересоздаётся (#2924): зависший стрим - не повод держать то же соединение.
    assert any("пересоздаю канал Yandex STT" in m for m in lines), lines
    assert any("stream=[" in m and "F#0" in m and " E " in m for m in lines), lines  # трасса стрима осталась в логе


@pytest.mark.parametrize(
    "events",
    [
        [_event("partial", "робот"), _event("partial", PHRASE)],  # только partial
        [_event("partial", "робот"), _event("final", "робот", index=0, start=0, end=700)],  # final без EOU
    ],
    ids=["only-partial", "final-without-eou"],
)
def test_phase_keeps_plain_timeout_when_utterance_not_closed(stt_node_module, log_lines, events):
    logger, _lines = log_lines
    node = _node(stt_node_module, logger, chain=["yandex"])
    node.yandex_stub = _stub(events, error=_RpcError("DEADLINE_EXCEEDED"))

    with pytest.raises(STTTimeoutError) as caught:
        node._recognize_yandex_phase(AUDIO, phase="REAL_TIME", enable_speech_analysis=True)

    assert not isinstance(caught.value, STTDeadlineFinal)


def test_phase_other_grpc_error_passes_through_even_with_closed_phrase(stt_node_module, log_lines):
    logger, _lines = log_lines
    node = _node(stt_node_module, logger, chain=["yandex"])
    error = _RpcError("UNAVAILABLE")
    node.yandex_stub = _stub(_closed_phrase_events(), error=error)

    with pytest.raises(_RpcError) as caught:
        node._recognize_yandex_phase(AUDIO, phase="REAL_TIME", enable_speech_analysis=True)

    assert caught.value is error


def test_chain_accepts_yandex_phrase_past_timeout_without_fallback(stt_node_module, log_lines):
    """Главное: дедлайн сработал ровно на ``timeout_s`` (elapsed > timeout) - фраза всё равно Yandex-а, minimax не зван."""
    logger, lines = log_lines
    node = _node(stt_node_module, logger, chain=["yandex", "minimax", "vosk"], minimax_text="чужой текст", yandex_timeout_s=0.05)
    node.yandex_stub = _stub(_closed_phrase_events(), error=_RpcError("DEADLINE_EXCEEDED"), delay_s=0.07)

    text, attempts = node._recognize_with_fallback(AUDIO)

    assert text == PHRASE
    assert [(a.provider, a.reason, a.note) for a in attempts] == [("yandex", "ok", NOTE_DEADLINE_FINAL)]
    assert attempts[0].latency_ms > 50  # дедлайн действительно прошёл: без #4175 это был бы timeout
    assert node._minimax_calls == []
    assert "yandex:ok(" in summarize_attempts(attempts) and "deadline_final" in summarize_attempts(attempts)


def test_chain_still_falls_back_when_phrase_not_closed(stt_node_module, log_lines):
    logger, _lines = log_lines
    node = _node(stt_node_module, logger, chain=["yandex", "minimax", "vosk"], minimax_text="робот из minimax", yandex_timeout_s=0.05)
    node.yandex_stub = _stub(
        [_event("partial", "робот"), _event("final", "робот", index=0, start=0, end=700)],
        error=_RpcError("DEADLINE_EXCEEDED"),
        delay_s=0.07,
    )

    text, attempts = node._recognize_with_fallback(AUDIO)

    assert text == "робот из minimax"
    assert [(a.provider, a.reason, a.note) for a in attempts] == [("yandex", "timeout", None), ("minimax", "ok", None)]
