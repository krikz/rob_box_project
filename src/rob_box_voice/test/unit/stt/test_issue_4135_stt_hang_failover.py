"""#4135 — зависший вызов STT больше не делает робота глухим.

28.10 07:08 UTC (акт 9, run 40648027547, образ dev-7ec08dd): стрим Yandex STT упал по ``DEADLINE_EXCEEDED``, stt_node
повис в ``close()`` старого gRPC-канала прямо в обработчике ошибки. Попытка провайдера не вернулась, цепочка
yandex → minimax → vosk 5 минут не получала управления, робот не слышал вейк до ``docker restart``.

Проверяем поведение, а не текст:

* сторож цепочки (``stt_fallback.select_recognition`` + ``HangGuard``): провайдер, который виснет, теряет фразу
  за ``timeout_s + hang_grace_s``, фраза уходит следующему; пока брошенный вызов не вернулся, провайдер не зовётся
  второй раз параллельно;
* ``HangGuard.release``: закрытие не держит вызывающего, висящих закрытий под одним именем не больше одного;
* stt_node: канал, у которого ``close()`` блокируется навсегда, не мешает ни этой фразе (уходит minimax), ни
  следующей (распознаёт новый канал Yandex); зависший ``_recognize_yandex`` отдаёт фразу minimax.

Без ROS: rclpy/std_msgs/vosk/grpc — заглушки, как в ``test_robot_speech_gates.py``.
"""

from __future__ import annotations

import importlib
import logging
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_voice.stt_fallback import (
    DEFAULT_HANG_GRACE_S,
    ProviderDeadCache,
    ProviderPolicy,
    STTTimeoutError,
    select_recognition,
    summarize_attempts,
)
from rob_box_voice.stt_hang_guard import HangGuard

install_dialog_msgs_stub()

AUDIO = b"\x10\x00" * 1600
TEXT = "робот который час"


@pytest.fixture
def hangs():
    """События, на которых висят подменные вызовы; после теста отпускаем, чтобы daemon-потоки ушли."""
    events: list[threading.Event] = []
    yield events
    for event in events:
        event.set()


def _wait_until(predicate, timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class _Provider:
    """Провайдер цепочки: первый вызов может висеть на ``block``, дальше отвечает ``text``."""

    def __init__(
        self,
        name: str,
        text=None,
        *,
        block: threading.Event | None = None,
        late_text: str = "поздно",
    ):
        self.name = name
        self._text = text
        self._block = block
        self._late_text = late_text
        self.calls = 0
        self.threads: list[str] = []

    def recognize(self, audio_bytes: bytes):
        self.calls += 1
        self.threads.append(threading.current_thread().name)
        if self._block is not None and self.calls == 1:
            self._block.wait()
            return self._late_text
        return self._text


def _fast_policies(**overrides) -> dict:
    policies = {
        "yandex": ProviderPolicy(
            timeout_s=0.05, max_retries=1, retry_backoff_s=0.0, hang_grace_s=0.05
        ),
        "minimax": ProviderPolicy(
            timeout_s=1.0, max_retries=0, retry_backoff_s=0.0, hang_grace_s=0.5
        ),
    }
    policies.update(overrides)
    return policies


# ── сторож цепочки ────────────────────────────────────────────────────────────────────────────────────────────────


def test_hung_provider_hands_phrase_to_next_within_guard_limit(hangs):
    hang = threading.Event()
    hangs.append(hang)
    yandex, minimax = _Provider("yandex", block=hang), _Provider("minimax", TEXT)
    messages: list[str] = []
    cache = ProviderDeadCache()

    started = time.monotonic()
    text, attempts = select_recognition(
        [yandex, minimax],
        AUDIO,
        policies=_fast_policies(),
        dead_cache=cache,
        guard=HangGuard(log=messages.append),
    )
    elapsed = time.monotonic() - started

    assert text == TEXT
    assert [(a.provider, a.reason) for a in attempts] == [
        ("yandex", "timeout"),
        ("minimax", "ok"),
    ], summarize_attempts(attempts)
    assert attempts[0].hung and "hung" in attempts[0].error
    assert (
        elapsed < 1.0
    ), elapsed  # предел yandex 0.05 + 0.05 с, а не «пока не вернётся»
    assert (
        not hang.is_set() and yandex.calls == 1
    )  # yandex ещё висит — цепочка его не ждала и не повторяла
    assert cache.is_dead("yandex") and cache.reason("yandex").startswith("timeout>")
    assert any("'yandex' не вернулся за 0.1 с" in m for m in messages), messages
    assert any(
        "'yandex' завис → фраза уходит 'minimax'" in m for m in messages
    ), messages


def test_still_hung_provider_is_skipped_until_it_returns(hangs):
    """Без кэша «мёртвых» (или когда TTL истёк) висящего провайдера не зовём параллельно второй раз."""
    hang = threading.Event()
    hangs.append(hang)
    yandex = _Provider("yandex", "робот привет", block=hang)
    minimax = _Provider("minimax", TEXT)
    messages: list[str] = []
    guard = HangGuard(log=messages.append)

    select_recognition([yandex, minimax], AUDIO, policies=_fast_policies(), guard=guard)
    text, attempts = select_recognition(
        [yandex, minimax], AUDIO, policies=_fast_policies(), guard=guard
    )

    assert text == TEXT
    assert [(a.provider, a.reason) for a in attempts] == [
        ("yandex", "hung"),
        ("minimax", "ok"),
    ]
    assert yandex.calls == 1
    assert any("'yandex': прошлая попытка висит" in m for m in messages), messages

    hang.set()
    assert _wait_until(lambda: guard.hung_for_s("yandex") is None)
    assert any("'yandex' вернулся через" in m for m in messages), messages

    text, attempts = select_recognition(
        [yandex, minimax], AUDIO, policies=_fast_policies(), guard=guard
    )
    assert text == "робот привет" and yandex.calls == 2
    assert [(a.provider, a.reason) for a in attempts] == [("yandex", "ok")]


def test_late_answer_within_grace_is_timeout_not_hang():
    """Вернулся позже timeout_s, но до предела сторожа: как и до #4135 — timeout, текст отброшен."""

    class _Slow:
        name = "yandex"

        def recognize(self, audio_bytes):
            time.sleep(0.15)
            return "поздний ответ"

    policies = _fast_policies(
        yandex=ProviderPolicy(timeout_s=0.05, max_retries=0, hang_grace_s=2.0)
    )
    text, attempts = select_recognition(
        [_Slow(), _Provider("minimax", TEXT)], AUDIO, policies=policies
    )

    assert text == TEXT
    assert [(a.provider, a.reason, a.hung) for a in attempts] == [
        ("yandex", "timeout", False),
        ("minimax", "ok", False),
    ]


def test_chain_without_guard_is_bounded_too(hangs):
    """``guard`` не передан — свежий сторож на вызов: ограничение то же, памяти между фразами нет."""
    hang = threading.Event()
    hangs.append(hang)
    started = time.monotonic()
    text, attempts = select_recognition(
        [_Provider("yandex", block=hang), _Provider("minimax", TEXT)],
        AUDIO,
        policies=_fast_policies(),
    )
    assert text == TEXT and attempts[0].hung
    assert time.monotonic() - started < 1.0


def test_default_grace_is_two_seconds():
    """Запас один и в одном месте (stt_fallback), у политики по умолчанию — он же."""
    assert DEFAULT_HANG_GRACE_S == 2.0
    assert ProviderPolicy().hang_grace_s == DEFAULT_HANG_GRACE_S


# ── фоновое закрытие ──────────────────────────────────────────────────────────────────────────────────────────────


def test_release_does_not_wait_and_keeps_one_hung_close_per_name(hangs):
    hang, entered = threading.Event(), threading.Event()
    hangs.append(hang)
    close_threads: list[str] = []
    messages: list[str] = []
    guard = HangGuard(log=messages.append)

    def stuck_close():
        close_threads.append(threading.current_thread().name)
        entered.set()
        hang.wait()

    started = time.monotonic()
    assert guard.release("yandex.channel.close", stuck_close, limit_s=0.05) is True
    assert time.monotonic() - started < 0.5
    assert entered.wait(1.0)
    assert close_threads[0] != threading.current_thread().name
    assert _wait_until(lambda: guard.hung_for_s("yandex.channel.close") is not None)

    second = MagicMock()
    assert guard.release("yandex.channel.close", second, limit_s=0.05) is False
    second.assert_not_called()
    assert any("прошлый вызов висит" in m for m in messages), messages

    hang.set()
    assert _wait_until(lambda: guard.hung_for_s("yandex.channel.close") is None)
    third = threading.Event()
    assert guard.release("yandex.channel.close", third.set, limit_s=1.0) is True
    assert third.wait(1.0)


def test_release_logs_close_error():
    messages: list[str] = []
    guard = HangGuard(log=messages.append)

    def broken_close():
        raise RuntimeError("channel already closed")

    assert guard.release("yandex.channel.close", broken_close, limit_s=1.0) is True
    assert _wait_until(
        lambda: any("channel already closed" in m for m in messages)
    ), messages


# ── stt_node ──────────────────────────────────────────────────────────────────────────────────────────────────────


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
    logger = logging.getLogger("test_issue_4135_stt_node")
    logger.setLevel(logging.INFO)
    handler = _Log()
    logger.addHandler(handler)
    yield logger, handler.lines
    logger.removeHandler(handler)


def _final(text: str):
    """Ответ стрима v3 «final» (поля как в stt.proto)."""
    return SimpleNamespace(
        WhichOneof=lambda _oneof: "final",
        final=SimpleNamespace(alternatives=[SimpleNamespace(text=text)]),
        audio_cursors=SimpleNamespace(final_index=0),
    )


def _stub(responses=(), error=None):
    """Стаб Yandex: стрим отдаёт ``responses``, потом (если задано) падает ``error``."""

    def streaming(gen, metadata=None, timeout=None):
        def _iter():
            yield from responses
            if error is not None:
                raise error

        return _iter()

    return SimpleNamespace(RecognizeStreaming=MagicMock(side_effect=streaming))


def _node(module, logger, *, chain, minimax_text=None, yandex_timeout_s=5.0):
    """STTNode без ROS: настоящая проводка цепочки (``_init_provider_chain_params``), Yandex — на стабе."""
    node = module.STTNode.__new__(module.STTNode)
    params = {
        "stt_provider_chain": chain,
        "minimax_stt_enabled": minimax_text is not None,
        "minimax_stt_api_key": "FAKE" if minimax_text is not None else "",
        "minimax_stt_api_key_env": "MINIMAX_API_KEY_UNSET_FOR_TESTS_4135",
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
    node.yandex_timeout_s, node.yandex_max_retries, node.retry_backoff_s = (
        yandex_timeout_s,
        1,
        0.0,
    )
    node.min_text_chars, node.sample_rate = 3, 16000
    node.yandex_api_key, node.yandex_model, node.yandex_language = (
        "FAKE",
        "general",
        "ru-RU",
    )
    node.eou_profile, node.eou_profiles = "balanced", {
        "balanced": {"type": 0, "max_pause_ms": 1000}
    }
    node.recognizer, node._vosk_available, node._last_speaker_tag = None, False, None
    node._offer_interrupt = lambda text: None
    if minimax_text is not None:
        node._recognize_minimax = lambda audio_bytes: minimax_text
    return node


def _channel_with_stuck_close(hangs):
    """gRPC-канал, у которого ``close()`` не возвращается (как 28.10): запоминает поток, из которого его закрыли."""
    hang, entered = threading.Event(), threading.Event()
    hangs.append(hang)
    channel = SimpleNamespace(closed_from=[])

    def close():
        channel.closed_from.append(threading.get_ident())
        entered.set()
        hang.wait()

    channel.close, channel.entered = close, entered
    return channel


def _install_new_channel(node, stub):
    """Подмена ``initialize_yandex``: новый канал и стаб (шов создания gRPC-канала)."""

    def initialize_yandex():
        node.yandex_channel, node.yandex_stub = (
            SimpleNamespace(close=lambda: None),
            stub,
        )

    node.initialize_yandex = initialize_yandex


def test_stuck_channel_close_does_not_block_phrase_minimax_takes_it(
    stt_node_module, log_lines, hangs
):
    """Повтор 28.10: DEADLINE_EXCEEDED, close() старого канала висит — фраза всё равно распознана (minimax)."""
    logger, lines = log_lines
    node = _node(
        stt_node_module, logger, chain=["yandex", "minimax", "vosk"], minimax_text=TEXT
    )
    old = _channel_with_stuck_close(hangs)
    node.yandex_channel, node.yandex_stub = old, _stub(
        error=_RpcError("DEADLINE_EXCEEDED")
    )
    new_stub = _stub([_final("робот привет")])
    _install_new_channel(node, new_stub)

    started = time.monotonic()
    text, attempts = node._recognize_with_fallback(AUDIO)
    elapsed = time.monotonic() - started

    assert text == TEXT
    assert [(a.provider, a.reason) for a in attempts] == [
        ("yandex", "timeout"),
        ("minimax", "ok"),
    ]
    assert elapsed < 1.5, elapsed  # close() висит до конца теста — фраза его не ждала
    assert old.entered.wait(1.0)  # старый канал закрывают...
    assert (
        old.closed_from and old.closed_from[0] != threading.get_ident()
    )  # ...не из потока фразы
    assert node.yandex_stub is new_stub
    assert any("пересоздаю канал Yandex STT" in m for m in lines), lines


def test_stuck_channel_close_next_phrase_recognized_by_new_channel(
    stt_node_module, log_lines, hangs
):
    """Только Yandex: эта фраза — без текста (таймаут), следующая — распознана новым каналом."""
    logger, _lines = log_lines
    node = _node(stt_node_module, logger, chain=["yandex"])
    node.yandex_channel, node.yandex_stub = _channel_with_stuck_close(hangs), _stub(
        error=_RpcError("DEADLINE_EXCEEDED")
    )
    _install_new_channel(node, _stub([_final(TEXT)]))

    text, attempts = node._recognize_with_fallback(AUDIO)
    assert text is None and [(a.provider, a.reason) for a in attempts] == [
        ("yandex", "timeout")
    ]

    started = time.monotonic()
    text, attempts = node._recognize_with_fallback(AUDIO)
    assert (
        text == TEXT
        and attempts[-1].provider == "yandex"
        and attempts[-1].reason == "ok"
    )
    assert time.monotonic() - started < 1.0


def test_hung_yandex_call_phrase_goes_to_minimax_within_guard(
    stt_node_module, log_lines, hangs
):
    """Вызов Yandex завис где угодно (не только в close) — фраза уходит minimax за timeout + запас сторожа."""
    logger, lines = log_lines
    node = _node(
        stt_node_module,
        logger,
        chain=["yandex", "minimax", "vosk"],
        minimax_text=TEXT,
        yandex_timeout_s=0.1,
    )
    node.yandex_channel, node.yandex_stub = SimpleNamespace(close=lambda: None), _stub()
    hang = threading.Event()
    hangs.append(hang)
    calls: list[int] = []

    def recognize_yandex(audio_bytes):
        calls.append(1)
        if len(calls) == 1:
            hang.wait()
            raise STTTimeoutError("поздно")
        return "робот привет"

    node._recognize_yandex = recognize_yandex

    started = time.monotonic()
    text, attempts = node._recognize_with_fallback(AUDIO)
    elapsed = time.monotonic() - started
    assert text == TEXT
    assert [(a.provider, a.reason, a.hung) for a in attempts] == [
        ("yandex", "timeout", True),
        ("minimax", "ok", False),
    ]
    assert (
        0.1 + DEFAULT_HANG_GRACE_S - 0.1 <= elapsed < 0.1 + DEFAULT_HANG_GRACE_S + 1.0
    ), elapsed
    assert any("'yandex' не вернулся за" in m for m in lines), lines
    assert any("'yandex' завис → фраза уходит 'minimax'" in m for m in lines), lines

    # Следующая фраза: yandex в кэше «мёртвых» (транзиентный таймаут) — сразу minimax, без ожидания.
    started = time.monotonic()
    text, attempts = node._recognize_with_fallback(AUDIO)
    assert text == TEXT and [a.reason for a in attempts] == ["dead", "ok"]
    assert time.monotonic() - started < 0.5 and len(calls) == 1

    # TTL истёк, а вызов всё ещё висит — второй раз параллельно не зовём.
    node._provider_dead_cache.mark_alive("yandex")
    text, attempts = node._recognize_with_fallback(AUDIO)
    assert (
        text == TEXT
        and [a.reason for a in attempts] == ["hung", "ok"]
        and len(calls) == 1
    )

    # Брошенный вызов вернулся — yandex снова в деле.
    hang.set()
    assert _wait_until(lambda: node._hang_guard.hung_for_s("yandex") is None)
    text, attempts = node._recognize_with_fallback(AUDIO)
    assert text == "робот привет" and len(calls) == 2
