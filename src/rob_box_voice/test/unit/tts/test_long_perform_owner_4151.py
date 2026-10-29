"""#4151: длинное выступление через настоящего владельца речи — ``PerformHost`` ↔ ``SpeechOwner`` и конвейер tts_node.

Сказка n412 (12 сегментов) и счёт n802 (11) 28.10 не звучали: хост отдавал пред-синтез всех сегментов одной пачкой, а
подписка владельца ``/voice/speech/request`` — KEEP_LAST той глубины, что просит ``SpeechOwner.attach`` (10). Пачка
больше неё теряет самые старые запросы — первые сегменты; их ``synthesized`` не приходит, а ``SegmentList.due`` ждёт
готовый сегмент — тишина под продлеваемую подложку.

Здесь хост выступления говорит с настоящим ``SpeechOwner`` и конвейером ``tts_node`` (провайдер и динамик — фейки
:mod:`test_commit_not_behind_presynth`) через провод с очередью подписки владельца: что хост отдал между двумя разборами
очереди (однопоточный executor ноды), — одна пачка, из неё доходят последние ``depth``. Это модель очереди, не замер
rmw_zenoh: на роботе executor успевал взять один запрос из пачки (дошли 11 из 13), здесь — ни одного.
"""

from __future__ import annotations

import re
import sys
import threading
import time
import types
from pathlib import Path
from typing import Any, Callable, Dict, List, Tuple

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.test_commit_not_behind_presynth import _KIND, M, _Node  # noqa: E402

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog import phrases  # noqa: E402
from rob_box_dialog.command import Performance  # noqa: E402
from rob_box_voice.core.perform_host import PerformHost  # noqa: E402

REQUEST, CANCEL = "/voice/speech/request", "/voice/speech/cancel"
#: Сколько ждать исхода выступления (стенные секунды): 12 сегментов по 0.1 с звучат за ~2 с.
TIMEOUT_S = 20.0


def _segments(n: int) -> Tuple[str, ...]:
    return tuple(f"Часть {i}: шла Красная Шапочка лесом к бабушке." for i in range(1, n + 1))


def _text(msg: Any) -> str:
    return re.sub(r"<[^>]+>", "", msg.text).strip()


class _Wire:
    """Провод dialog → tts: публикатор хоста копит запросы, executor владельца разбирает очередь подписки KEEP_LAST
    ``depth[topic]`` — из накопленной пачки доходят последние ``depth``, старшие выброшены."""

    def __init__(self, depth: Dict[str, int]) -> None:
        self.depth = depth
        self.pending: Dict[str, List[Any]] = {topic: [] for topic in depth}
        self.lost: List[Any] = []
        self.requests: Dict[str, Any] = {}  # speech_id → последний SpeechRequest хоста (дошёл он или нет)

    def publisher(self, topic: str) -> Any:
        def publish(msg: Any) -> None:
            self.pending[topic].append(msg)
            if topic == REQUEST:
                self.requests[msg.speech_id] = msg

        return types.SimpleNamespace(publish=publish)

    def deliver(self, topic: str) -> List[Any]:
        burst, self.pending[topic] = self.pending[topic], []
        cut = max(0, len(burst) - self.depth[topic])
        self.lost += burst[:cut]
        return burst[cut:]


class _DialogNode:
    """Нода dialog v2 для хоста выступления: публикаторы ``speech_client`` — в провод, ROS-часы — стенные."""

    def __init__(self, wire: _Wire) -> None:
        self._wire = wire
        self.logs: List[str] = []

    def create_publisher(self, msg_type: Any, topic: str, depth: int) -> Any:
        return self._wire.publisher(topic)

    def get_clock(self) -> Any:
        ns = time.time_ns()
        now = types.SimpleNamespace(nanoseconds=ns, to_msg=lambda: M.SpeechEvent().stamp)
        return types.SimpleNamespace(now=lambda: now)

    def get_logger(self) -> Any:
        return types.SimpleNamespace(info=self.logs.append)


class _Events:
    """``/voice/speech/event`` владельца: события копятся из потоков синтеза, хост получает их в своём потоке."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._inbox: List[Tuple[str, str, int, int]] = []
        self.log: List[Tuple[str, str]] = []

    def publish(self, ev: Any) -> None:
        with self._lock:
            self._inbox.append((_KIND[ev.kind], ev.speech_id, int(ev.duration_ms), int(ev.spoken_chars)))
            self.log.append((_KIND[ev.kind], ev.speech_id))

    def take(self) -> List[Tuple[str, str, int, int]]:
        with self._lock:
            items, self._inbox = self._inbox, []
        return items

    def of(self, kind: str) -> List[str]:
        with self._lock:
            return [sid for k, sid in self.log if k == kind]


def _subscription_depth(node: Any, topic: str) -> int:
    """Глубина очереди подписки, которую владелец речи попросил у ноды (``SpeechOwner.attach``)."""
    return next(int(args[3]) for args, _ in node._subscriptions if args[1] == topic)


class _Stand:
    """Хост выступления + настоящий владелец речи; :meth:`run` — однопоточный «executor» обеих нод."""

    def __init__(self) -> None:
        self.tts = _Node()
        self.tts.synth_s = 0.02
        self.events = _Events()
        self.tts._speech._event_pub = self.events
        self.wire = _Wire({topic: _subscription_depth(self.tts, topic) for topic in (REQUEST, CANCEL)})
        self.plan: List[Any] = []
        self.host = PerformHost(_DialogNode(self.wire), lambda call: None, lambda: None, self.plan.append, 0)

    def run(self, until: Callable[[], bool]) -> bool:
        deadline = time.monotonic() + TIMEOUT_S
        while time.monotonic() < deadline:
            for msg in self.wire.deliver(REQUEST):
                self.tts._speech.on_request(msg)
            for msg in self.wire.deliver(CANCEL):
                self.tts._speech.on_cancel(msg)
            for kind, speech_id, duration_ms, spoken_chars in self.events.take():
                self.host.on_speech(kind, speech_id, duration_ms, spoken_chars)
            self.host.tick()
            if until():
                return True
            time.sleep(0.005)
        return False

    def texts(self, kind: str) -> List[str]:
        """Тексты реплик хоста, о которых владелец речи сообщил событие ``kind`` (по порядку событий)."""
        return [_text(self.wire.requests[sid]) for sid in self.events.of(kind) if sid in self.wire.requests]

    def plan_events(self, kind: str) -> List[Any]:
        return [e for e in self.plan if e.kind == kind]


@pytest.fixture
def stand():
    s = _Stand()
    yield s
    s.tts.close()


def _tale(n: int) -> Performance:
    return Performance(kind="tale", segments=_segments(n), backing="none")


@pytest.mark.parametrize("n", [12, 11, 4])
def test_long_tale_reaches_the_real_speech_owner_and_sounds_in_order(stand, n):
    """12 сегментов (n412), 11 (n802) и короткая (n806): каждый запрос доходит до владельца речи, сегменты звучат
    по порядку, конец ``done``."""
    assert stand.wire.depth == {REQUEST: 10, CANCEL: 10}  # подписки владельца речи (28.10)
    stand.host.start(_tale(n), "t1")
    assert stand.run(lambda: stand.host.block.ended)
    assert stand.wire.lost == []
    assert stand.texts("started") == list(_segments(n))
    # каждый сегмент синтезирован провайдером один раз — пред-синтезом; звучание взяло готовое, а не синтез в пути
    assert sorted(stand.tts.synthesized_texts) == sorted(_segments(n))
    assert [e.segment_no for e in stand.plan_events("segment_started")] == list(range(1, n + 1))
    assert [e.reason for e in stand.plan_events("block_end")] == ["done"]


@pytest.mark.parametrize("n, lost", [(12, [1, 2]), (11, [1])])
def test_without_the_window_the_owner_queue_loses_first_segments_and_the_watchdog_ends_honestly(
        stand, monkeypatch, n, lost):
    """Воспроизведение 28.10 на настоящем владельце: окно шире очереди (как до #4151) — пачка пред-синтеза больше
    подписки, первые сегменты до владельца не доходят, выступление молчит. Сторож снимает его фразой кода, которую
    владелец произносит; ни один сегмент не звучал."""
    monkeypatch.setattr(kn, "PERFORM_PRESYNTH_WINDOW", 100)
    monkeypatch.setattr(kn, "PERFORM_SILENCE_MAX_S", 0.5)
    stalled = phrases.text("perform.stalled")
    stand.host.start(_tale(n), "t1")
    assert stand.run(lambda: stalled in stand.texts("finished"))
    missing = [_segments(n)[i - 1] for i in lost]
    assert [_text(m) for m in stand.wire.lost if hasattr(m, "commit") and not m.commit] == missing
    queued = [text for text in stand.texts("queued") if text in _segments(n)]
    assert sorted(queued) == sorted(set(_segments(n)) - set(missing))  # владелец принял остальные, как queued ×11
    assert set(_segments(n)).isdisjoint(stand.texts("started"))
    assert stand.texts("started") == [stalled]
    assert [e.reason for e in stand.plan_events("block_end")] == ["stalled"]
