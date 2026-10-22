"""#3834: готовая реплика ``commit=true`` не стоит в очереди tts_node за пред-синтезом ``commit=false``.

Живой прогон PR-7c (#3700, 22.10): «и про енота» на сегменте 2 → пред-синтез правок [3, 4, 5]; сегмент 3
(``commit=true, at=такт``) ждал в общем пуле синтеза (2 потока) за этими правками — дыра 3.5 с посреди рэпа.

Тест гонит настоящий путь ``tts_node``: ``SpeechOwner.on_request`` → ``dialogue_callback`` → ``_submit_synthesis``
(пул из двух потоков, как ``SYNTHESIS_MAX_WORKERS_DEFAULT``) → ``_run_synthesis_worker`` →
``_synthesize_and_play`` (готовое аудио пред-синтеза, FIFO-ворота) → ``SpeechOwner.started``. Подменены только
провайдер (синтез длится ``SYNTH_S``) и динамик (``_sap_local_playback`` без ALSA).
"""

from __future__ import annotations

import concurrent.futures
import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_core.msg_stub import install_dialog_msgs_stub  # noqa: E402

M = install_dialog_msgs_stub()

from rob_box_voice.core.speech_owner import SpeechOwner, owner_of  # noqa: E402
from rob_box_voice.core.speech_timing import Presynth  # noqa: E402
from rob_box_voice.tts_node import SYNTHESIS_MAX_WORKERS_DEFAULT, TTSNode  # noqa: E402

#: Сколько длится синтез одного сегмента у фейкового провайдера.
SYNTH_S = 0.3
#: Сколько «звучит» сегмент в фейковом динамике.
PLAY_S = 0.05
#: Порог карточки: от ``commit=true`` с наступившим ``at`` до ``started``.
STARTED_WITHIN_S = 0.05
_KIND = {getattr(M.SpeechEvent, f"KIND_{n.upper()}"): n for n in (
    "queued", "synthesized", "started", "progress", "finished", "cancelled", "voice_changed", "rejected")}


class _Events:
    """Публикатор ``/voice/speech/event``: ``(monotonic, kind, speech_id)``."""

    def __init__(self):
        self.sent = []
        self.lock = threading.Lock()

    def publish(self, ev):
        with self.lock:
            self.sent.append((time.monotonic(), _KIND[ev.kind], ev.speech_id))

    def first(self, kind, speech_id):
        with self.lock:
            return next((t for t, k, s in self.sent if k == kind and s == speech_id), None)

    def of(self, kind):
        with self.lock:
            return [s for _, k, s in self.sent if k == kind]


class _Clock:
    def now(self):
        ns = time.time_ns()
        return MagicMock(nanoseconds=ns, to_msg=lambda: M.SpeechEvent().stamp)


class _Node(TTSNode):
    """``TTSNode`` без тяжёлого ``__init__``: настоящие очередь синтеза и FIFO-ворота, фейковые провайдер и динамик."""

    def __init__(self):  # noqa: D401 — тестовая заглушка, без TTSNode.__init__
        super(TTSNode, self).__init__("tts_node")  # FakeNode: publishers/subscriptions/timers
        self._logger = MagicMock()
        self._play_seq_counter, self._next_play_seq = 0, 1
        self._play_order_cond = threading.Condition()
        self._pending_seqs, self._play_active_seq, self._active_batch_id = {}, None, None
        self._synthesis_executor_shutdown, self._synthesis_in_flight = False, 0
        self._synthesis_executor_max_workers = SYNTHESIS_MAX_WORKERS_DEFAULT
        self._synthesis_slots = threading.Semaphore(16 + SYNTHESIS_MAX_WORKERS_DEFAULT)
        self._synthesis_executor = concurrent.futures.ThreadPoolExecutor(max_workers=SYNTHESIS_MAX_WORKERS_DEFAULT)
        self.finished_pub = MagicMock()
        self.normalize_text, self.minimax_streaming, self.stop_requested = False, False, False
        self.provider_chain, self.minimax_voice = ["minimax"], "male-qn-qingse"
        self.synthesized_texts = []
        self.synth_gate = threading.Event()
        self.synth_gate.set()
        self._speech = SpeechOwner(self, output_latency_ms=0)
        self._speech.attach()
        self.events = self._speech._event_pub = _Events()
        self._speech._envelope_pub = MagicMock()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def publish_state(self, state):
        pass

    # -- фейки: провайдер и динамик ------------------------------------------------

    def _sap_run_provider_chain(self, voice, language, attrs, text, sink, ctx):
        self.synth_gate.wait(5.0)
        time.sleep(SYNTH_S)
        self.synthesized_texts.append(text)
        ctx.update(audio_np=np.zeros(1600, dtype=np.float32), sample_rate=16000, used_provider="minimax")

    def _sap_publish_for_sink(self, *args):
        pass

    def _sap_local_playback(self, audio_np, sample_rate, text, voice, play_seq, speech_id, *rest):
        if not owner_of(self).started(speech_id, np.column_stack((audio_np, audio_np)), 16000):
            self._release_play_seq(play_seq, speech_id=speech_id)
            self._publish_tts_finished(speech_id, success=False, error="cancelled")
            return
        time.sleep(PLAY_S)
        self._release_play_seq(play_seq, speech_id=speech_id)
        self._publish_tts_finished(speech_id, success=True)

    def close(self):
        self.synth_gate.set()
        self.shutdown_synthesis_executor(wait=True)


@pytest.fixture
def node():
    n = _Node()
    yield n
    n.close()


def _say(node, speech_id, text, *, commit, at=0.0, epoch=1):
    req = M.SpeechRequest()
    req.speech_id, req.text, req.agent, req.epoch = speech_id, text, "personality", epoch
    req.commit, req.at, req.interruptible = commit, at, True
    node._speech.on_request(req)


def _wait(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def _presynth_ready(node, *speech_ids):
    for sid in speech_ids:
        _say(node, sid, f"Куплет {sid}.", commit=False)
    assert _wait(lambda: all(sid in node.events.of("synthesized") for sid in speech_ids))


def test_commit_with_due_at_starts_at_once_behind_a_presynth_queue(node):
    """Правка «и про енота»: шесть ``commit=false`` в очереди синтеза — готовый сегмент всё равно звучит сразу."""
    _presynth_ready(node, "seg-3")
    for n in range(4, 10):
        _say(node, f"edit-{n}", f"Новый куплет {n} про енота.", commit=False)
    time.sleep(0.02)  # правки разобраны пулом пред-синтеза, синтез идёт

    t_commit = time.monotonic()
    _say(node, "seg-3", "Куплет seg-3.", commit=True, at=time.time())
    assert _wait(lambda: node.events.first("started", "seg-3") is not None)

    delay = node.events.first("started", "seg-3") - t_commit
    assert delay <= STARTED_WITHIN_S, f"started через {delay * 1000:.0f} мс после commit=true"
    assert "Куплет seg-3." not in node.synthesized_texts[1:]  # звучит пред-синтез, а не второй синтез


def test_commit_order_is_kept_while_presynth_runs(node):
    """Порядок звучания ``commit=true`` — порядок их запросов, пока пул пред-синтеза занят."""
    _presynth_ready(node, "a", "b", "c")
    for n in range(3):
        _say(node, f"edit-{n}", f"Правка {n}.", commit=False)
    for sid in ("a", "b", "c"):
        _say(node, sid, f"Куплет {sid}.", commit=True)
    assert _wait(lambda: len(node.events.of("finished")) == 3)
    assert node.events.of("started") == ["a", "b", "c"]


def test_epoch_cancel_clears_presynth_queue_held_audio_and_waiting_commit(node):
    """«Стой» по эпохе: ждущий пред-синтез не синтезируется, готовый выброшен, ждущий ``commit=true`` молчит."""
    _presynth_ready(node, "held", "waiting")
    node.synth_gate.clear()  # провайдер «завис»: оба потока пред-синтеза заняты, остальное ждёт в очереди
    for n in range(4):
        _say(node, f"edit-{n}", f"Правка {n}.", commit=False)
    _say(node, "waiting", "Куплет waiting.", commit=True, at=time.time() + 5.0)
    assert _wait(lambda: "waiting" in node.events.of("queued"))
    synthesized_before = len(node.synthesized_texts)

    cancel = M.SpeechCancel()
    cancel.epoch, cancel.agent, cancel.reason = 1, "personality", "stop"
    node._speech.on_cancel(cancel)
    node.synth_gate.set()

    everything = {"held", "waiting", "edit-0", "edit-1", "edit-2", "edit-3"}
    assert _wait(lambda: everything <= set(node.events.of("cancelled")))
    assert node._speech.claim_presynth("held") is None
    assert "waiting" not in node.events.of("started")
    # Синтез завершили только два потока, взявшие правки до отмены; ждавшие в очереди — не синтезировались.
    assert len(node.synthesized_texts) - synthesized_before <= 2
    assert all(node._speech.claim_presynth(f"edit-{n}") is None for n in range(4))


def test_presynth_runs_on_its_own_pool_not_the_synthesis_executor(node):
    """Структура фикса: пред-синтез не занимает потоки, на которых ``commit=true`` ждёт ``at`` и играет."""
    submitted = []
    node._synthesis_executor = MagicMock(submit=lambda *a, **kw: submitted.append(a))
    _say(node, "s", "Куплет.", commit=False)
    assert _wait(lambda: "s" in node.events.of("synthesized"))
    assert submitted == []


def test_shutdown_stops_the_presynth_pool(node):
    node.shutdown_synthesis_executor(wait=True)
    _say(node, "late", "Поздно.", commit=False)
    assert node.events.of("rejected") == ["late"]


def test_presynth_with_injected_executor():
    calls = []
    presynth = Presynth(executor=MagicMock(submit=lambda fn, *a: calls.append(fn)))
    stub = MagicMock(minimax_streaming=False)
    presynth.run(stub, {"speech_id": "x", "ssml": "<speak>x</speak>"}, MagicMock(), lambda sid: False)
    assert len(calls) == 1
