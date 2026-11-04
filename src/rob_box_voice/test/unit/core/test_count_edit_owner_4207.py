"""#4207 на двойнике владельца речи: правка звучащего счёта режет его на границе числа и продолжает в новой форме.

Сухой стенд гонку не ловит: его ``settle`` досказывает реплику сразу. Здесь реплики хода идут настоящими портами
(``V2Ports.say``/``cancel_speech`` → ``SpeechRequest``/``SpeechCancel``) в настоящий ``SpeechOwner`` с журналом на
часах теста; воспроизведение - очередь стенда (одна реплика звучит, следующие ждут), ``SpeechEvent`` идут обратно в
``TurnEngine``. Живой лог 04.11 04:07Z: «Один, два, … двадцать» (192 знака), через ~15 с правка - модель прислала
«Один шаг, два шага, …» с единицы, и он прозвучал целиком после старого.
"""

from __future__ import annotations

import asyncio
import json
import types
from typing import Any, List, Optional
from unittest.mock import MagicMock

import numpy as np

from rob_box_core import speech_client
from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()

from rob_box_dialog import counting  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.command import Command  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.turn import Answer, TurnEngine  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports  # noqa: E402
from rob_box_voice.core.speech_owner import SpeechLedger, SpeechOwner  # noqa: E402

TIME = type(M.SpeechEvent().stamp)
N806 = "посчитай вслух от одного до двадцати спокойно"
N807 = "и после каждого числа добавляй слово шаг"
COUNT = ", ".join(counting.say(n) for n in range(1, 21)).capitalize() + "."
EDIT = ", ".join(f"{counting.say(n)} {'шаг' if n == 1 else 'шага' if n < 5 else 'шагов'}"
                 for n in range(1, 21)).capitalize() + "."
#: Секунд звучания на знак: 192 знака счёта - ~17 с, как в живом логе.
PER_CHAR_S = 0.09
STEP_S = 0.05


class _Pub:
    def __init__(self) -> None:
        self.sent: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.sent.append(msg)


class _Node:
    """Одна нода на обе стороны: публикаторы портов хода и всё, чем ``SpeechOwner`` пользуется у ``tts_node``."""

    def __init__(self) -> None:
        self.pubs: dict = {}
        self.now_s = 1000.0
        self.interrupted = False
        self.chunks: List[dict] = []

    def create_publisher(self, msg_type: Any, topic: str, depth: int) -> _Pub:
        return self.pubs.setdefault(topic, _Pub())

    def create_subscription(self, *args: Any) -> None:
        return None

    def create_timer(self, *args: Any) -> None:
        return None

    def get_clock(self) -> Any:
        ns = int(self.now_s * 1e9)
        stamp = TIME(sec=ns // 10**9, nanosec=ns % 10**9)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=ns, to_msg=lambda: stamp))

    def get_logger(self) -> Any:
        return MagicMock()

    def dialogue_callback(self, msg: Any) -> None:
        self.chunks.append(json.loads(msg.data))

    def _interrupt_playback(self) -> None:
        self.interrupted = True


class _Ports:
    """Порты хода: ответы LLM по очереди; речь и отмена - настоящие ``V2Ports`` (их сигнатуры видит ход)."""

    def __init__(self, node: _Node, answers: List[Command]) -> None:
        self.answers = [Answer(cmd, "deepseek") for cmd in answers]
        v2 = V2Ports(node, None, CallCounter(), None)
        self.say, self.cancel_speech = v2.say, v2.cancel_speech

    async def ask(self, messages: Any, tools: Any, timeout_s: float) -> Answer:
        return self.answers.pop(0)

    async def execute(self, actions: Any, snapshot: Any, turn_id: str) -> list:
        return []

    def stage(self, *args: Any, **kwargs: Any) -> None:
        pass

    def stop_perform(self, how: str) -> bool:
        return False

    def intent(self, *args: Any) -> None:
        pass

    def perform(self, *args: Any) -> None:
        return None


class _Stand:
    """Владелец речи с очередью воспроизведения; ``heard`` - что прозвучало (текст до ``spoken_chars``)."""

    def __init__(self, answers: List[Command]) -> None:
        self.node = _Node()
        self.owner = SpeechOwner(self.node, ledger=SpeechLedger(clock=lambda: self.node.now_s))
        self.owner.attach()
        self.ports = _Ports(self.node, answers)
        self.engine = TurnEngine(Session("personality"), self.ports, system_prompt="Ты робот.")
        self.texts: dict = {}
        self.queue: List[str] = []
        self.playing: Optional[str] = None
        self.heard: List[str] = []

    def turn(self, text: str) -> None:
        asyncio.run(self.engine.run(Addressed(text, VOICE)))
        self._deliver()

    def run(self, seconds: float) -> None:
        end = self.node.now_s + seconds
        while self.node.now_s < end:
            self.node.now_s += STEP_S
            self.owner.tick()
            self._play()
            self._deliver()

    def _deliver(self) -> None:
        """Запросы и отмены портов - владельцу; новые реплики - в очередь воспроизведения."""
        requests, self.node.pubs[speech_client.SPEECH_REQUEST_TOPIC].sent = \
            self.node.pubs[speech_client.SPEECH_REQUEST_TOPIC].sent, []
        for msg in requests:
            self.owner.on_request(msg)
        for chunk in self.node.chunks:
            self.owner.admit(chunk, chunk["speech_id"])
            self.texts[chunk["speech_id"]] = chunk["text"]
            self.queue.append(chunk["speech_id"])
        self.node.chunks = []
        cancels = self.node.pubs.get(speech_client.SPEECH_CANCEL_TOPIC)
        for msg in (cancels.sent if cancels else []):
            self.owner.on_cancel(msg)
        if cancels:
            cancels.sent = []
        self._events()

    def _play(self) -> None:
        if self.playing is not None:
            rec = self.owner.ledger.get(self.playing)
            done = self.owner.ledger.elapsed_s(rec) >= rec.duration_s
            if done or self.node.interrupted:
                sid, self.playing, self.node.interrupted = self.playing, None, False
                self.owner.terminal(sid, done and not rec.cancelled, None if done else "cancelled", False)
        while self.playing is None and self.queue:
            sid = self.queue.pop(0)
            rec = self.owner.ledger.get(sid)
            if rec is None or rec.cancelled:
                self.owner.terminal(sid, False, "cancelled", rec is None)
                continue
            duration = len(self.texts[sid]) * PER_CHAR_S
            self.owner.synthesized(sid, duration, {}, None)
            if self.owner.started(sid, np.zeros(160, dtype=np.float32), 16000, duration_s=duration):
                self.playing = sid
        self._events()

    def _events(self) -> None:
        events, self.node.pubs["/voice/speech/event"].sent = self.node.pubs["/voice/speech/event"].sent, []
        for ev in events:
            if ev.kind == M.SpeechEvent.KIND_STARTED:
                self.engine.speech_started(ev.speech_id)
            elif speech_client.is_speech_end(ev):
                self.heard.append(self.texts.get(ev.speech_id, "")[:ev.spoken_chars])
                self.engine.speech_ended(ev.speech_id, int(ev.spoken_chars))


def _numbers(heard: List[str]) -> List[int]:
    return [n for text in heard for n in counting.said_numbers(text)]


def test_edit_mid_count_finishes_the_sounding_number_and_continues_in_the_new_form():
    """«Один … Четы-» звучит → правка → «Четыре.» договаривается, дальше «Пять шагов. … Двадцать шагов.»."""
    stand = _Stand([Command(speech=COUNT), Command(speech=EDIT)])
    stand.turn(N806)
    stand.run(0.2)
    sounding = stand.texts[stand.playing]
    stand.run((sounding.lower().index("четыре") + 2) * PER_CHAR_S)  # звучит «Четыре»
    stand.turn(N807)
    stand.run(80.0)  # правка с единицы (~28 с) успела бы прозвучать целиком
    assert _numbers(stand.heard) == list(range(1, 21))  # каждое число один раз, с единицы заново не считает
    assert sounding == counting.count_text(range(1, 21))  # счёт речью - по предложению на число
    assert stand.heard[0] == counting.count_text(range(1, 5))  # «Четыре» договорено, не обрублено
    assert stand.heard[1] == " ".join(f"{counting.say(n).capitalize()} шагов." for n in range(5, 21))


def test_edit_after_the_count_sounded_does_not_cut_or_repeat_it():
    """Правка пришла, когда счёт уже досказан: звучащего счёта нет - ход идёт как раньше (речь модели)."""
    stand = _Stand([Command(speech=COUNT), Command(speech="Хорошо, в следующий раз с шагом.")])
    stand.turn(N806)
    stand.run(25.0)
    stand.turn(N807)
    stand.run(10.0)
    assert stand.heard == [counting.count_text(range(1, 21)), "Хорошо, в следующий раз с шагом."]
