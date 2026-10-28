"""Эхо-окно микрофона после речи робота (#989 Fix B, #4072): решает, КОГДА звучал голос фразы, а не когда она доехала.

Было два окна по 2,5 с после КАЖДОЙ реплики робота: ``audio_node`` глушил VAD (микрофон глух), ``stt_node``
выбрасывал фразу, доставленную в окне. Выступление звучит сегментами с паузой 1–2 с, поэтому:

* «Робот, хватит» в паузе между сегментами микрофон не слышал, а старт следующего сегмента сбрасывал начатую
  фразу — до STT доезжал хвост без вейка («замолчи», марафон 27.10, n805_hard_stop);
* фраза, сказанная во время реплики, доезжает до STT через ``speech_continuation`` (3 с) после последнего VAD —
  уже в окне после конца реплики — и выбрасывалась как эхо (va_marathon.log 23.10, 22:48:33 и 22:53:21: голос
  человека, speaker_id 0.89 и 0.888).

Стало: окно одно — здесь, у ``stt_node``. ``audio_node`` слышит всегда, как во время речи (barge-in #993), а
:class:`EchoWindow` по фронтам ``/audio/vad`` помнит, где звучал голос ещё не доставленной фразы: во время речи
робота (:data:`SPEECH`), в эхо-окне после её конца, пока робот молчит (:data:`GRACE`), или в тишине (:data:`FREE`).
Решение по фразе (:meth:`EchoWindow.admit`):

* голос звучал только в эхо-окне — это эхо собственного голоса или хвост реплики: фраза распознаётся, но проходит
  только ясная стоп-команда с вейком (:func:`stop_passes`, «стоп важнее»); остальное — молча, без «не расслышал»
  (петля #989);
* иначе — как раньше: software-AEC глушит фразы, пока робот говорит; hardware-AEC короткий обрывок при звучащей речи
  (< :data:`SHORT_DURING_SPEECH_S`) распознаёт, но пропускает только ясный стоп с вейком: «Робот, стоп» бывает короче
  порога, и режь мы его до STT — стоп в паузе между сегментами пропал бы (ревью #4122); щелчок и эхо до диалога не
  доходят, как и раньше. Остальное идёт дальше — перебивание через вейк решает диалог.

Длина окна — одна константа знания ``knowledge.IDENTITY_MIC_GRACE_S`` (дефолт параметра ``stt_node.tts_grace_s``).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import AbstractSet, Any, Callable, FrozenSet, Optional, Sequence, Set, Tuple

from rob_box_core import speech_client
from rob_box_dialog.words import STOP_CLEAR, has_wake_word, stop_verdict

#: Где звучал голос: во время речи робота, в эхо-окне после её конца (робот молчит), в тишине.
SPEECH, GRACE, FREE = "speech", "grace", "free"

#: Вердикты :meth:`EchoWindow.admit`.
PASS = "pass"
STOP_ONLY = "stop_only"  # голос звучал только в эхо-окне после речи робота
SHORT_STOP_ONLY = "short_stop_only"  # короткая фраза при звучащей речи (hardware-AEC)
DROP_SPEAKING = "drop_speaking"
#: Вердикты, после которых фраза идёт дальше только как ясный стоп с вейком (``Admission.keeps``).
STOP_ONLY_VERDICTS = (STOP_ONLY, SHORT_STOP_ONLY)

#: Короче этого фраза при звучащей речи робота — эхо/щелчок (hardware-AEC, #989).
SHORT_DURING_SPEECH_S = 0.8
#: Длинную речь audio_node режет на куски и шлёт их подряд (#3762): кусок без своих фронтов VAD берёт места
#: предыдущего куска той же фразы, если доехал не позже этого.
CHUNK_GAP_S = 1.0

VAD_TOPIC = "/audio/vad"
#: Строки лога фронта речи робота: software-AEC глушит распознавание, hardware — XVF-3000 фильтрует эхо.
_SOFTWARE_EDGE_LOG = {
    True: "🔇 [software AEC] Робот говорит - распознавание отключено",
    False: "🎙️ [software AEC] Робот замолчал - распознавание включено",
}
_HARDWARE_EDGE_LOG = {
    True: "🎤 [hardware AEC] Робот говорит - XVF-3000 фильтрует эхо",
    False: "🎙️ [hardware AEC] Робот замолчал — эхо-окно {grace_s}с",
}


def stop_passes(text: str, wake_words: Sequence[str]) -> bool:
    """Фраза из эхо-окна проходит, только если это ясная стоп-команда с вейком.

    Правило то же, по которому диалог рвёт речь по ``/dialog/interrupt_request``: вейк (``has_wake_word``) и
    ``stop_verdict == clear`` — «Робот, хватит», «Робот, всё, хватит, замолчи». Эхо собственной речи без вейка
    и стоп-слова первым словом команды не проходит."""
    return (
        has_wake_word(text, wake_words) and stop_verdict(text, wake_words) == STOP_CLEAR
    )


@dataclass(frozen=True)
class Admission:
    """Решение по доставленной фразе ReSpeaker: ``verdict`` и строка лога ``note`` (``""`` — молча)."""

    verdict: str
    note: str = ""

    @property
    def dropped(self) -> bool:
        """Фраза отброшена до распознавания."""
        return self.verdict == DROP_SPEAKING

    def keeps(self, text: Optional[str], wake_words: Sequence[str]) -> bool:
        """После распознавания (и на каждом промежуточном результате STT): фраза идёт дальше? Из эхо-окна и короткая при
        звучащей речи — только ясный стоп с вейком. Пока фраза не пропущена, диалог о ней не знает: ни результата,
        ни ``/dialog/interrupt_request`` (``stt_node._offer_interrupt``)."""
        return self.verdict not in STOP_ONLY_VERDICTS or stop_passes(
            text or "", wake_words
        )

    def echo_note(self, text: Optional[str], duration_s: float) -> str:
        """Строка лога отброшенной после распознавания фразы: эхо-окно или короткий обрывок при речи робота."""
        if self.verdict == SHORT_STOP_ONLY:
            return (
                f"🔇 [hardware AEC] Короткая фраза {duration_s:.2f}с {text or ''!r} во время TTS и не стоп с вейком — "
                "молчу (эхо или щелчок)"
            )
        return (
            f"🔇 [issue 989/#4072] Эхо-окно: фраза {duration_s:.2f}с {text or ''!r} звучала только после речи "
            "робота и не стоп с вейком — молчу (эхо собственного голоса)"
        )


class EchoWindow:
    """Речь робота и эхо-окно после неё для одного микрофона; что звучало в окне, решает :meth:`admit`.

    Фронты речи — ``SpeechEvent`` (``speech_client.watch_speaking``: реплика любого агента), фронты голоса —
    ``/audio/vad``. Всплеск VAD отмечает место на подъёме и на спаде, фронт речи робота посреди всплеска — тоже.
    Фраза доезжает после ``speech_continuation`` тишины, поэтому её всплески к доставке закрыты; открытый
    всплеск — уже следующая фраза, он остаётся ей.
    """

    def __init__(
        self,
        grace_s: float,
        aec_mode: str = "hardware",
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.grace_s = float(grace_s)
        self.aec_mode = aec_mode
        self.speaking = False
        self.ended_at = float("-inf")
        self._clock = clock
        self._open: Optional[Set[str]] = None
        self._closed: Set[str] = set()
        self._last: Tuple[float, FrozenSet[str]] = (float("-inf"), frozenset())

    def where(self) -> str:
        """Где мы сейчас: говорит робот, эхо-окно после его речи или тишина."""
        if self.speaking:
            return SPEECH
        return GRACE if self._clock() - self.ended_at < self.grace_s else FREE

    def on_speech(self, speaking: bool) -> None:
        """Фронт речи робота: конец открывает эхо-окно; всплеск голоса, идущий через фронт, отмечает новое место."""
        if not speaking:
            self.ended_at = self._clock()
        self.speaking = bool(speaking)
        if self._open is not None:
            self._open.add(self.where())

    def on_vad(self, active: bool) -> None:
        """Фронт ``/audio/vad``: подъём открывает всплеск, спад закрывает его (с местом конца)."""
        if active and self._open is None:
            self._open = {self.where()}
        elif not active and self._open is not None:
            self._closed |= self._open | {self.where()}
            self._open = None

    def heard(self) -> FrozenSet[str]:
        """Где звучал голос доставленной фразы (и забыть его). Фронтов VAD не было — кусок той же фразы
        (:data:`CHUNK_GAP_S`) или место в момент доставки (поведение до #4072)."""
        now = self._clock()
        if self._closed:
            places = frozenset(self._closed)
        elif now - self._last[0] < CHUNK_GAP_S:
            places = self._last[1]
        else:
            places = frozenset({self.where()})
        self._closed = set()
        self._last = (now, places)
        return places

    def admit(self, duration_s: float) -> Admission:
        """Решение по доставленной фразе ReSpeaker (длина ``duration_s``) — до распознавания."""
        return decide(
            self.heard(),
            speaking=self.speaking,
            duration_s=duration_s,
            aec_mode=self.aec_mode,
        )


#: Фраза шлема (ptt/wake): эхо-окно ReSpeaker не про неё.
OPEN = Admission(PASS)


def decide(
    heard: AbstractSet[str], *, speaking: bool, duration_s: float, aec_mode: str
) -> Admission:
    """Вердикт по местам голоса фразы ``heard`` и речи робота сейчас (``speaking``)."""
    software = aec_mode == "software"
    if software and speaking:
        return Admission(
            DROP_SPEAKING,
            f"🔇 [software AEC] Игнор фразы {duration_s:.2f}с: робот говорит",
        )
    if heard and set(heard) <= {GRACE}:
        return Admission(
            STOP_ONLY,
            f"🔇 [issue 989/#4072] Фраза {duration_s:.2f}с звучала только в эхо-окне после речи робота — "
            "распознаю, пропущу только стоп с вейком",
        )
    if speaking and duration_s < SHORT_DURING_SPEECH_S:
        return Admission(
            SHORT_STOP_ONLY,
            f"🔇 [hardware AEC] Короткая фраза {duration_s:.2f}с при звучащем TTS — распознаю, пропущу только стоп "
            "с вейком",
        )
    if speaking:
        return Admission(
            PASS,
            f"🎤 [hardware AEC] Фраза {duration_s:.2f}с во время TTS — обрабатываем (возможно прерывание)",
        )
    return Admission(PASS)


def watch(
    node: Any,
    *,
    grace_s: float,
    aec_mode: str,
    vad_type: Any,
    clock: Callable[[], float] = time.monotonic,
) -> EchoWindow:
    """Подписать узел на речь робота (``SpeechEvent``) и ``/audio/vad``; вернуть его эхо-окно.

    ``vad_type`` — ``std_msgs.msg.Bool`` (модуль ROS-сообщений не тянет)."""
    window = EchoWindow(grace_s, aec_mode, clock)

    def on_speech(speaking: bool) -> None:
        window.on_speech(speaking)
        if aec_mode == "software":
            node.get_logger().info(_SOFTWARE_EDGE_LOG[speaking])
        else:
            node.get_logger().debug(
                _HARDWARE_EDGE_LOG[speaking].format(grace_s=window.grace_s)
            )

    speech_client.watch_speaking(node, on_speech)
    node.create_subscription(
        vad_type, VAD_TOPIC, lambda msg: window.on_vad(bool(msg.data)), 10
    )
    return window
