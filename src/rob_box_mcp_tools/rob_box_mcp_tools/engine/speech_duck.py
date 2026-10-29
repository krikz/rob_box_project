"""Приглушение музыки под речь: робота (ADR-0150 §9.1 О8 (а), В16, PR-7d #3701) и человека (#4155).

Один писатель ручки ``duck`` мастер-шины (``masterfilter.scd``, узел 999) и два источника приглушения; музыка уходит
на ``duckDb`` источника за ``duckAttack``, возвращается за ``duckRelease``. Огибающую ведёт scsynth (``Slew``), отсюда
только переключение 0/1 — без потока ``/n_set``.

**Речь робота.** ``tts_node`` — владелец речи — публикует ``SpeechEvent`` на каждом переходе реплики. Пока звучит хоть
одна реплика, ``duck 1`` с ручками :data:`rob_box_music.knowledge.SPEECH_DUCK` (−9 дБ); последняя закончилась —
``duck 0``. ``started`` у ``SpeechEvent`` бывает только перед локальным воспроизведением в динамики
(``tts_node._sap_play_local``); реплики в шлем оператора (``sink=headset``) идут мимо владельца и событий не дают.
Несколько реплик одновременно — множество ``speech_id``. Финал потерялся (нода упала, сообщение не дошло) — реплика
снимается по сроку ``duration_ms + grace_s`` в :meth:`SpeechDuck.tick`.

**Голос человека (#4155).** ``audio_node`` публикует ``/audio/vad`` на фронтах: подъём — тот самый момент, когда его
сегментатор начинает фразу для STT (``audio_node.check_vad_and_doa``: ``vad_eff`` открывает ``is_speeching``), спад —
когда VAD отпустил. Подъём (музыка играет, это не эхо) — ``duck 1`` с мягкими ручками
:data:`rob_box_music.knowledge.VOICE_DUCK`; спад + удержание :data:`rob_box_music.knowledge.VOICE_DUCK_HOLD_S` —
``duck 0``. Подъём внутри удержания (дребезг VAD между словами) ручку не трогает. Не считается голосом:

* VAD без играющей музыки — приглушать нечего;
* пока звучит речь робота и в эхо-окне после неё (:data:`rob_box_dialog.knowledge.IDENTITY_MIC_GRACE_S` — то же окно,
  по которому ``stt_node`` отличает эхо от фразы): микрофон слышит самого робота, а под его речью музыка и так
  приглушена глубже. Голос, который продолжается после окна, подхватывает :meth:`SpeechDuck.tick`.

Звучат оба источника — действует более глубокий (речь робота); ``duck 0`` — только когда кончились оба. Потерянный
спад VAD снимается по :data:`rob_box_music.knowledge.VOICE_DUCK_MAX_S`. Модуль без ROS: тесты гоняют его на фейковой
отправке и часах.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping, Optional

from rob_box_dialog.knowledge import IDENTITY_MIC_GRACE_S
from rob_box_music import knowledge as kn

from .renardo_adapter import MASTER_NODE

_LOG = logging.getLogger(__name__)

#: Запас к длительности реплики, после которого потерянный финал снимает приглушение, с.
GRACE_S = 3.0
#: Срок реплики без известной длительности (``duration_ms = 0``), с.
UNKNOWN_DURATION_S = 60.0
#: Период :meth:`SpeechDuck.tick` на ноде, с: удержание голоса человека отпускается с точностью до него.
TICK_S = 0.25
#: Финалы реплики (имена ``SpeechEvent.KIND_*`` в нижнем регистре).
_ENDS = ("finished", "cancelled", "rejected")
#: Кто держит приглушение сейчас. Речь робота глубже голоса человека и перекрывает его.
SPEECH, VOICE = "speech", "voice"


def duck_controls(depth_db: float, attack_ms: float, release_ms: float) -> Dict[str, float]:
    """Ручки огибающей для ``/n_set 999``: глубина только вниз, времена не меньше 1 мс."""
    return {"duckDb": min(0.0, float(depth_db)), "duckAttack": max(1.0, float(attack_ms)) / 1000.0,
            "duckRelease": max(1.0, float(release_ms)) / 1000.0}


@dataclass(frozen=True)
class VoiceSource:
    """Голос человека как источник приглушения: чем узнать, что играет музыка, и числа (по умолчанию — знание).

    ``music_playing`` зовётся только пока всплеск VAD ждёт решения; ``echo_grace_s`` — эхо-окно после речи робота.
    """

    music_playing: Callable[[], bool]
    controls: Mapping[str, float] = field(default_factory=lambda: dict(kn.VOICE_DUCK))
    hold_s: float = kn.VOICE_DUCK_HOLD_S
    max_s: float = kn.VOICE_DUCK_MAX_S
    echo_grace_s: float = IDENTITY_MIC_GRACE_S


class VoiceGate:
    """Фронты ``/audio/vad`` → «голос человека держит музыку тише» (с удержанием). Без ROS, OSC и блокировок.

    ``edge`` пишет фронт, ``step`` решает по времени и разрешению ``allowed`` (музыка играет и это не эхо): начать
    приглушение, отпустить его по удержанию или по сроку всплеска. Вызывающий держит свою блокировку.
    """

    def __init__(self, hold_s: float, max_s: float) -> None:
        self.hold_s = float(hold_s)
        self.max_s = float(max_s)
        self.vad_high = False  # последний фронт VAD — подъём
        self.voiced = False  # всплеск засчитан голосом: ручка опущена (в том числе в удержании)
        self.lost_fall = False  # последний step снял всплеск по сроку: спад VAD не дошёл
        self._since = 0.0
        self._hold_until: Optional[float] = None

    @property
    def pending(self) -> bool:
        """Всплеск идёт, а решения «это голос» ещё нет."""
        return self.vad_high and not self.voiced

    def edge(self, high: bool, now: float) -> None:
        """Фронт VAD. Подъём внутри удержания отменяет отпуск: ручка не дёргается."""
        if high == self.vad_high:
            return
        self.vad_high = high
        if high:
            self._since, self._hold_until = now, None
        elif self.voiced:
            self._hold_until = now + self.hold_s

    def step(self, now: float, allowed: bool) -> bool:
        """Продвинуть состояние к ``now``; ``True`` — ``voiced`` изменился."""
        before = self.voiced
        cut = self.vad_high and now - self._since >= self.max_s
        self.lost_fall = cut and self.voiced
        if cut:  # спад потерян: считаем его пришедшим сейчас, а не держим музыку тише вечно
            self.vad_high = False
            self._hold_until = now if self.voiced else None
        if self.voiced and not self.vad_high and self._hold_until is not None and now >= self._hold_until:
            self.voiced, self._hold_until = False, None
        elif self.pending and allowed:
            self.voiced = True
        return self.voiced != before


class SpeechDuck:
    """Звучащие реплики робота и голос человека → ``/n_set 999 duck 0|1`` с ручками огибающей источника.

    Args:
        send_osc: ``RenardoRuntime.send_osc`` или фейк.
        controls: ручки речи робота ``duckDb``/``duckAttack``/``duckRelease`` (:func:`duck_controls`); по умолчанию —
            :data:`rob_box_music.knowledge.SPEECH_DUCK`.
        clock: монотонные часы для срока потерянных финалов и удержания голоса.
        voice: голос человека как источник (:class:`VoiceSource`); ``None`` — источника нет, :meth:`on_vad` молчит.
    """

    def __init__(self, send_osc: Callable[..., None], controls: Mapping[str, float] = kn.SPEECH_DUCK, *,
                 logger: Any = None, clock: Callable[[], float] = time.monotonic, grace_s: float = GRACE_S,
                 voice: Optional[VoiceSource] = None) -> None:
        self._send = send_osc
        self._controls = dict(controls)
        self._log = logger or _LOG
        self._clock = clock
        self._grace_s = grace_s
        self._voice = voice
        self._gate = VoiceGate(voice.hold_s, voice.max_s) if voice is not None else None
        self._lock = threading.Lock()
        self._active: Dict[str, float] = {}  # speech_id → срок, после которого финал считается потерянным
        self._speech_ended_at = float("-inf")  # когда кончилась последняя реплика: от неё считается эхо-окно
        self._level: Optional[str] = None  # кто держит ручку сейчас: SPEECH | VOICE | None (отпущена)

    @property
    def ducked(self) -> bool:
        """Последнее отправленное состояние ручки ``duck``."""
        return self._level is not None

    def on_event(self, kind: str, speech_id: str, duration_ms: int = 0) -> None:
        """``SpeechEvent``: ``started`` — реплика звучит; ``finished``/``cancelled``/``rejected`` — нет."""
        if not speech_id:
            return
        with self._lock:
            if kind == "started":
                span = duration_ms / 1000.0 if duration_ms > 0 else UNKNOWN_DURATION_S
                self._active[speech_id] = self._clock() + span + self._grace_s
            elif kind in _ENDS:
                if self._active.pop(speech_id, None) is not None:  # финал реплики, что не звучала, эха не оставляет
                    self._note_speech_end()
            else:
                return
        self._apply(f"{kind} speech_id={speech_id[:8]}")

    def on_vad(self, high: bool) -> None:
        """``/audio/vad`` (``audio_node``, только фронты): ``True`` — человек заговорил, ``False`` — VAD отпустил."""
        if self._gate is None:
            return
        now = self._clock()
        with self._lock:
            self._gate.edge(bool(high), now)
        self._refresh_voice(now, "vad")

    def tick(self) -> None:
        """Снять реплики, чей финал не пришёл к сроку (громко: это сбой владельца речи), и довести голос человека:
        отпустить по удержанию, подхватить речь, что продолжается после эхо-окна."""
        now = self._clock()
        with self._lock:
            lost = [sid for sid, due in self._active.items() if due <= now]
            for sid in lost:
                del self._active[sid]
            if lost:
                self._note_speech_end()
        if lost:
            self._log.warning(f"⚠️ [duck] финал реплики не пришёл к сроку: {', '.join(s[:8] for s in lost)} — снято")
            self._apply("lost")
        self._refresh_voice(now, "tick")

    def _note_speech_end(self) -> None:
        """Под блокировкой: кончилась последняя реплика — открыть эхо-окно."""
        if not self._active:
            self._speech_ended_at = self._clock()

    def _is_echo(self, now: float) -> bool:
        """Под блокировкой: микрофон сейчас слышит самого робота — речь звучит или эхо-окно после неё не вышло."""
        return bool(self._active) or now - self._speech_ended_at < self._voice.echo_grace_s

    def _refresh_voice(self, now: float, why: str) -> None:
        gate = self._gate
        if gate is None:
            return
        # музыку спрашиваем вне блокировки (у владельца плеера своя) и только пока всплеск ждёт решения
        playing = gate.pending and bool(self._voice.music_playing())
        with self._lock:
            changed = gate.step(now, playing and not self._is_echo(now))
        if gate.lost_fall:
            self._log.warning(f"⚠️ [duck] спад /audio/vad не пришёл за {gate.max_s:g} с — голос человека снят")
        if changed:
            self._apply(why)

    def _want(self) -> Optional[str]:
        """Под блокировкой: кто должен держать ручку — речь робота перекрывает голос человека."""
        if self._active:
            return SPEECH
        return VOICE if self._gate is not None and self._gate.voiced else None

    def _apply(self, why: str) -> None:
        with self._lock:
            want, prev = self._want(), self._level
            if want == prev:
                return
            self._level = want
            count = len(self._active)
        controls = self._voice.controls if (want or prev) == VOICE else self._controls
        args = ["duck", 1.0 if want else 0.0]
        for name, value in controls.items():  # ручки — с каждым переключением: узел мог пересоздаться
            args += [name, float(value)]
        try:
            self._send("/n_set", MASTER_NODE, *args)
        except Exception as exc:  # noqa: BLE001 — SC недоступен: речь важнее, музыку не роняем
            self._log.warning(f"⚠️ [duck] /n_set не отправлен: {exc}")
            with self._lock:
                self._level = prev  # следующее событие попробует снова
            return
        self._log_change(prev, want, why, count, controls)

    def _log_change(self, prev: Optional[str], want: Optional[str], why: str, count: int,
                    controls: Mapping[str, float]) -> None:
        """Строка лога на каждом переходе ручки (фронты, не поток): голос человека — своей фразой."""
        knobs = " ".join(f"{k}={v:g}" for k, v in controls.items())
        if want == VOICE and prev is None:
            self._log.info(f"🔉 [duck] голос человека — музыка тише ({knobs})")
        elif want is None and prev == VOICE:
            self._log.info("🔉 [duck] голос человека — возврат")
        else:
            self._log.info(f"🔉 [duck] {'on' if want else 'off'} ({why}, реплик={count}) {knobs}")


__all__ = ["SPEECH", "SpeechDuck", "TICK_S", "VOICE", "VoiceGate", "VoiceSource", "duck_controls"]
