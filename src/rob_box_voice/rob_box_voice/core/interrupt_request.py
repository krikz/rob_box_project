"""Перебивание речи по вейку (ADR-0150 §6.2, PR-5 #3694): stt_node спрашивает, диалог решает.

Раньше stt_node сам глушил речь (``SpeechCancel``) на любую фразу, начинавшуюся с вейка, — по префиксу
(«работает» начинается с «роб»), и диалог после этого глушил её второй раз. Теперь stt_node публикует
``/dialog/interrupt_request`` на каждый новый сегмент фразы с вейком (:class:`InterruptOffer`), а решает
``dialogue_node`` (:class:`InterruptGate`): ясная стоп-команда (:func:`rob_box_dialog.words.stop_verdict`) —
``SpeechCancel`` по эпохе за доли миллисекунды кода; иначе речь продолжается, новая реплика встаёт после
текущей (APPEND у tts_node; ``compose()`` — PR-7b). Спорный случай («спой песню про стоп») речь не рвёт —
его решает Chooser по итоговой фразе в приёме (``Tier1Step`` → ``core.stop_chooser``, PR-6 #3696).

Без ROS: сообщение, публикация, отмена и часы приходят снаружи.
"""

from __future__ import annotations

import time
from collections import OrderedDict
from typing import Any, Callable, Optional, Sequence

from rob_box_dialog.words import STOP_CLEAR, STOP_DISPUTED, has_wake_word, stop_verdict

#: Сколько последних фраз помнить, чтобы не рвать речь дважды на partial и final одной фразы.
SEEN_UTTERANCES = 32


def _stamp(field: Any, moment: float) -> None:
    field.sec = int(moment)
    field.nanosec = int((moment - int(moment)) * 1e9)


class InterruptOffer:
    """Сторона stt_node: сегмент с вейком → ``InterruptRequest`` (один раз на одинаковый текст фразы)."""

    def __init__(
        self,
        publish: Callable[[Any], None],
        make_msg: Callable[[], Any],
        wake_words: Callable[[], Sequence[str]],
        doa_deg: Callable[[], Optional[float]],
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._publish, self._make_msg = publish, make_msg
        self._wake_words, self._doa_deg, self._clock = wake_words, doa_deg, clock
        self._last = ("", "")

    def offer(self, text: str, utterance_id: str) -> bool:
        """Опубликовать запрос, если в сегменте есть вейк и такой текст этой фразы ещё не отправлялся."""
        text = (text or "").strip()
        if not text or (utterance_id, text) == self._last or not has_wake_word(text, self._wake_words()):
            return False
        self._last = (utterance_id, text)
        msg = self._make_msg()
        msg.utterance_id = utterance_id or ""
        msg.text_partial = text
        doa = self._doa_deg()
        msg.doa_deg = float("nan") if doa is None else float(doa)
        _stamp(msg.stamp, self._clock())
        self._publish(msg)
        return True


class InterruptGate:
    """Сторона диалога v1: ``InterruptRequest`` → ясный стоп рвёт речь личности, остальное — нет.

    ``cancel(reason)`` — отмена речи личности текущей эпохи (``speech_client.cancel``); одна фраза рвёт речь
    не больше одного раза, сколько бы её сегментов ни пришло.
    """

    def __init__(
        self,
        cancel: Callable[[str], None],
        wake_words: Callable[[], Sequence[str]],
        log: Callable[[str], None],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._cancel, self._wake_words, self._log, self._clock = cancel, wake_words, log, clock
        self._done: "OrderedDict[str, None]" = OrderedDict()

    def on_request(self, msg: Any) -> str:
        """Решить по сегменту; вернуть вердикт (``clear``/``disputed``/``none``/``duplicate``)."""
        uid = getattr(msg, "utterance_id", "") or ""
        text = getattr(msg, "text_partial", "") or ""
        if uid and uid in self._done:
            return "duplicate"
        started = self._clock()
        verdict = stop_verdict(text, self._wake_words())
        if verdict == STOP_CLEAR:
            self._cancel(f"interrupt:{uid or '-'}")
            if uid:
                self._done[uid] = None
                while len(self._done) > SEEN_UTTERANCES:
                    self._done.popitem(last=False)
            ms = (self._clock() - started) * 1000.0
            self._log(f"🛑 [interrupt] ясный стоп {text[:60]!r} → SpeechCancel за {ms:.1f} мс utterance={uid or '-'}")
        elif verdict == STOP_DISPUTED:
            self._log(f"🤔 [interrupt] спорный стоп {text[:60]!r} — речь не рвём (решит Chooser по итоговой фразе)")
        return verdict
