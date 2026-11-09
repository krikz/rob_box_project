"""Перебивание речи по вейку (ADR-0150 §6.2, PR-5 #3694): stt_node спрашивает, диалог решает.

Раньше stt_node сам глушил речь (``SpeechCancel``) на любую фразу, начинавшуюся с вейка, — по префиксу
(«работает» начинается с «роб»). Теперь stt_node публикует ``/dialog/interrupt_request`` на каждый новый сегмент
фразы с вейком (:class:`InterruptOffer`), а решает ``dialog_node`` (``_on_interrupt`` →
:func:`rob_box_dialog.interrupt.on_interrupt`): ясная стоп-команда (:func:`rob_box_dialog.words.stop_verdict`) —
``SpeechCancel`` по эпохе; иначе речь продолжается, новая реплика встаёт после текущей (APPEND у tts_node).
Спорный случай («спой песню про стоп») речь не рвёт (``disputed`` — не стоп).

Без ROS: сообщение, публикация, отмена и часы приходят снаружи.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional, Sequence

from rob_box_dialog.words import has_wake_word


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

    def offer(self, text: str, utterance_id: str, admission: Any = None) -> bool:
        """Опубликовать запрос, если в сегменте есть вейк и такой текст этой фразы ещё не отправлялся.

        ``admission`` - решение эхо-окна по фразе (``echo_window.Admission``): фраза из окна после речи робота или
        короткий обрывок при речи доходит до диалога, только когда её текст стал ясным стопом с вейком
        (``Admission.keeps``), - partial с вейком из хвоста собственной речи речь не рвёт (ревью #4122). ``None`` - окно
        фразу пускает как есть (шлем, окна нет)."""
        text = (text or "").strip()
        words = self._wake_words()
        if not text or (utterance_id, text) == self._last or not has_wake_word(text, words):
            return False
        if admission is not None and not admission.keeps(text, words):
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
