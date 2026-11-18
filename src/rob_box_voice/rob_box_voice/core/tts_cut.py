"""Честная фраза обрыва TTS-потока (ADR-0148, #4694).

Поток уже начал звучать и оборвался (``tts_stream.BROKEN``): хвост реплики не прозвучал.
Фразу говорит код из этого факта, не из текста LLM. Отмена человеком и отказ до первого
звука сюда не входят: там реплику не «оборвало на полуслове».
"""

from __future__ import annotations

from typing import Any, Optional

from rob_box_core.speech_client import SYSTEM_AGENT
from rob_box_core.utterance import Utterance

from .tts_stream import BROKEN

#: Свой ключ: пин ядра v0.7.8 не отдаёт ``knowledge.TTS_CUT_PHRASE`` (#4819).
TTS_CUT_PHRASE = "tts.cut"
#: Ровно ``phrases/ru.yaml``. Не ``SPEECH_CUT_PHRASE`` («и так далее.»).
TTS_CUT_TEXT = "Речь оборвалась, хвост не прозвучал."

#: Одна фраза на оборванную реплику: повтор того же ``speech_id`` молчит.
_said: set = set()


def phrase() -> str:
    """Текст каталога. Ключ один, вариант не выбирает LLM."""
    try:
        from rob_box_dialog import knowledge as kn  # noqa: PLC0415
        from rob_box_dialog import phrases  # noqa: PLC0415

        key = getattr(kn, "TTS_CUT_PHRASE", TTS_CUT_PHRASE)
        return phrases.text(key)
    except (ImportError, KeyError):
        return TTS_CUT_TEXT


def notice(error: Optional[str], speech_id: Optional[str]) -> Optional[str]:
    """Фраза, если ``error`` — обрыв потока после звука, и эту реплику ещё не называли.

    Другая ошибка, пустой ``speech_id`` и повтор — ``None`` (не выдумываем обрыв).
    """
    if error != BROKEN or not speech_id or speech_id in _said:
        return None
    _said.add(speech_id)
    return phrase()


def say(node: Any, error: Optional[str], speech_id: Optional[str]) -> bool:
    """Сказать фразу в динамики, если :func:`notice` её дал. Сбой отправки — в лог, не молча."""
    text = notice(error, speech_id)
    if text is None:
        return False
    try:
        from rob_box_core import speech_client  # noqa: PLC0415 — клиент тянет сообщения диалога

        speech_client.prepare(node, "request")  # первый publish после старта не должен потеряться
        speech_client.say(node, Utterance(text=text), agent=SYSTEM_AGENT)
    except Exception as exc:  # noqa: BLE001 — обрыв уже факт; фраза не должна ронять воспроизведение
        node.get_logger().error(f"🗣️ обрыв речи не назван ({text!r}): {exc!r}")
        return False
    node.get_logger().info(f"🗣️ речь оборвалась — говорю: {text!r} speech_id={(speech_id or '')[:8]}")
    return True
