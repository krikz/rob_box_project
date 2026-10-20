"""Заполнение ``Utterance`` для фразы ReSpeaker (ADR-0150 §10.1, PR-2).

Чистая функция без ROS: сообщение приходит снаружи (настоящий тип или
заглушка из ``rob_box_core.msg_stub``), поэтому соответствие
«фраза STT -> поля Utterance» проверяется юнит-тестом без сборки rosidl.
"""

from __future__ import annotations

from typing import Any

SOURCE_RESPEAKER = "respeaker"


def final_provider(attempts) -> str:
    """Провайдер, чей ответ принят: последняя попытка (``""`` - попыток не было)."""
    return attempts[-1].provider if attempts else ""


def _set_time(field: Any, moment: float) -> None:
    """Записать секунды эпохи в ``builtin_interfaces/Time``."""
    field.sec = int(moment)
    field.nanosec = int((moment - int(moment)) * 1e9)


def fill_respeaker_utterance(
    msg: Any,
    *,
    text: str,
    utterance_id: str,
    duration_s: float,
    now_s: float,
    speaker_tag: str = "",
    provider: str = "",
) -> Any:
    """Записать фразу ReSpeaker в ``msg`` и вернуть его.

    ``utterance_id`` - детерминированный хеш PCM (``core/utterance_id.py``),
    одинаковый у stt_node и speaker_id_node без координации. Конец фразы -
    ``now_s`` (момент публикации), начало - конец минус длительность аудио.
    """
    msg.utterance_id = utterance_id
    msg.text = text
    msg.source = SOURCE_RESPEAKER
    msg.speaker_tag = speaker_tag or ""
    msg.provider = provider or ""
    _set_time(msg.started_at, now_s - max(float(duration_s), 0.0))
    _set_time(msg.ended_at, now_s)
    return msg
