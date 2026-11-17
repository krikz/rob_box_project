"""Заполнение ``Utterance`` для фразы ReSpeaker (ADR-0150 §10.1, PR-2).

Чистая функция без ROS: сообщение приходит снаружи (настоящий тип или
заглушка из ``rob_box_core.msg_stub``), поэтому соответствие
«фраза STT -> поля Utterance» проверяется юнит-тестом без сборки rosidl.

Гипотезы распознавания (уверенность, другие тексты; #4017 шаг C1) знает только адаптер Yandex: у принятой фразы
другого провайдера они неизвестны (:func:`phrase_hypotheses`). Правила значений — ``rob_box_core.stt_hypotheses``.
"""

from __future__ import annotations

from typing import Any

from rob_box_core.stt_hypotheses import NO_HYPOTHESES, SttHypotheses, fill_hypotheses

SOURCE_RESPEAKER = "respeaker"
#: Провайдер цепочки STT, чей адаптер собирает гипотезы фразы (``YandexSegmentCollector.hypotheses``).
HYPOTHESES_PROVIDER = "yandex"


def final_provider(attempts) -> str:
    """Провайдер, чей ответ принят: последняя попытка (``""`` - попыток не было)."""
    return attempts[-1].provider if attempts else ""


def phrase_hypotheses(provider: str, yandex: SttHypotheses) -> SttHypotheses:
    """Гипотезы принятой фразы: последние от адаптера Yandex, если фразу принял он; иначе неизвестны
    (Yandex мог ответить, но фразу отдал фолбек — его гипотезы не про этот текст)."""
    return yandex if provider == HYPOTHESES_PROVIDER else NO_HYPOTHESES


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
    hypotheses: SttHypotheses = NO_HYPOTHESES,
) -> Any:
    """Записать фразу ReSpeaker в ``msg`` и вернуть его.

    ``utterance_id`` - детерминированный хеш PCM (``core/utterance_id.py``),
    одинаковый у stt_node и speaker_id_node без координации. Конец фразы -
    ``now_s`` (момент публикации), начало - конец минус длительность аудио. ``hypotheses`` - уверенность и
    другие тексты фразы (по умолчанию неизвестны: ``confidence=-1``, альтернатив нет).
    """
    msg.utterance_id = utterance_id
    msg.text = text
    msg.source = SOURCE_RESPEAKER
    msg.speaker_tag = speaker_tag or ""
    msg.provider = provider or ""
    _set_time(msg.started_at, now_s - max(float(duration_s), 0.0))
    _set_time(msg.ended_at, now_s)
    fill_hypotheses(msg, hypotheses)
    return msg
