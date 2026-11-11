"""Гипотезы распознавания фразы: уверенность STT и другие тексты (#4017 шаг C1, ADR-0148).

Поля ``Utterance.confidence`` и ``Utterance.alternatives`` (``rob_box_dialog_msgs``) <-> значение
:class:`SttHypotheses` для потребителей. ЕДИНСТВЕННОЕ место, где живут правила «что значит −1»,
«сколько альтернатив везём» и «какая уверенность провайдера осмысленна»: писатель (``stt_node``, адаптер Yandex)
кладёт значение в сообщение через :func:`fill_hypotheses`, читатель (диалог, ``address.heard_from``) достаёт
его через :func:`hypotheses_from_msg`.

Шаг C1 — только данные: поведение по уверенности (переспрос «я услышал X — ты про Y?» при действии, от которого
зависит стоп, деньги или необратимое) решит код на шаге C2.

Yandex SpeechKit v3 (``yandex/cloud/ai/stt/v3/stt.proto``): ``Alternative.confidence`` описан как
«The hypothesis confidence. Currently is not used», опции «дай N гипотез» в ``RecognitionModelOptions`` нет.
Нуль proto3 — это «поле не заполнено», а не «уверенность ноль»: :func:`clean_confidence` даёт для него
:data:`CONFIDENCE_UNKNOWN`.

Модуль не импортирует ROS: сообщение — любой объект с полями ``Utterance`` (настоящий тип или
``rob_box_core.msg_stub``). У сообщения без новых полей (старая заглушка, чужой издатель) — :data:`NO_HYPOTHESES`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Tuple

#: Уверенность неизвестна: значение по умолчанию поля ``Utterance.confidence``.
CONFIDENCE_UNKNOWN = -1.0
#: Сколько других гипотез текста фразы везём в ``Utterance.alternatives``.
MAX_ALTERNATIVES = 3
_DIGITS = 4


@dataclass(frozen=True)
class SttHypotheses:
    """Что распознавание знает о фразе сверх лучшего текста.

    Attributes:
        confidence: Уверенность в лучшей гипотезе, ``(0, 1]``; :data:`CONFIDENCE_UNKNOWN` — неизвестно.
        alternatives: Другие тексты фразы, лучшие первыми, без лучшего текста, не больше :data:`MAX_ALTERNATIVES`.
    """

    confidence: float = CONFIDENCE_UNKNOWN
    alternatives: Tuple[str, ...] = ()


#: Распознавание ничего сверх текста не сказало (Telegram, оператор, Vosk, MiniMax, Yandex без уверенности).
NO_HYPOTHESES = SttHypotheses()


def clean_confidence(value: Any) -> float:
    """Уверенность провайдера в ``(0, 1]`` (округлённая) или :data:`CONFIDENCE_UNKNOWN`.

    Ноль, отрицательное, больше единицы, NaN, не число — неизвестно: proto3 не отличает «не заполнено» от нуля.
    """
    try:
        number = float(value)
    except (TypeError, ValueError):
        return CONFIDENCE_UNKNOWN
    return round(number, _DIGITS) if 0.0 < number <= 1.0 else CONFIDENCE_UNKNOWN


def hypotheses(confidence: Any, alternatives: Iterable[Any], *, text: str = "") -> SttHypotheses:
    """Собрать :class:`SttHypotheses`: уверенность через :func:`clean_confidence`; альтернативы — непустые,
    без повторов и без самого ``text`` (сравнение без регистра и крайних пробелов), по порядку, не больше
    :data:`MAX_ALTERNATIVES`."""
    seen = {_key(text)} if text else set()
    kept = []
    for raw in alternatives or ():
        alt = str(raw or "").strip()
        if alt and _key(alt) not in seen and len(kept) < MAX_ALTERNATIVES:
            seen.add(_key(alt))
            kept.append(alt)
    return SttHypotheses(clean_confidence(confidence), tuple(kept))


def _key(text: str) -> str:
    return " ".join(text.lower().split())


def fill_hypotheses(msg: Any, value: SttHypotheses) -> Any:
    """Записать гипотезы в ``Utterance`` и вернуть сообщение."""
    msg.confidence = float(value.confidence)
    msg.alternatives = list(value.alternatives)
    return msg


def hypotheses_from_msg(msg: Any) -> SttHypotheses:
    """Гипотезы из ``Utterance``; у сообщения без полей — :data:`NO_HYPOTHESES` (разбор как до шага C1)."""
    return hypotheses(
        getattr(msg, "confidence", CONFIDENCE_UNKNOWN),
        getattr(msg, "alternatives", ()) or (),
        text=str(getattr(msg, "text", "") or ""),
    )


__all__ = [
    "CONFIDENCE_UNKNOWN",
    "MAX_ALTERNATIVES",
    "NO_HYPOTHESES",
    "SttHypotheses",
    "clean_confidence",
    "fill_hypotheses",
    "hypotheses",
    "hypotheses_from_msg",
]
