"""Незнакомому голосу — профиль без имени, с кличкой (карточка #3858, решение Шифу 22.10, «режим мастерской»).

Раньше профиль появлялся только по ``register_speaker`` с именем: голос, который не представился, не оставлял следа,
и в следующий раз робот знакомого не узнавал. Теперь решение «завести профиль» принимает владелец биометрии
(``speaker_id_node``) кодом, без LLM (ADR-0148): здесь — чистое правило «можно ли», нода — исполнение.

Пороги опознания (0.72) и слияния (0.75) не трогаются: профиль заводится только тогда, когда голос НИКОГО не опознал
(``identify()`` не прошёл порог). Защита от мусора (#2779) — два рубежа: гейт длительности речи (тот же, что у
регистрации по имени: ``MIN_REGISTER_AUDIO_DURATION_SEC``) и лимит числа безымянных профилей с вытеснением старейших
(``SpeakerDatabase.unnamed.register``).

«Спорный» голос (``contested``, #2809: порог пройден, но в полосе сомнения есть кандидат с другим именем) двойника НЕ
получает (решение Шифу 26.10, #3858): он уже похож на знакомого, и каждая спорная фраза длиннее гейта речи заводила
бы безымянную копию своего человека (акт 2b: «Саша» 0.944 / «Борис» 0.661 при band_high 0.99). Диалог видит такой
голос как незнакомца (имени и чужих фактов нет, #4023), но снимок честно говорит «голос похож на знакомый» (#3993).
"""

from __future__ import annotations

from typing import Any, Optional

from rob_box_voice.utils import speaker_embeddings as _se

SKIP_DISABLED = "disabled"
SKIP_TOO_SHORT = "too_short"
SKIP_RECOGNISED = "recognised"
SKIP_GROWTH_SESSION = "growth_session"


def enroll_skip_reason(
    *,
    enabled: bool,
    voiced_sec: Optional[float],
    min_voiced_sec: float,
    recognised: bool,
    growth_session_open: bool,
) -> Optional[str]:
    """Почему профиль без имени заводить НЕ надо; ``None`` — можно.

    * ``recognised`` — ``identify()`` вернул профиль (порог 0.72 пройден) — при ЛЮБОМ решении об имени: уверенное,
      одна гипотеза («single», свой на плохом канале) или «contested» (конкурент с другим именем в полосе сомнения).
      Узнанный голос на безымянные клоны не дробим (#3858, решение Шифу 26.10: спорному голосу двойника не заводить).
    * ``voiced_sec`` — речь после VAD; неизвестна/короче ``min_voiced_sec`` — профиля нет (шум, обрывок фразы).
    * ``growth_session_open`` — идёт growth-сессия (человек только что представился, #2747): реплика, не прошедшая
      вето, сессию закрыла бы; пока она открыта, голос считается голосом её владельца, а не новым человеком.
    """
    if not enabled:
        return SKIP_DISABLED
    if voiced_sec is None or voiced_sec < min_voiced_sec:
        return SKIP_TOO_SHORT
    if recognised:
        return SKIP_RECOGNISED
    if growth_session_open:
        return SKIP_GROWTH_SESSION
    return None


def enroll(
    node: Any,
    embedding: Any,
    match: Optional[_se.SpeakerMatch],
    duration_sec: float,
) -> Optional[_se.SpeakerMatch]:
    """#3858: незнакомый голос -> профиль без имени + кличка; результат -- «узнан, имени нет» (или ``None``).

    Решает код (:func:`enroll_skip_reason`), LLM не участвует (ADR-0148). Отказ -- штатный путь (короткая фраза,
    голос уже опознан, идёт growth-сессия): профиль не заводится, результат прежний. ``node`` -- ``SpeakerIdNode``
    (БД, параметры, кличка); вынесено из класса ради бюджета (ADR-0145).
    """
    reason = enroll_skip_reason(
        enabled=node._unnamed_profiles_enabled,
        voiced_sec=duration_sec,
        min_voiced_sec=_se.MIN_REGISTER_AUDIO_DURATION_SEC,
        recognised=match is not None,
        growth_session_open=getattr(node, "_growth_session", None) is not None,
    )
    if reason is not None:
        node.get_logger().debug(f"[issue #3858] профиль без имени не заводим: {reason}")
        return None
    try:
        outcome = node._db.unnamed.register(embedding, duration_sec, max_unnamed=node._unnamed_profiles_max)
    except _se.AudioTooShortError:  # пол длительности в БД строже/свежее параметров ноды
        return None
    for victim in outcome.evicted:
        forget(node, victim)
    node._ensure_epithet(outcome.speaker_id)
    epithet = node._db.get_epithet(outcome.speaker_id)
    node.get_logger().info(
        f"🆕 [issue #3858] незнакомый голос -> профиль без имени id={outcome.speaker_id[:8]} "
        f"кличка={epithet!r} (речь {duration_sec:.2f}с, безымянных {node._db.unnamed.count()}/"
        f"{node._unnamed_profiles_max}, вытеснено {len(outcome.evicted)})"
    )
    return _se.SpeakerMatch(
        speaker_id=outcome.speaker_id, name=_se.UNNAMED, confidence=0.0, epithet=epithet, gallery_size=1
    )


def forget(node: Any, speaker_id: str) -> None:
    """Забыть служебное состояние удалённого (вытесненного) профиля."""
    with node._speech_log_lock:
        node._speech_log.pop(speaker_id, None)
    node._epithet_llm_reasked.discard(speaker_id)
    session = node._growth_session
    if session is not None and session["speaker_id"] == speaker_id:
        node._growth_session = None
