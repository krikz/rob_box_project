"""
ROS-parameter / request-payload parsing helpers for ``TTSNode``.

ADR-0145 §4 TTSNode step 1 — pure-Python module-level functions extracted
verbatim (no behaviour change) from ``tts_node.py``. Dependency-free
(only ``json`` and stdlib typing) so it can be unit-tested standalone,
same rationale as the sibling ``tts_chunking.py`` module.

``tts_node.py`` re-exports every name here for backward compatibility with
existing imports/tests (``from rob_box_voice.tts_node import
_parse_optional_int`` etc.).
"""

import json
from collections.abc import Mapping
from typing import Any, Callable


def _parse_optional_number(value: object, cast: Callable[[Any], Any]) -> Any:
    """Parse a ROS-stringy value into ``cast(value)`` (``int``/``float``) or ``None``.

    architecture audit 2026-09-29, ADR-0145: общая реализация для
    :func:`_parse_optional_int` / :func:`_parse_optional_float`.

    Empty string / ``None`` → ``None`` (field omitted from payload).
    Coercion failures are treated as "unset" so a typo in YAML doesn't
    take the whole node down.
    """
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return None
        try:
            return cast(stripped)
        except ValueError:
            return None
    try:
        return cast(value)
    except (TypeError, ValueError):
        return None


def _parse_optional_int(value: object) -> int | None:
    """Parse a ROS-stringy value into an ``int`` or ``None``.

    Used for ``minimax_pitch`` (issue #1780). See
    :func:`_parse_optional_number`.
    """
    return _parse_optional_number(value, int)


def _parse_optional_float(value: object) -> float | None:
    """Parse a ROS-stringy value into a ``float`` or ``None``.

    Used for ``minimax_volume`` (issue #1780). See
    :func:`_parse_optional_number`.
    """
    return _parse_optional_number(value, float)


def _parse_pronunciation_dict(value: object) -> dict | None:
    """Parse the YAML/ROS string ``minimax_pronunciation_dict`` into a dict.

    Used for ``minimax_pronunciation_dict`` (issue #1780). Accepts:

    * Empty string / ``None`` → ``None`` (field omitted from payload).
    * A JSON-encoded object — parsed via :mod:`json`; the MiniMax T2A v2
      spec asks for ``{"tone": [...], "phoneme": [...], "contextual": [...]}``
      so we expect ``Mapping[str, Sequence[str]]``-shaped payloads.
    * Already a ``Mapping`` — passed through.

    Anything else (``str`` that's not JSON, ``int``, ``list``) is logged
    as "ignored" and we return ``None``. We deliberately do NOT raise
    here: this is operator-config, not user-facing input; crashing the
    node on a typo is worse than silently ignoring the malformed value.
    """
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return None
        try:
            parsed = json.loads(stripped)
        except (ValueError, TypeError):
            return None
    elif isinstance(value, Mapping):
        parsed = value
    else:
        return None
    if not isinstance(parsed, Mapping):
        return None
    return dict(parsed)


# Issue #1996 / operator-agent step 7a — allowed values for the top-level
# ``priority`` field of ``/voice/tts/request``.
#
# Набор ВЫРОВНЕН с ADR-0056: те же три значения, что валидирует
# ``scheduler/pregen/pre_gen.py`` для вложенного ``pregenerate.priority``.
# Раньше здесь был двухзначный набор, и top-level ``priority="personality"``
# молча превращался в ``"normal"`` — молчаливая потеря значения на границе
# двух контрактов. Решение владельца: держать полный набор.
#
# Прецеденция в FIFO-gate (см. ``TTSNode._assign_priority_play_seq``):
#
#   operator     — врезка: запрос встаёт сразу за играющим чанком.
#                  Целевая архитектура §8а.3: «ТАРС не договаривается с
#                  планировщиком личности, он просто говорит роботом».
#   personality  — речь личности, хвост очереди.
#   normal       — немаркированный / legacy-трафик, хвост очереди.
#
# ``personality`` и ``normal`` сегодня по порядку НЕ различаются — обоих
# кладём в хвост. Значение сохраняется отдельно намеренно: оно доезжает до
# планировщика предгенерации и метрик, которым важно, чья это реплика.
# Если появится своя прецеденция у личности — менять здесь, тесты на
# порядок уже есть.
_TTS_PRIORITY_VALUES = frozenset({"operator", "personality", "normal"})

#: Значения, дающие врезку. Отдельная константа, чтобы «кто прыгает
#: очередь» читалось в одном месте, а не выводилось из сравнения строк.
_TTS_PRIORITY_PREEMPTS = frozenset({"operator"})


# Issue #2318 — whitelist поля ``sink`` в ``/voice/tts/request`` и его
# канонизация. Ключ — то, что реально приходит в payload; значение —
# каноническое имя, которым дальше по стеку оперируют
# ``_submit_synthesis(sink=...)`` / ``_sap_publish_for_sink``.
#
# ``"speakers"`` — значение ``Sink.SPEAKERS`` из SoT-сборщика
# ``rob_box_core.utterance`` (ADR-0080 §2.3). Продюсеры (dialogue_node,
# telegram_node, stt_node, startup_greeting_node, core.speak_helpers)
# перешли на него в voice-vr 12 (#2197), а consumer в voice-vr 13 (#2198)
# остался на единственном числе ``"speaker"`` — рассинхрон контракта,
# из-за которого КАЖДАЯ реплика в динамики уходила в DROP (deploy #2318:
# «unknown sink='speakers' ... DROP»). Держим оба написания: SoT-имя и
# исторический ``"speaker"`` (legacy-паблишеры и явный kwarg внутри
# самого узла).
#
# Отсутствие поля и пустая строка → ``"speaker"`` (backward-compat, тот
# же default, что и до фикса).
_VOICE_TTS_SINK_ALIASES = {
    "speaker": "speaker",
    "speakers": "speaker",
    "": "speaker",
    "headset": "headset",
    "preview": "preview",
}


def _normalize_tts_priority(raw: object) -> str:
    """Whitelist-normalize the ``priority`` field of ``/voice/tts/request``.

    Backward-compat contract (issue #1996 DoD): the field is optional, and
    anything outside the whitelist — missing, ``None``, wrong case
    (``"OPERATOR"``), or garbage — is treated as ``"normal"``. A malformed
    payload must never raise or drop the request; it just loses the
    priority bump.

    Whitelist — ``{"operator", "personality", "normal"}``, тот же, что у
    вложенного ``pregenerate.priority`` в ADR-0056. Значение возвращается
    как есть, без схлопывания ``personality`` в ``normal``.
    """
    if raw in _TTS_PRIORITY_VALUES:
        return raw  # type: ignore[return-value]
    return "normal"
