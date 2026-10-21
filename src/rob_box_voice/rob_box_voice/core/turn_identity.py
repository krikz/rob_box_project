"""Личность хода для ``<user_profile>`` — решает код (issue #3598).

Живой лог 13.10 08:38 UTC: товарищ Шифу написал из Telegram (chat
``495039871``) «продолжай играть вагнера», а ``<user_profile>`` хода
получил последний голос из комнаты — ``Литвин``, 0.82. Модель решила, что
это «не директорский приказ, а гостевая реплика», и вместо ответа
рассуждала вслух.

Голосовая биометрия описывает того, кто говорит в микрофон. К сообщению
из Telegram она отношения не имеет: личность TG-хода определяется по
chat_id, голос комнаты к ней не применяется. Это единственное место,
где знают, кто владелец.

Владельцы — ``TELEGRAM_OWNER_CHAT_IDS`` (через запятую, env контейнера
voice-assistant); без неё — :data:`DEFAULT_OWNER_CHAT_IDS`. Остальные
разрешённые чаты — гости (``RULE #SRC`` промпта: «The operator (Шифу) is
the DIRECTOR … Other chat users are guests»).

Модуль без ROS2 — тестируется напрямую.
"""

from __future__ import annotations

import os
from typing import Any, Mapping, Optional

#: Env со списком chat_id владельцев.
OWNER_CHAT_IDS_ENV: str = "TELEGRAM_OWNER_CHAT_IDS"

#: chat_id товарища Шифу (он же в ``scripts/music/live_dj/s7_styles.sh``).
DEFAULT_OWNER_CHAT_IDS: frozenset = frozenset({495039871})

#: Имя владельца — как его называет ``RULE #SRC`` в промпте.
OWNER_NAME: str = "Шифу"

# Issue #2779 — незнакомец получил факты и имя Бориса: биометрия верно
# сказала unknown, но LLM всё равно нашла в истории диалога/памяти данные
# ПРЕДЫДУЩЕГО известного собеседника и адресовала их новому человеку. Явный
# запрет прямо рядом с <name>unknown</name> — LLM не должна домысливать
# личность по истории/памяти, когда биометрия честно говорит «не знаю».
_UNKNOWN_VOICE_NOTE: str = (
    "    <privacy_note>Текущий собеседник НЕ опознан голосом. "
    "НЕ обращайся к нему по имени другого (даже недавнего) "
    "диктора и не пересказывай факты/предпочтения, "
    "относящиеся к другому диктору — ни из истории этого "
    "диалога, ни из долговременной памяти. Отвечай по сути "
    "вопроса, не приписывая его чужой личности.</privacy_note>"
)

# Issue #1787 — внутренняя кличка. Даёт LLM якорь, когда имён два
# одинаковых или имени нет вовсе. Research §5.3: озвучивать её НЕЛЬЗЯ — юзер
# этого слова никогда не слышал. Issue #2864 — живой прогон: на «кого
# запомнил?» LLM насчитала троих при двух профилях, приняв кличку Саши
# «Незнакомец» за отдельного человека. Правило явно говорит, что кличка —
# второе обозначение ЭТОГО собеседника и при подсчёте людей не считается.
_EPITHET_RULE: str = (
    "    <epithet_rule>Внутренняя метка робота для различения "
    "тёзок. НИКОГДА не произноси её вслух и не упоминай в "
    "ответе — используй только как признак «это тот же "
    "человек». Кличка — НЕ отдельный человек, а второе "
    "обозначение ЭТОГО собеседника: когда считаешь или "
    "перечисляешь знакомых, не считай и не называй её как "
    "ещё одного человека.</epithet_rule>"
)


def owner_chat_ids(environ: Optional[Mapping[str, str]] = None) -> frozenset:
    """chat_id владельцев: из env, иначе :data:`DEFAULT_OWNER_CHAT_IDS`.

    Нечисловые элементы списка пропускаются; пустой или целиком мусорный
    env — значение по умолчанию.
    """
    raw = (environ if environ is not None else os.environ).get(OWNER_CHAT_IDS_ENV, "")
    ids = set()
    for part in raw.split(","):
        try:
            ids.add(int(part.strip()))
        except ValueError:
            continue
    return frozenset(ids) or DEFAULT_OWNER_CHAT_IDS


def telegram_profile_lines(
    chat_id: int, owners: Optional[frozenset] = None
) -> list:
    """Строки ``<user_profile>`` хода из Telegram — без голосовой биометрии.

    Владелец → имя :data:`OWNER_NAME` и ``role=director``; иной чат →
    ``unknown`` и ``role=guest``. Ни ``voice_confidence``, ни
    ``speaker_id``, ни клички голоса комнаты сюда не попадают.
    """
    if chat_id in (owners if owners is not None else owner_chat_ids()):
        name, role = OWNER_NAME, "director"
    else:
        name, role = "unknown", "guest"
    return [
        f"    <name>{name}</name>",
        "    <channel>telegram</channel>",
        f"    <role>{role}</role>",
    ]


def voice_profile_lines(name: str, speaker: Mapping[str, Any]) -> list:
    """Строки ``<user_profile>`` голосового хода — по биометрии ``speaker``.

    ``name`` — уже очищенное ``sanitize_speaker_name`` имя (его же нода
    использует для бэклога). Issue #2440 — ``speaker_id`` полный, без
    усечения: LLM и MCP-тулы должны видеть тот же id, что в voice_memory.
    """
    introduced = str(speaker.get("introduced_name") or "")
    if introduced:  # #3765: назвался сам, голос похож на другого — выясняет робот; LLM — только имя, без биометрии
        return [f"    <name>{introduced}</name>", "    <name_source>назвался сам в этой реплике</name_source>"]
    lines = [f"    <name>{name or 'unknown'}</name>"]
    if not name:
        lines.append(_UNKNOWN_VOICE_NOTE)
    confidence = float(speaker.get("confidence") or 0.0)
    if confidence:
        lines.append(f"    <voice_confidence>{confidence:.2f}</voice_confidence>")
    speaker_id = speaker.get("speaker_id") or ""
    if speaker_id:
        lines.append(f"    <speaker_id>{speaker_id}</speaker_id>")
    epithet = str(speaker.get("epithet") or "").strip()
    if epithet:
        lines.append(f"    <epithet internal=\"true\">{epithet}</epithet>")
        lines.append(_EPITHET_RULE)
    return lines


def user_profile_lines(
    name: str, speaker: Mapping[str, Any], tg_chat_id: Optional[int]
) -> list:
    """Тело ``<user_profile>`` хода: TG — по chat_id, голос — по биометрии."""
    if tg_chat_id is not None:
        return telegram_profile_lines(tg_chat_id)
    return voice_profile_lines(name, speaker)


__all__ = [
    "DEFAULT_OWNER_CHAT_IDS",
    "OWNER_CHAT_IDS_ENV",
    "OWNER_NAME",
    "owner_chat_ids",
    "telegram_profile_lines",
    "user_profile_lines",
    "voice_profile_lines",
]
