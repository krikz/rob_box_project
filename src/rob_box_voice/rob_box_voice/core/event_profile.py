"""Event profile loader + FAQ-event instructions renderer.

Extracted from :class:`DialogueNode` so the same YAML schema
(``event: {name, faq_file, location, ...}``) and the same
``[EVENT MODE] / [EVENT FAQ PREFETCH]`` prompt blocks can be reused
by any future harness-style adapter. Pure functions: no I/O is
performed at import time, no ROS2 node is required.

Two top-level helpers:

* :func:`load_event_profile` — parses a YAML file into an
  :class:`EventProfile` dataclass (or returns ``None`` when the
  feature is disabled / the file is missing).
* :func:`render_event_instructions` — prepends an ``[EVENT MODE]``
  block to the agent's base instructions.
* :func:`build_event_faq_prefetch_context` — runs an FAQ search and
  formats the hits into a `[EVENT FAQ PREFETCH]` system-prompt block
  for the current turn.

The dataclass keeps the shape stable so downstream callers can
``profile["event_id"]`` or ``profile.event_id`` interchangeably.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional

import yaml

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Slug helper (was DialogueNode._slugify_event_id)
# ---------------------------------------------------------------------------


def slugify_event_id(value: str) -> str:
    """Normalise an event name into a YAML-safe slug.

    >>> slugify_event_id("RoboConf 2026!")
    'roboconf-2026'
    """
    slug = re.sub(r"[^a-zA-Z0-9]+", "-", value.lower()).strip("-")
    return slug or "faq-event"


# ---------------------------------------------------------------------------
# EventProfile dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class EventProfile:
    """Immutable view of an event-mode profile."""

    event_id: str
    name: str
    organization: str
    location: str
    date: str
    description: str
    robot_role: str
    intro_identity: str
    faq_file: str

    def as_dict(self) -> dict[str, str]:
        """Return a plain dict (matches the legacy DialogueNode API)."""
        return {
            "event_id": self.event_id,
            "name": self.name,
            "organization": self.organization,
            "location": self.location,
            "date": self.date,
            "description": self.description,
            "robot_role": self.robot_role,
            "intro_identity": self.intro_identity,
            "faq_file": self.faq_file,
        }


# ---------------------------------------------------------------------------
# YAML loader
# ---------------------------------------------------------------------------


_DEFAULT_ROBOT_ROLE = "РОББОКС — ровер-помощник"


def load_event_profile(
    *,
    enabled: bool,
    config_file: str,
) -> Optional[EventProfile]:
    """Load an :class:`EventProfile` from ``config_file``.

    Args:
        enabled: Whether FAQ mode is on (caller passes the
            ``faq_mode_enabled`` ROS2 param).
        config_file: Path to the YAML event config (caller passes
            ``faq_event_config_file``).

    Returns:
        An :class:`EventProfile`, or ``None`` if disabled / missing /
        malformed. Any error is logged but never raised.
    """
    if not enabled:
        return None
    if not config_file:
        logger.warning("⚠️ FAQ mode enabled but faq_event_config_file is empty")
        return None

    config_path = Path(config_file).expanduser()
    if not config_path.is_absolute():
        config_path = config_path.resolve()
    if not config_path.exists():
        logger.warning(f"⚠️ FAQ event config not found: {config_path}")
        return None

    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        logger.error(f"❌ Failed to read FAQ event config: {exc}")
        return None

    event = payload.get("event", payload) if isinstance(payload, dict) else None
    if not isinstance(event, dict):
        logger.warning("⚠️ FAQ event config must be a mapping or contain 'event:' block")
        return None

    name = str(event.get("name", "")).strip()
    faq_file = str(event.get("faq_file", "")).strip()
    if not name or not faq_file:
        logger.warning("⚠️ FAQ event config requires 'name' and 'faq_file'")
        return None

    faq_path = Path(faq_file).expanduser()
    if not faq_path.is_absolute():
        faq_path = (config_path.parent / faq_path).resolve()

    date = str(event.get("date", "")).strip()
    raw_id = str(event.get("id", "")).strip()
    event_id = raw_id or slugify_event_id(f"{name}-{date or faq_path.stem}")

    return EventProfile(
        event_id=event_id,
        name=name,
        organization=str(event.get("organization", "")).strip(),
        location=str(event.get("location", "")).strip(),
        date=date,
        description=str(event.get("description", "")).strip(),
        robot_role=str(event.get("robot_role", _DEFAULT_ROBOT_ROLE)).strip(),
        intro_identity=str(event.get("intro_identity", "")).strip(),
        faq_file=str(faq_path),
    )


# ---------------------------------------------------------------------------
# Instruction renderers
# ---------------------------------------------------------------------------


def render_event_instructions(
    profile: EventProfile | Mapping[str, Any] | None,
    base_instructions: str,
    *,
    faq_store_available: bool = False,
) -> str:
    """Prepend an ``[EVENT MODE]`` block to ``base_instructions``.

    If ``profile`` is falsy, the base instructions are returned
    unchanged. ``faq_store_available`` controls the FAQ-related
    guidance lines.
    """
    if not profile:
        return base_instructions
    p = profile.as_dict() if isinstance(profile, EventProfile) else dict(profile)

    lines = [
        "[EVENT MODE]",
        f"Ты работаешь на мероприятии как {p['robot_role']}.",
        f"Мероприятие: {p['name']}.",
    ]
    if p.get("organization"):
        lines.append(f"Организация: {p['organization']}.")
    if p.get("location"):
        lines.append(f"Локация: {p['location']}.")
    if p.get("date"):
        lines.append(f"Дата: {p['date']}.")
    if p.get("description"):
        lines.append(f"Описание: {p['description']}")
    if p.get("intro_identity"):
        lines.append(f"Самоидентификация: {p['intro_identity']}")
    if faq_store_available:
        lines.append(
            "Если вопрос касается мероприятия, FAQ, поступления, программы, локации или организационных деталей, "
            "используй FAQ retrieval tool и говори по найденным данным."
        )
        lines.append(
            "Если пользователь просит рэп, стих, шутку, историю, песню или другой перформанс про тему мероприятия, "
            "сначала подними факты из FAQ и только потом стилизуй ответ."
        )
        lines.append(
            "Если после FAQ нужен бит или музыка, сначала получи факты, потом при необходимости вызови handle_music, "
            "и только после этого озвучивай ответ."
        )
    lines.append(
        "Даже если исходный ответ в FAQ длинный, озвучивай короткую, понятную, разговорную версию: 1-3 предложения."
    )
    lines.append(
        "Если точного ответа в FAQ нет, честно скажи это и не выдумывай детали."
    )
    return "\n".join(lines) + "\n\n" + base_instructions


def render_faq_skill_prompt(
    profile: EventProfile | Mapping[str, Any] | None,
    base_prompt: str,
) -> str:
    """Prepend event details to the FAQ sub-agent prompt."""
    if not profile:
        return base_prompt
    p = profile.as_dict() if isinstance(profile, EventProfile) else dict(profile)

    lines = [
        f"Активное мероприятие: {p['name']}",
        f"Роль робота: {p['robot_role']}",
    ]
    if p.get("organization"):
        lines.append(f"Организация: {p['organization']}")
    if p.get("location"):
        lines.append(f"Локация: {p['location']}")
    if p.get("date"):
        lines.append(f"Дата: {p['date']}")
    if p.get("description"):
        lines.append(f"Описание: {p['description']}")
    return "\n".join(lines) + "\n\n" + base_prompt


# ---------------------------------------------------------------------------
# FAQ prefetch
# ---------------------------------------------------------------------------


def build_event_faq_prefetch_context(
    *,
    profile: EventProfile | Mapping[str, Any] | None,
    faq_store: Any,
    user_input: str,
    limit: int = 3,
    logger_obj: logging.Logger | None = None,
) -> Optional[str]:
    """Run an FAQ search for ``user_input`` and format the hits.

    Returns ``None`` when FAQ mode is disabled or the store raises.
    Any internal error is logged but swallowed.
    """
    log = logger_obj or logger
    if not faq_store or not profile:
        return None

    query = user_input.strip()
    event_id = (
        profile.event_id
        if isinstance(profile, EventProfile)
        else dict(profile).get("event_id")
    )
    if not query or not event_id:
        return None

    try:
        results = faq_store.search(query=query, event_id=event_id, limit=limit)
    except Exception as exc:
        log.warning(f"⚠️ Event FAQ prefetch failed: {exc}")
        return None

    log.info(f"📚 Event FAQ prefetch: {len(results)} matches for '{query[:80]}'")
    if not results:
        return None

    lines = [
        "[EVENT FAQ PREFETCH]",
        "FAQ для текущего запроса уже проверен. Для всех фактических утверждений о мероприятии опирайся сначала на данные ниже.",
        "Если пользователь просит рэп, шутку, стих, историю, песню или другой стиль по теме мероприятия, сначала используй FAQ факты, а уже потом стилизуй ответ.",
        "Если нужен бит или музыкальный фон, сначала используй факты ниже, затем при необходимости вызови handle_music, а потом озвучь ответ.",
        "Если фактов ниже недостаточно, честно скажи, что точных деталей в FAQ не найдено, и не выдумывай их.",
        "Найденные FAQ факты:",
    ]
    for index, item in enumerate(results, start=1):
        lines.append(f"{index}. Вопрос: {item.get('question', '')}")
        lines.append(f"   Ответ: {item.get('answer', '')}")
        category = item.get("category")
        if category:
            lines.append(f"   Категория: {category}")
        source = item.get("source")
        if source:
            lines.append(f"   Источник: {source}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Pure-helper that needs an FAQ-store-like adapter (search() contract)
# ---------------------------------------------------------------------------


__all__ = [
    "EventProfile",
    "slugify_event_id",
    "load_event_profile",
    "render_event_instructions",
    "render_faq_skill_prompt",
    "build_event_faq_prefetch_context",
]