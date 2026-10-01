"""club_history.py — связка club-движка с историей сыгранного (issue #3224, ADR-0146).

Вынесено из ``ComposeMusicTool`` (бюджет размера класса, ADR-0145).
"""

from __future__ import annotations

from typing import Any, Dict, List, Mapping, Optional, Sequence

from .club_arranger import club_progression
from .club_pools import club_variant
from .music_diversity import MusicHistory

__all__ = ["CLUB_HISTORY_LIMIT", "recent_club_rows", "remember_classic", "remember_club"]

#: Сколько последних записей ``music_history`` учитывает выбор club-каркаса.
CLUB_HISTORY_LIMIT = 20


def recent_club_rows(history: Optional[MusicHistory]) -> List[Dict[str, Any]]:
    """Недавняя история (свежие первыми); ``[]`` — памяти нет или БД недоступна."""
    return history.recent(limit=CLUB_HISTORY_LIMIT) if history is not None else []


def remember_club(
    history: Optional[MusicHistory], kwargs: Mapping[str, Any], kit: Mapping[str, str], seed: int,
    bpm: float, hook_info: Optional[Mapping[str, Any]], recent: Sequence[Mapping[str, Any]],
) -> str:
    """Записать сыгранный club-трек в историю; вернуть INFO-строку про выбор.

    С хуком мелодии прогрессию задаёт хук — в историю она пишется как
    ``None``, чтобы не штрафовать то, чего сид не выбирал.
    """
    scale = kwargs.get("scale") or "minor"
    progression = None if hook_info else club_progression(seed, recent, scale)
    variant = club_variant(seed, recent)
    stored = history is not None and history.record(
        style="club", progression=progression, root=kwargs.get("root") or "A#", bpm=bpm,
        scale=scale, melody_name=hook_info["id"] if hook_info else None,
        fragment_offset=hook_info.get("offset") if hook_info else None,
        hook_fingerprint=hook_info.get("fingerprint") if hook_info else None, **kit, **variant,
    )
    return (
        f"[#3224] club выбор: seed={seed}, progression={progression}, "
        + ", ".join(f"{k}={v}" for k, v in {**kit, **variant}.items())
        + f"; история учтена: {len(recent)} записей, "
        + ("записано" if stored else "НЕ записано (истории нет или БД недоступна)")
    )


def remember_classic(history: Optional[MusicHistory], kwargs: Mapping[str, Any]) -> bool:
    """Записать сыгранный classic-трек (issue #3245).

    Первый трек DJ-сета идёт через ``style=classic`` (``form=buildup`` с явными
    синтами) и раньше в историю не попадал: «помнить сыгранное» не покрывало
    его. Каркас-поля (шаблон, бочка, …) и прогрессию не пишем — у classic их
    нет в club-формате, штрафовать нечего; пишем стиль, имя мелодии, ключ, темп.
    """
    if history is None:
        return False
    name = kwargs.get("name")
    return history.record(
        style="classic", melody_name=str(name) if name else None, root=kwargs.get("root"),
        bpm=kwargs.get("bpm"), scale=kwargs.get("scale"),
    )
