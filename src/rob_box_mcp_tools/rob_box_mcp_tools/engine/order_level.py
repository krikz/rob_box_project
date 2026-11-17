"""Уровень музыки из слов заказа применяет код при старте трека (ADR-0148; живой прогон 28.10, акт 8, n809).

«Поставь фоном ровный ритм на синтезаторе, только негромко» — трек заиграл, а «негромко» пропало: STT дал «не громко»,
из темы выпало «не», тема стала «…громко», громкость музыки не менялась и никто об этом не сказал. Теперь слова заказа
о громкости (:func:`rob_box_dialog.words.asked_level`: «негромко», «потише», «только не громко») не тема, а уровень:
как только заказ зазвучал, код сдвигает мастер-фейдер музыки (``set_music_volume``, ``RenardoRuntime.set_master_gain``)
на шаг в названную сторону. Итог пишется в данные результата (поле ``level``): сказано ли, применено ли и почему нет —
молчаливой потери просьбы нет.

Фейдер один на всю музыку, не на трек: шаг остаётся и для следующих треков, пока человек не скажет «громче» или
``normal`` (так же, как «потише» посреди трека).
"""

from __future__ import annotations

import logging
from dataclasses import replace
from typing import Any, Callable, Dict, Optional

from rob_box_dialog.words import asked_level

from ..base import MCPToolResult

_LOG = logging.getLogger(__name__)

#: Применить шаг громкости музыки: ``louder``/``quieter`` -> результат ``set_music_volume`` (``mcp_server`` ставит тул
#: из реестра; звукового сервера нет - ``None``, и шаг честно не применён).
LevelSetter = Callable[[str], MCPToolResult]
#: Причины, по которым просьба об уровне не применена (поле ``level.reason``).
NO_VOLUME_TOOL = "no_volume_tool"
UNCHANGED = "unchanged"


def apply_level(action: str, setter: Optional[LevelSetter], log: Any = _LOG) -> Dict[str, Any]:
    """Шаг ``action`` через ``setter`` и итог для данных результата: ``asked`` - направление, ``applied`` - фейдер
    сдвинулся, ``percent`` - уровень после, ``reason`` - почему нет (нет тула, ошибка тула, предел фейдера)."""
    if setter is None:
        log.warning(f"🎚️ [level] заказ просит {action}, а тула громкости музыки нет - уровень не изменён")
        return {"asked": action, "applied": False, "reason": NO_VOLUME_TOOL}
    try:
        done = setter(action)
    except Exception as exc:  # noqa: BLE001 - заказ уже зазвучал: сбой ручки не должен его ронять
        log.warning(f"🎚️ [level] {action}: тул громкости упал: {type(exc).__name__}: {exc}")
        return {"asked": action, "applied": False, "reason": f"{type(exc).__name__}"}
    data = done.data or {}
    if not done.success:
        log.warning(f"🎚️ [level] {action}: {done.error or 'отказ'}")
        return {"asked": action, "applied": False, "reason": done.error or "failed"}
    moved = data.get("new_gain") != data.get("old_gain")
    log.info(f"🎚️ [level] заказ просит {action}: {done.message or '—'}")
    return {
        "asked": action,
        "applied": moved,
        "percent": data.get("percent"),
        **({} if moved else {"reason": UNCHANGED}),
    }


def with_level(
    result: MCPToolResult, said: Optional[str], setter: Optional[LevelSetter], log: Any = _LOG
) -> MCPToolResult:
    """Результат заказа, который зазвучал, с применённым уровнем из слов человека (``said`` - реплика хода или текст
    заказа); заказ не зазвучал или об уровне не просили - как был."""
    action = asked_level(said or "")
    if not action or not result.success:
        return result
    return replace(result, data={**(result.data or {}), "level": apply_level(action, setter, log)})


__all__ = ["LevelSetter", "NO_VOLUME_TOOL", "UNCHANGED", "apply_level", "with_level"]
