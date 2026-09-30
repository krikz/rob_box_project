"""dj_material.py — материал из сообщения человека → хук ближайшего DJ-перехода (issue #3227).

Живой лог 30.09: Шифу прислал в Telegram Strudel-код Stranger Things «для
материала продолжения сэта», робот повторил прежний ``seed`` — ноты потерялись.
Чтобы материал не зависел от доброй воли LLM, приём идёт кодом:

1. :func:`looks_like_material` — дешёвый regex по тексту реплики;
2. :class:`MaterialIntake` вызывает MCP-тул ``add_music_material`` (парсинг и
   запись в RTTTL-библиотеку — на стороне mcp_server) тем же исполнителем, что
   и медиа-роутер;
3. имя записи кладётся в ``DJState.pending_material``; :func:`choose_melody`
   отдаёт его в ``DJModeController._club_call`` как ``name=`` ближайшего
   перехода вместо мелодии пула, а :func:`consume_material` снимает его, когда
   трек сета реально запущен.

Не распознанное тулом остаётся в диалоге: реплика идёт в LLM как обычно, а
правило skills/dj.txt велит честно сказать «не распознал».
"""

from __future__ import annotations

import ast
import asyncio
import re
import time
import uuid
from typing import Any, Callable, Optional

from rob_box_llm.provider import ToolCall
from rob_box_voice.core.dj_theme_melodies import pick_melody

__all__ = [
    "MATERIAL_TOOL",
    "MATERIAL_TTL_S",
    "MaterialIntake",
    "choose_melody",
    "consume_if_played_in_turn",
    "consume_material",
    "looks_like_material",
    "queue_material",
]

MATERIAL_TOOL = "add_music_material"
#: Сколько секунд принятый материал ждёт своего перехода (потом считается протухшим).
MATERIAL_TTL_S = 3600.0
#: Не гонять тул по огромным простыням (Telegram режет на 4096).
MAX_TEXT_CHARS = 6000

_MATERIAL_RE = re.compile(
    r"\bnote\s*\(|\bn\s*\(\s*[\"'`]|\bsetcpm\s*\(|:\s*d\s*=\s*\d+\s*,\s*o\s*=\s*\d+\s*,\s*b\s*=\s*\d+\s*:",
    re.IGNORECASE,
)


def looks_like_material(text: str) -> bool:
    """Похоже ли, что в реплике есть RTTTL/Strudel-материал (дешёвая проверка до тула)."""
    return bool(text) and len(text) <= MAX_TEXT_CHARS and _MATERIAL_RE.search(text) is not None


def queue_material(state: Any, name: str, now: float) -> None:
    """Запомнить: ближайший трек сета играет материал ``name``."""
    state.pending_material = name
    state.pending_material_at = now


def _pending(state: Any, now: float) -> str:
    name = getattr(state, "pending_material", "")
    fresh = now - getattr(state, "pending_material_at", 0.0) <= MATERIAL_TTL_S
    return name if name and fresh else ""


def consume_material(state: Any) -> None:
    """Трек сета запущен — материал отыгран (или отдан модели), очередь пуста."""
    state.pending_material = ""


def consume_if_played_in_turn(state: Any, turn_text: str) -> None:
    """Ход с материалом в реплике запустил музыку — LLM сыграл его сам, на переходе не повторять.

    Честная оговорка: если приём (``MaterialIntake.ingest``) закончится ПОЗЖЕ
    конца хода, материал встанет в очередь заново и сыграет один раз на переходе.
    """
    if "[DJ_AUTO" not in (turn_text or "") and looks_like_material(turn_text):
        consume_material(state)


def choose_melody(state: Any, now: float, track_no: int) -> str:
    """Мелодия перехода: принятый материал, иначе следующая из пула темы (``""`` — без ``name=``)."""
    return _pending(state, now) or pick_melody(state.melody_pool, track_no, state.played_names)


class MaterialIntake:
    """Приём материала из реплики: ``add_music_material`` через исполнителя ноды → очередь DJ.

    ``dj`` — :class:`DJModeController` (нужен только ``dj.state``); ``clock`` — той же
    эпохи, что часы контроллера (``time.time``).
    """

    def __init__(
        self,
        dj: Any,
        get_executor: Callable[[], Any],
        get_loop: Callable[[], Any],
        logger: Any,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._dj = dj  # ``dj.state`` берётся на месте: контроллер пересоздаёт DJState при сбросе
        self._get_executor = get_executor
        self._get_loop = get_loop
        self._logger = logger
        self._clock = clock

    def on_text(self, text: str) -> bool:
        """Реплика с материалом → фоновый вызов тула. ``True`` — приём запущен."""
        executor, loop = self._get_executor(), self._get_loop()
        if not looks_like_material(text) or executor is None or loop is None:
            return False
        asyncio.run_coroutine_threadsafe(self.ingest(text, executor), loop)
        return True

    async def ingest(self, text: str, executor: Any) -> Optional[str]:
        """Вызвать тул; успех — поставить имя в очередь DJ. Возвращает имя или ``None``."""
        call = ToolCall(id=f"material-{uuid.uuid4().hex[:8]}", name=MATERIAL_TOOL, arguments={"text": text})
        try:
            result = await executor.execute(call)
        except Exception as exc:  # noqa: BLE001 — приём материала не должен ронять ноду
            self._logger.warning(f"🎼 [material] {MATERIAL_TOOL} упал: {exc}")
            return None
        content = str(getattr(result, "content", "") or "")
        name = _name_from_content(content) if not getattr(result, "is_error", False) else None
        self._logger.info(f"🎼 [material] {MATERIAL_TOOL} → name={name!r} result={content[:160]!r}")
        if name:
            queue_material(self._dj.state, name, self._clock())
        return name


def _name_from_content(content: str) -> Optional[str]:
    """``data['name']`` из текста ``ToolResult.content`` (последняя строка — ``repr(data)``)."""
    last = content.rsplit("\n", 1)[-1].strip()
    if not last.startswith("{"):
        return None
    try:
        data = ast.literal_eval(last)
    except (ValueError, SyntaxError, MemoryError, RecursionError):
        return None
    name = data.get("name") if isinstance(data, dict) else None
    return name if isinstance(name, str) and name else None
