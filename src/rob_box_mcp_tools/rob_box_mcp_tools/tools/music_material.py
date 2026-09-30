"""music_material.py — MCP-тул ``add_music_material`` (issue #3227).

Человек прислал ноты в чат (RTTTL / Strudel ``note("...")`` / список нот) —
тул разбирает их, кладёт в RTTTL-библиотеку (``source=user``) и возвращает
имя, по которому дальше играется ``compose_music(style="club", name=...)``.
Вся логика — в :mod:`core.music_material`; здесь только обёртка.
"""

from __future__ import annotations

from typing import List

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from ..core.music_material import parse_material, store_material

__all__ = ["AddMusicMaterialTool"]


class AddMusicMaterialTool(MCPTool):
    """Принять музыкальный материал из сообщения человека и сохранить в библиотеку."""

    def __init__(self, node, library) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "add_music_material"

    @property
    def description(self) -> str:
        return (
            "Принять музыкальный материал, который человек ПРИСЛАЛ в сообщении "
            "(RTTTL-строка, Strudel/Tidal-код note(\"...\")/n(\"...\"), или список "
            "нот «e4 g4 b4 c5»), и сохранить его в библиотеку мелодий. Возвращает "
            "name — им играется хук: compose_music(style=\"club\", name=<name>). "
            "Передай text ДОСЛОВНО как прислал человек. Если не распознал — "
            "вернёт честную ошибку: скажи об этом и не выдумывай ноты."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="text",
                type="string",
                description="Текст сообщения с материалом дословно (код, RTTTL или ноты).",
                required=True,
            ),
            MCPToolParameter(
                name="title",
                type="string",
                description="Название материала (необязательно; иначе из комментария в коде).",
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, text: str, title: str = "") -> MCPToolResult:
        material = parse_material(text or "")
        if material is None:
            head = (text or "").strip().replace("\n", " ")[:60]
            return MCPToolResult(
                success=False,
                error=(
                    f"не распознал музыкальный материал: {head!r}. Понимаю RTTTL-строку, "
                    "Strudel note(\"...\")/n(\"...\") и список нот вроде «e4 g4 b4 c5». "
                    "Скажи об этом честно, не выдумывай ноты."
                ),
            )
        name, added = store_material(self._library, material, title=(title or "").strip())
        note = "сохранил" if added else "такой материал уже был в библиотеке"
        return MCPToolResult(
            success=True,
            data={
                "name": name,
                "bpm": material.bpm,
                "notes": len(material.sounding()),
                "source_format": material.source_format,
                "pattern": material.detail,
                "added": added,
            },
            message=(
                f"Материал ({material.describe()}): {note}. "
                f"Играть: compose_music(style=\"club\", name=\"{name}\")."
            ),
        )
