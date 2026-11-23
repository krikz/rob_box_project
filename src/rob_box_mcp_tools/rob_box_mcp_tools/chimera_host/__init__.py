"""Хост синтеза Химеры (ADR-0175 §2.1, карточка #5205).

Бинарь в репозиторий не входит: он собирается из закреплённого SHA и кладётся рядом с данными.
Здесь только проверка пина и сборка команды. Текст песни сюда не приходит.
"""

from rob_box_mcp_tools.chimera_host.invoke import (
    DATA_DIR,
    PINNED_SHA,
    TIMBRES,
    ChimeraHost,
    host_dir,
)

__all__ = ["DATA_DIR", "PINNED_SHA", "TIMBRES", "ChimeraHost", "host_dir"]
