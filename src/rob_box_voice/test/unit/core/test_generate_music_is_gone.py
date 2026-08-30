"""
test_generate_music_is_gone.py — промпты не должны звать мёртвый тул.

🔴 Живой прогон 30.08.2026. Пользователь: «спой песню про денчика». Гуард
не увидел музыкального тула и отправил CRITICAL-ретрай, который ТРЕБОВАЛ
вызвать ``generate_music``. Вызвать его невозможно: MiniMax Music API
отключён 20.08.2026 (410 Gone), и ``mcp_server`` с тех пор его НЕ
регистрирует — в живом списке из 52 тулов на роботе его нет::

    mcp_server: MiniMax music generation disabled (API discontinued 410 Gone).
                gen_* library tools only.

Модель дважды отвечала прозой, бюджет ретраев кончался, и пользователь
слышал «Я тут растерялся — бит не запустился».

Каталог тулов (``rob_box_core/_tool_catalog_data.py``) тут не помощник: класс
``MinimaxGenerateMusicTool`` жив и в каталог попадает — не регистрируется
только на сервере. Поэтому источник правды здесь — сам ``mcp_server``.

Тест связывает две стороны: пока сервер не регистрирует тул, ни один промпт,
которым мы командуем модели, не имеет права требовать его вызова. Если
MiniMax когда-нибудь вернут — тест покраснеет и потребует вернуть промпты,
а не наоборот.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rob_box_voice.core.dialogue_guards import (
    build_babble_retry_prompt,
    build_music_retry_prompt,
)


_REPO = Path(__file__).resolve().parents[4]
MCP_SERVER = _REPO / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "mcp_server.py"
MUSIC_SKILL_PROMPT = (
    _REPO / "rob_box_voice" / "prompts" / "skills" / "music_skill_prompt.txt"
)
MASTER_PROMPT = (
    _REPO / "rob_box_voice" / "prompts" / "master_prompt_compact.txt"
)

DEAD_TOOL = "generate_music"


def _server_registers_generate_music() -> bool:
    """Регистрирует ли ``mcp_server`` тул ``generate_music``.

    Ищем живую регистрацию, а не упоминание в комментарии: строка вида
    ``self.register_tool(MinimaxGenerateMusicTool(...))`` без ведущего ``#``.
    """
    if not MCP_SERVER.exists():
        pytest.skip(f"mcp_server.py не найден: {MCP_SERVER}")
    for raw in MCP_SERVER.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip()
        if line.startswith("#"):
            continue
        if "register_tool" in line and "GenerateMusic" in line:
            return True
    return False


def test_server_still_does_not_register_generate_music() -> None:
    """Предпосылка остальных проверок. Красный = MiniMax вернули.

    Тогда правки ниже надо ОТКАТИТЬ (вернуть generate_music в промпты), а не
    ослаблять тест.
    """
    assert not _server_registers_generate_music(), (
        "mcp_server снова регистрирует generate_music — верни его в "
        "music_skill_prompt.txt и в build_music_retry_prompt, затем поправь "
        "этот тест"
    )


def test_music_retry_prompt_does_not_demand_a_dead_tool() -> None:
    """CRITICAL-ретрай Bug C не должен требовать несуществующий тул."""
    if _server_registers_generate_music():
        pytest.skip("generate_music снова зарегистрирован")
    prompt = build_music_retry_prompt("спой песню про денчика")
    assert DEAD_TOOL not in prompt, (
        "build_music_retry_prompt требует generate_music, которого нет на "
        "сервере — модель не сможет его вызвать и ответит прозой"
    )
    # И указывает на то, что действительно работает.
    assert "compose_music" in prompt


def test_babble_retry_prompt_does_not_demand_a_dead_tool() -> None:
    if _server_registers_generate_music():
        pytest.skip("generate_music снова зарегистрирован")
    assert DEAD_TOOL not in build_babble_retry_prompt("спой песню про кота")


def test_music_skill_prompt_never_orders_the_dead_tool() -> None:
    """В скилл-промпте тул может УПОМИНАТЬСЯ, но только как недоступный.

    Полностью вычищать 1150-строчный промпт рискованно (см. RC3 в
    docs/analysis/2026-08-30-music-quality-audit.md: противоречивые запреты
    схлопывают модель в один шаблон). Поэтому контракт мягче: любая строка,
    где тул встречается, обязана быть либо стрелкой-маршрутом на что-то
    другое, либо помеченной как устаревшая/недоступная.
    """
    if _server_registers_generate_music():
        pytest.skip("generate_music снова зарегистрирован")
    if not MUSIC_SKILL_PROMPT.exists():
        pytest.skip(f"промпт не найден: {MUSIC_SKILL_PROMPT}")

    text = MUSIC_SKILL_PROMPT.read_text(encoding="utf-8-sig")
    assert "RULE #GONE" in text, (
        "в music_skill_prompt нет блока RULE #GONE — модель не узнает, что "
        "generate_music недоступен"
    )

    # Директива = строка, которая МАРШРУТИЗИРУЕТ в мёртвый тул («→ generate_music»)
    # или требует его вызвать.
    offenders = []
    for n, line in enumerate(text.splitlines(), 1):
        if DEAD_TOOL not in line:
            continue
        low = line.lower()
        exempt = any(
            marker in line
            for marker in ("RULE #GONE", "STALE", "GONE", "discontinued",
                           "NEVER call", "was the 7th", "WAS a", "not registered",
                           "DOES NOT EXIST")
        )
        if exempt:
            continue
        routes_to_dead = bool(re.search(r"(→|->)\s*" + DEAD_TOOL, line))
        demands = ("must be called" in low and DEAD_TOOL in line)
        if routes_to_dead or demands:
            offenders.append(f"{n}: {line.strip()}")

    assert not offenders, (
        "строки маршрутизируют в недоступный generate_music:\n  "
        + "\n  ".join(offenders)
    )


def test_master_prompt_does_not_route_to_the_dead_tool() -> None:
    if _server_registers_generate_music():
        pytest.skip("generate_music снова зарегистрирован")
    if not MASTER_PROMPT.exists():
        pytest.skip(f"промпт не найден: {MASTER_PROMPT}")
    text = MASTER_PROMPT.read_text(encoding="utf-8-sig")
    offenders = [
        f"{n}: {line.strip()}"
        for n, line in enumerate(text.splitlines(), 1)
        if re.search(r"(→|->)\s*" + DEAD_TOOL, line)
    ]
    assert not offenders, "master_prompt маршрутизирует в мёртвый тул:\n  " + "\n  ".join(offenders)
