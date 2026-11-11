"""Мастер-промпт — персона и протокол хода, а не свод правил (ADR-0150 §7.3, A5; ADR-0148; #3696; PR-15 15-rule #3711).

Знание о тулах живёт в их схемах (видны только тулы среза хода), данные — в снимке хода, правила модели — в таблице
``knowledge.PROMPT_RULES`` (в основе промпта их нет). Системный промпт агента — основа + правила — обязан влезать в
бюджет системного блока ``knowledge.CONTEXT_BUDGET_TOKENS["system"]``. Основа Личности — данные ядра с персоной
профиля робота (ADR-0167 Ш1б-3), основа телеграма — данные ядра с именем продукта из профиля (Ш1б-4).
"""

from pathlib import Path

import pytest

from rob_box_core.token_estimate import estimate_tokens
from rob_box_dialog import knowledge as kn
from rob_box_dialog.context import SNAPSHOT_TAG
from rob_box_dialog.profile import load_profile
from rob_box_dialog.prompts import personality_prompt, telegram_prompt
from rob_box_dialog_host.dialog_v2_ports import system_prompt

PACKAGE = Path(__file__).resolve().parents[2]
PROFILE = PACKAGE.parent.parent / "docker" / "sound" / "config" / "assistant.yaml"
AGENTS = ("personality", "telegram")


def _base(agent: str) -> str:
    profile = load_profile(PROFILE)
    if agent == "personality":
        return personality_prompt(profile.persona)
    return telegram_prompt(profile.name)


def _system(agent: str) -> str:
    return system_prompt(_base(agent), agent)


@pytest.mark.parametrize("agent", AGENTS)
def test_system_prompt_fits_the_system_block_budget(agent):
    assert estimate_tokens(_system(agent)) <= kn.CONTEXT_BUDGET_TOKENS["system"]


@pytest.mark.parametrize("agent", AGENTS)
def test_system_prompt_carries_every_rule_of_the_agent_from_knowledge(agent):
    """Правила агента берутся из таблицы знания и приходят в промпт целиком, в её порядке."""
    system = _system(agent)
    rules = [rule.text for rule in kn.PROMPT_RULES if agent in rule.agents]
    assert rules, "у агента нет правил"
    assert all(text in system for text in rules)
    assert [system.index(text) for text in rules] == sorted(system.index(text) for text in rules)


@pytest.mark.parametrize("agent", AGENTS)
def test_system_prompt_teaches_no_pause_tags(agent):
    """Модель пишет голый текст, паузы ставит код (ADR-0160 §2.1, #4238): тега паузы в системном промпте нет."""
    assert "<break" not in _system(agent)


@pytest.mark.parametrize("agent", AGENTS)
def test_system_prompt_teaches_no_speech_tool_of_v1(agent):
    """Речь v2 — поле ``speech`` команды (#4354 B1): тулы-каналы речи v1 (``knowledge.SPEECH_CHANNEL_TOOLS``) системный
    промпт агента не называет — форму ответа несёт только фрагмент knowledge, который дописывает мост."""
    system = _system(agent)
    assert not [name for name in kn.SPEECH_CHANNEL_TOOLS if name in system]


def test_prompt_names_only_snapshot_sections_that_exist():
    """Промпт ссылается на разделы снимка по именам — каждое имя есть в таблице ``CONTEXT_SECTIONS``."""
    system = _system("personality")
    assert f"<{SNAPSHOT_TAG}>" in system
    named = {s for s in ("user_profile", "tts_context", "speech_gender", "providers", "music_state",
                         "recent_unaddressed", "shift_log", "hardware", "tts_voice", "reminder") if f"`{s}`" in system}
    assert named <= set(kn.CONTEXT_SECTIONS)
