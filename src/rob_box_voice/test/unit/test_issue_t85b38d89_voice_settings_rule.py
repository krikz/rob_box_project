#!/usr/bin/env python3
"""
test_issue_t85b38d89_voice_settings_rule.py — prompt-contract test для
kanban-задачи t_85b38d89 (LLM-prompt: emotion/pitch/volume + skill set_pitch/set_volume).

Контекст:
* LLM знает про SSML `<prosody>`, но MiniMax не понимает SSML (text
  strip до синтеза), Yandex игнорирует pitch-атрибут. Skill
  ``set_pitch``/``set_volume`` существуют, но не вызываются (0
  вызовов за 5 мин в логах).
* Корневая причина 1: ``status_skill.py`` объявлял LLM сигнатуру
  ``set_pitch(pitch: float)`` (0.5..2.0), а реальный MCP-инструмент
  (``SetPitchTool``) принимает ``action: enum ∈ {higher, lower,
  normal}`` — step-control по ±0.2. LLM генерил вызовы с float, они
  не проходили schema-validation и тихо отбрасывались.
* Корневая причина 2: мастер-промпт говорил LLM использовать
  ``<prosody pitch="high">``, но на minimax/yandex атрибут pitch
  молча отбрасывается, и юзер слышит обычный голос — request "looks
  ignored".

Фикс:
1. ``status_skill.py`` объявляет ``set_pitch(action: str)`` —
   синхронно с MCP ``SetPitchTool``.
2. ``prompts/skills/voice-tts.txt`` документирует enum и step-control
   семантику; явно запрещает SSML pitch-атрибут как workaround.
3. ``master_prompt_compact.txt`` имеет новый RULE #VOICE-SETTINGS
   с provider-conditional cheat sheet (minimax/yandex/silero).

Этот тест фиксирует wording RULE #VOICE-SETTINGS как prompt-contract.
Если кто-то сократит правило или удалит cross-provider guidance —
тест сломается, что и нужно.

Refs:
* t_85b38d89 — kanban-карточка с этим fix-path
* src/rob_box_voice/prompts/skills/voice-tts.txt — фрагмент скилла
  (change skill-scoped-dialogue-context: status_skill.py удалён как
  мёртвый код Compositor'а, правило переехало сюда)
* src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py — MCP SetPitchTool
* src/rob_box_voice/rob_box_voice/tts_node.py:2794-2797 — Yandex pitch ignored
"""

from __future__ import annotations

from pathlib import Path
import re


MASTER_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "master_prompt_compact.txt"
)

_SKILL_FRAGMENT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "skills"
    / "voice-tts.txt"
)


def _read(prompt_path: Path) -> str:
    return prompt_path.read_text(encoding="utf-8")


def _voice_settings_rule_block(content: str) -> str:
    """Извлечь текст RULE #VOICE-SETTINGS."""
    m = re.search(
        r"🚨 \*\*RULE #VOICE-SETTINGS[^\n]*\*\*:.*?(?=\n# |\n🚨 \*\*RULE #|\Z)",
        content,
        re.DOTALL,
    )
    assert m is not None, (
        "master_prompt_compact.txt потерял RULE #VOICE-SETTINGS — LLM не будет "
        "знать, что prosody pitch молча отбрасывается на minimax/yandex и "
        "нужно вызывать set_pitch/set_volume как отдельный tool."
    )
    return m.group(0)


# ── master_prompt_compact.txt ────────────────────────────────────────


def test_master_prompt_contains_voice_settings_rule() -> None:
    """RULE #VOICE-SETTINGS должен присутствовать в мастер-промпте."""
    content = _read(MASTER_PROMPT)
    assert "RULE #VOICE-SETTINGS" in content, (
        "master_prompt_compact.txt потерял RULE #VOICE-SETTINGS — нужен явный "
        "блок про provider-conditional prosody/set_pitch/set_volume."
    )


def test_voice_settings_rule_documents_minimax_yandex_pitch_drop() -> None:
    """Правило должно явно говорить, что pitch отбрасывается на
    minimax и yandex (а не молчаливо подразумевать)."""
    content = _read(MASTER_PROMPT)
    rule = _voice_settings_rule_block(content)
    assert "minimax" in rule.lower() or "MiniMax" in rule, (
        "RULE #VOICE-SETTINGS должен упомянуть minimax провайдера"
    )
    assert "yandex" in rule.lower() or "Yandex" in rule, (
        "RULE #VOICE-SETTINGS должен упомянуть yandex провайдера"
    )
    # "silently dropped" / "silently ignored" — оба варианта приемлемы
    assert re.search(
        r"silently (dropped|ignored)",
        rule,
        re.IGNORECASE,
    ), (
        "RULE #VOICE-SETTINGS должен явно предупреждать, что pitch "
        "silently dropped/ignored на minimax/yandex"
    )


def test_voice_settings_rule_advertises_set_pitch_set_volume_actions() -> None:
    """Правило должно документировать сигнатуру set_pitch(action)/set_volume(action)."""
    content = _read(MASTER_PROMPT)
    rule = _voice_settings_rule_block(content)
    # set_pitch: action ∈ {higher, lower, normal}
    assert "higher" in rule and "lower" in rule and "normal" in rule, (
        "RULE #VOICE-SETTINGS должен перечислить enum set_pitch: "
        "higher/lower/normal"
    )
    # set_volume: action ∈ {louder, quieter, max, normal}
    assert "louder" in rule and "quieter" in rule and "max" in rule, (
        "RULE #VOICE-SETTINGS должен перечислить enum set_volume: "
        "louder/quieter/max/normal"
    )


def test_voice_settings_rule_forbids_inline_prosody_pitch() -> None:
    """Правило должно явно запрещать LLM использовать <prosody pitch=...>
    как workaround для изменения голоса."""
    content = _read(MASTER_PROMPT)
    rule = _voice_settings_rule_block(content)
    assert "prosody pitch" in rule.lower(), (
        "RULE #VOICE-SETTINGS должен упомянуть тег `<prosody pitch=...>` "
        "и запретить его как workaround"
    )
    # ❌ marker — это анти-pattern
    assert "❌" in rule, (
        "RULE #VOICE-SETTINGS должен содержать ❌ marker для запрета"
    )


def test_voice_settings_rule_has_provider_cheat_sheet() -> None:
    """Правило должно содержать таблицу provider → support matrix."""
    content = _read(MASTER_PROMPT)
    rule = _voice_settings_rule_block(content)
    # markdown-таблица с тремя колонками провайдеров
    assert "| `minimax`" in rule or "| minimax" in rule.lower(), (
        "RULE #VOICE-SETTINGS должен содержать строку таблицы для minimax"
    )
    assert "| `yandex`" in rule or "| yandex" in rule.lower(), (
        "RULE #VOICE-SETTINGS должен содержать строку таблицы для yandex"
    )
    assert "silero" in rule.lower(), (
        "RULE #VOICE-SETTINGS должен содержать строку таблицы для silero (fallback)"
    )


def test_voice_settings_rule_anchors_to_tts_context_tag() -> None:
    """Правило должно ссылаться на <tts_context> / [TTS] provider тег —
    чтобы LLM знал, откуда брать активный провайдер."""
    content = _read(MASTER_PROMPT)
    rule = _voice_settings_rule_block(content)
    assert "[TTS]" in rule and "provider" in rule, (
        "RULE #VOICE-SETTINGS должен ссылаться на `<tts_context>` / `[TTS] provider:` "
        "тег для определения активного провайдера"
    )


# ── prompts/skills/voice-tts.txt ───────────────────────────────────────────


def test_voice_tts_fragment_documents_set_pitch_action_enum() -> None:
    """Фрагмент обязан документировать set_pitch как action-enum.

    Формулировка русская (фрагменты скиллов пишутся на языке диалога),
    поэтому проверяем СУТЬ: назван инструмент и назван параметр action.
    Инвариант тот же, что был у английского status_skill_prompt.txt:
    LLM не должна считать, что pitch задаётся числом (t_85b38d89).
    """
    content = _read(_SKILL_FRAGMENT)
    assert "set_pitch" in content, (
        "prompts/skills/voice-tts.txt потерял упоминание set_pitch"
    )
    assert re.search(r"set_pitch.{0,80}action", content, re.S), (
        "prompts/skills/voice-tts.txt не связывает set_pitch с параметром action"
    )
    # enum values
    assert "higher" in content and "lower" in content and "normal" in content, (
        "prompts/skills/voice-tts.txt должен перечислять set_pitch enum: "
        "higher/lower/normal"
    )


def test_voice_tts_fragment_documents_step_clamps() -> None:
    """Фрагмент должен документировать диапазоны clamp
    (pitch [0.5, 2.0], volume dB-диапазон) — чтобы LLM не обещал
    невозможное (например, pitch=3.0)."""
    content = _read(_SKILL_FRAGMENT)
    assert "0.5" in content and "2.0" in content, (
        "prompts/skills/voice-tts.txt должен документировать clamp-диапазон pitch "
        "[0.5, 2.0]"
    )


def test_voice_tts_fragment_warns_against_inline_prosody_pitch() -> None:
    """Фрагмент обязан предупреждать, что <prosody pitch=...> внутри
    speak_text — НЕ workaround для смены высоты голоса.

    Это самое коварное место t_85b38d89: тег молча вырезается, юзер не
    слышит разницы, а модель считает просьбу выполненной. Проверяем и
    упоминание тега, и явное указание, что он не работает — на русском,
    как и сам фрагмент."""
    content = _read(_SKILL_FRAGMENT).lower()
    assert "prosody" in content, (
        "prompts/skills/voice-tts.txt должен упоминать prosody-тег "
        "(предупреждение о неработающем workaround)"
    )
    assert any(
        marker in content
        for marker in ("игнорир", "вырезает", "ignored", "dropped")
    ), (
        "prompts/skills/voice-tts.txt должен явно говорить, что prosody pitch "
        "не срабатывает на minimax/yandex"
    )


# ── schema parity: каталог инструментов ──────────────────────────────
#
# Раньше здесь проверялась сигнатура ``set_pitch`` в
# ``skills/status_skill.py`` — обёртке удалённого Compositor'а. Обёртка
# была ВТОРЫМ объявлением контракта и разошлась с ``execute()``: она
# обещала LLM ``set_pitch(pitch: float)`` при реальном
# ``set_pitch(action: str)``. Обёртки больше нет; источник правды —
# каталог, генерируемый из самих классов инструментов.


def test_set_pitch_advertises_an_action_enum() -> None:
    """``set_pitch`` принимает строку-действие, а не число."""
    from rob_box_core.tool_catalog import get_tool

    schema = dict(get_tool("set_pitch").parameters)
    properties = schema.get("properties", {})
    assert "action" in properties, (
        "set_pitch должен принимать action; параметр pitch:float — это "
        "регрессия класса t_85b38d89"
    )
    assert properties["action"].get("type") == "string"


def test_voice_tts_fragment_mentions_pitch_actions() -> None:
    """Фрагмент скилла обязан описывать реальные значения action."""
    text = _SKILL_FRAGMENT.read_text(encoding="utf-8").lower()
    for action in ("higher", "lower", "normal"):
        assert action in text, f"voice-tts.txt не описывает action={action!r}"
