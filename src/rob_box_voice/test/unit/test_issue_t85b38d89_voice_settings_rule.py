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
2. ``status_skill_prompt.txt`` документирует enum и step-control
   семантику; явно запрещает SSML pitch-атрибут как workaround.
3. ``master_prompt_compact.txt`` имеет новый RULE #VOICE-SETTINGS
   с provider-conditional cheat sheet (minimax/yandex/silero).

Этот тест фиксирует wording RULE #VOICE-SETTINGS как prompt-contract.
Если кто-то сократит правило или удалит cross-provider guidance —
тест сломается, что и нужно.

Refs:
* t_85b38d89 — kanban-карточка с этим fix-path
* src/rob_box_voice/rob_box_voice/skills/status_skill.py — код skill'а
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

STATUS_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "skills"
    / "status_skill_prompt.txt"
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


# ── status_skill_prompt.txt ───────────────────────────────────────────


def test_status_skill_prompt_documents_set_pitch_action_enum() -> None:
    """status_skill_prompt должен документировать set_pitch(action: enum)."""
    content = _read(STATUS_PROMPT)
    # set_pitch(action) signature
    assert re.search(r"set_pitch\(action\)", content), (
        "status_skill_prompt.txt потерял сигнатуру set_pitch(action)"
    )
    # enum values
    assert "higher" in content and "lower" in content and "normal" in content, (
        "status_skill_prompt.txt должен перечислять set_pitch enum: "
        "higher/lower/normal"
    )


def test_status_skill_prompt_documents_step_clamps() -> None:
    """status_skill_prompt должен документировать диапазоны clamp
    (pitch [0.5, 2.0], volume dB-диапазон) — чтобы LLM не обещал
    невозможное (например, pitch=3.0)."""
    content = _read(STATUS_PROMPT)
    assert "0.5" in content and "2.0" in content, (
        "status_skill_prompt.txt должен документировать clamp-диапазон pitch "
        "[0.5, 2.0]"
    )


def test_status_skill_prompt_warns_against_inline_prosody_pitch() -> None:
    """status_skill_prompt должен явно предупреждать skill-LLM, что
    <prosody pitch=...> внутри speak_text — не workaround для
    изменения голоса."""
    content = _read(STATUS_PROMPT)
    assert "prosody" in content.lower(), (
        "status_skill_prompt.txt должен упоминать prosody-тег "
        "(предупреждение о неработающем workaround)"
    )
    assert "ignored" in content.lower() or "dropped" in content.lower(), (
        "status_skill_prompt.txt должен явно говорить, что prosody pitch "
        "ignored/dropped на minimax/yandex"
    )


# ── status_skill.py code (schema parity) ─────────────────────────────


def test_status_skill_set_pitch_signature_matches_mcp_action_enum() -> None:
    """``status_skill.py`` объявляет ``set_pitch(action: str)`` —
    синхронно с MCP ``SetPitchTool`` (action ∈ {higher, lower, normal}).

    Если кто-то рефакторит skill обратно на float — тест сломается,
    что и нужно (это и был исходный баг t_85b38d89)."""
    skill_path = (
        Path(__file__).resolve().parents[2]
        / "rob_box_voice"
        / "skills"
        / "status_skill.py"
    )
    src = skill_path.read_text(encoding="utf-8")
    # Сигнатура set_pitch(action: str)
    m = re.search(
        r"async def set_pitch\s*\(([^)]*)\)",
        src,
    )
    assert m is not None, "status_skill.py не содержит set_pitch(...)"
    sig = m.group(1)
    assert "action" in sig, (
        f"status_skill.py set_pitch(...) должен принимать 'action', "
        f"получили: {sig!r} (это был исходный баг t_85b38d89 — "
        f"сигнатура была pitch: float, не совпадала с MCP SetPitchTool)"
    )
    # Не должно быть float как типа параметра
    assert ": float" not in sig, (
        f"status_skill.py set_pitch(...): float НЕ допустим — MCP принимает "
        f"action: enum, не число. Сигнатура: {sig!r}"
    )
