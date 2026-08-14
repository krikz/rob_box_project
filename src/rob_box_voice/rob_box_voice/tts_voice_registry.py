#!/usr/bin/env python3
"""
tts_voice_registry.py — единый реестр голосов TTS-провайдеров (issue #1219).

Реестр отвечает на три вопроса:

1. Какие голоса доступны у провайдера?  (``voices_for``)
2. Какой голос по умолчанию?            (``default_voice_for``)
3. Какой голос реально использовать?    (``resolve_voice``)

Чистый Python без ROS-зависимостей: модуль импортируется и
``rob_box_voice`` (tts_node, dialogue_node), и ``rob_box_mcp_tools``
(SpeakTextTool, SetVoiceTool) — без тяжёлых зависимостей (rclpy, torch).

Дизайн-решения (docs/design/LLM_VOICE_SELECTION_PROPOSAL.md, Q1-Q11):

* LLM выбирает голос **по имени** (``voice="alena"``) — характеристики
  (``female_calm``) остаются fallback'ом на будущее (Q1).
* Список голосов — **только активного провайдера** (Q4); при фолбеке
  список меняется на голоса нового провайдера.
* Неизвестный/недоступный голос → **дефолтный голос провайдера** + в
  результате ``voice_used`` сообщает фактический (Q6).

Дефолтные голоса зеркалят ``src/rob_box_voice/config/tts_node.yaml``:
yandex → anton, minimax → male-qn-qingse, silero → aidar.
"""

from __future__ import annotations

# ── Реестр голосов по провайдерам ────────────────────────────────────────────
#
# Источники:
# * Yandex SpeechKit v3 (gRPC) — стандартные голоса API v3.
# * MiniMax T2A v2 — voice_id из документации MiniMax (api.minimax.io).
# * Silero v5 — speaker id (aidar/baya/kseniya/xenia/eugene).
PROVIDER_VOICES: dict[str, list[str]] = {
    "yandex": [
        "anton", "alena", "filipp", "jane", "omazh", "zahar",
        "ermil", "madirus", "arina", "kostya", "rush",
    ],
    "minimax": [
        "male-qn-qingse", "female-shaonv", "male-chengshu",
        "female-tianmei", "male-jingxi", "female-yejian",
        "male-qingse", "female-qingse", "male-zhiyu", "female-zhiyu",
    ],
    "silero": [
        "aidar", "baya", "kseniya", "xenia", "eugene",
    ],
}

# Дефолтный голос провайдера (Q6/Q8). Значения совпадают с
# src/rob_box_voice/config/tts_node.yaml (yandex_voice / minimax_voice /
# silero_speaker), чтобы LLM-контекст и фактический синтез не расходились.
DEFAULT_VOICES: dict[str, str] = {
    "yandex": "anton",
    "minimax": "male-qn-qingse",
    "silero": "aidar",
}


def voices_for(provider: str) -> list[str]:
    """Список голосов провайдера (пустой список для неизвестного)."""
    return list(PROVIDER_VOICES.get(provider, []))


def default_voice_for(provider: str) -> str:
    """Дефолтный голос провайдера; для неизвестного — пустая строка."""
    return DEFAULT_VOICES.get(provider, "")


def resolve_voice(provider: str, requested: str | None) -> tuple[str, bool]:
    """Определить фактический голос для провайдера.

    Args:
        provider: имя провайдера ("yandex" | "minimax" | "silero").
        requested: запрошенный голос (может быть None / пустой строкой).

    Returns:
        ``(voice_used, fell_back)``:
        * ``voice_used`` — голос, который реально будет использован;
        * ``fell_back`` — True, если запрошенный голос неизвестен/недоступен
          у провайдера и пришлось откатиться на дефолт (Q6).
    """
    voices = voices_for(provider)
    if requested and requested in voices:
        return requested, False
    return default_voice_for(provider), True


def format_tts_context(
    provider: str,
    default_voice: str | None = None,
    current_voice: str | None = None,
) -> str:
    """Строка LLM-контекста (Q8).

    Формат::

        [TTS] provider: yandex | default_voice: anton | current_voice: zahar | voices: anton, alena, jane, ...

    ``current_voice`` — установленный через set_voice голос (если был);
    None → показываем дефолтный (current == default).
    """
    voices = voices_for(provider)
    default = default_voice or default_voice_for(provider)
    current = current_voice or default
    voices_str = ", ".join(voices) if voices else "-"
    return (
        f"[TTS] provider: {provider} | "
        f"default_voice: {default} | "
        f"current_voice: {current} | "
        f"voices: {voices_str}"
    )
