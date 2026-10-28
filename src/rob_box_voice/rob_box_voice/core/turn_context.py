"""Разделы снимка хода в старом пути v1 (ADR-0150 §7.3, PR-6 #3696): данные ноды → ``context.build_context``.

Каждый раздел — данные из владельца (параметр/событие TTS, ``/odom``, доска провайдеров, снимок плеера), без правил
для модели. Род речи робота (``speech_gender``) считает код по метаданным голоса (``tts_voice_registry``), а не
модель по имени голоса.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from rob_box_voice.core.provider_reporter import reporter_context
from rob_box_voice.tts_voice_registry import (
    ROBOT_DEFAULT_PROVIDER,
    default_voice_for,
    format_tts_context,
    known_provider,
    named_voices_for,
    voice_info_for,
)


def tts_state(node: Any) -> Tuple[str, str]:
    """``(провайдер, голос)``, которые реально звучат: факт от tts_node (#1229, после фолбека) важнее параметра.

    Провайдер всегда из каталога реестра (#4024): метка не из него (``prefetch``) факт не заменяет, берётся настроенный
    ``tts_provider``, а при его отсутствии - провайдер робота по умолчанию. Иначе каталог голосов пуст, и голоса
    сегментов выступления перестают сверяться."""
    provider = known_provider(getattr(node, "_actual_tts_provider", None))
    if not provider:
        try:
            provider = known_provider(node.get_parameter("tts_provider").value)
        except Exception:  # noqa: BLE001 — тестовая нода без параметров
            provider = ""
    provider = provider or ROBOT_DEFAULT_PROVIDER
    return provider, getattr(node, "_actual_tts_voice", None) or default_voice_for(provider)


def tts_voices(node: Any) -> List[Tuple[str, ...]]:
    """``(voice_id, display_name, *алиасы)`` голосов звучащего провайдера (:func:`tts_state`) - реестр имён для Tier-1
    «включи голос <имя>» (``grammar._voice``, #4086). Одна функция для v1 (``tier1_host``) и v2 (``dialog_node``): v2
    без неё звал ``TurnEngine`` без ``voices=``, реестр был пуст, и «говори голосом Весельчак» уходил модели."""
    return named_voices_for(tts_state(node)[0])


def _tts_context(provider: str, voice: str) -> str:
    try:
        return format_tts_context(provider, current_voice=voice)
    except Exception:  # noqa: BLE001 — реестр сбойнул, ход не валим
        return f"[TTS] provider: {provider}"


def _position(node: Any) -> Any:
    pose = getattr(node, "_pose_snapshot", None)
    if not pose:
        return "unknown"
    return {"x": round(pose["x"], 2), "y": round(pose["y"], 2), "theta": round(pose["theta"], 3), "frame": "odom"}


def _performance(node: Any) -> Optional[Dict[str, Any]]:
    """Идущее выступление (хост ``perform_host`` у исполнителя тулов): спетое/звучащее дословно, первый не начатый."""
    performer = getattr(getattr(node, "_tool_executor", None), "performer", None)
    block = performer.context() if performer is not None else None
    return block if isinstance(block, dict) else None  # исполнитель без хоста выступления — раздела нет


def context_sections(node: Any, profile: str, recent: Optional[Any] = None,
                     speaker: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Разделы ``knowledge.CONTEXT_SECTIONS`` хода: ``profile`` — строки ``<user_profile>`` (шов собеседника v1),
    ``speaker`` — кто сказал реплику (``speaker_adapter.speaker_block``; у хода из Telegram — нет)."""
    provider, voice = tts_state(node)
    info = voice_info_for(provider, voice) or {}
    memory = getattr(node, "_music_state_mem", None)
    return {
        "user_profile": profile,
        "speaker": speaker,
        "tts_context": _tts_context(provider, voice),
        "speech_gender": info.get("gender"),
        "position": _position(node),
        "providers": reporter_context(getattr(node, "_provider_reporter", None)),
        "music_state": memory().section() if callable(memory) else None,
        "performance": _performance(node),  # PR-7c: что звучит из выступления (риск §16)
        "recent_unaddressed": recent,
    }


__all__ = ["context_sections", "tts_state", "tts_voices"]
