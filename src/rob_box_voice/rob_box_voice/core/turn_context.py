"""Звучащий TTS хода: провайдер и голос — факт от tts_node важнее параметра (ADR-0150 §7.3), реестр имён голосов.

Данные из владельца (параметр/событие TTS), без правил для модели.
"""

from __future__ import annotations

from typing import Any, List, Tuple

from rob_box_dialog_host.tts_voice_registry import ROBOT_DEFAULT_PROVIDER, default_voice_for, known_provider, named_voices_for


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
    «включи голос <имя>» (``grammar._voice``, #4086). Читатель — ``dialog_node``: без неё
    v2 звал ``TurnEngine`` без ``voices=``, реестр был пуст, и «говори голосом Весельчак» уходил модели."""
    return named_voices_for(tts_state(node)[0])


__all__ = ["tts_state", "tts_voices"]
