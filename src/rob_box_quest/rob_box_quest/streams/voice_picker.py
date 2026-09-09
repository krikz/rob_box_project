"""voice_picker — pure-функция валидации голоса по кэшу ``/voice/tts/voices``.

Источник истины для списка голосов — latched-топик ``/voice/tts/voices``,
который ``tts_node`` публикует на старте и при каждой смене провайдера.
``QuestBridge`` кэширует его в ``_voices_cache`` (``list[dict]``, где
каждый dict — это ``voice_info_for(provider, voice_id)``: ``voice_id``,
``display_name``, ``language``, ``gender``, ``presets``, ``provider``).

Этот модуль выделяет ЧИСТУЮ часть валидации (``pick_voice``), чтобы её
можно было тестировать без ``audio_common_msgs`` и без импорта
``QuestBridge`` (который тянет rclpy/geometry_msgs и skip'ается на
dev-env без Docker — см. ``test_quest_bridge.py:_make_voice_bridge``).

Почему ВАЛИДАЦИЯ ПО КЭШУ, а не по компайл-тайм реестру
(``rob_box_voice.tts_voice_registry``):

* До фикса issue #2138 ``QuestBridge.set_voice`` ходил через
  ``voices_for(provider)`` из ``tts_voice_registry``. В образе
  ``rob-box-quest`` этого пакета нет (конвенция «пакет должен
  оставаться импортируемым без rob_box_voice» — см. ``mcp_tools``).
  Fallback импорта возвращал ``[]``, и ``set_voice`` ВСЕГДА отдавал
  ``tts_unreachable`` при живом TTS. UI Quest показывал «TTS
  недоступен» хотя спикер-выбор реально работал в tts_node.
* tts_node — единственный источник правды о том, какие голоса
  реально есть у провайдера в текущий момент (провайдер может
  сменить каталог в любой момент, и tts_node это отразит, а
  компайл-тайм реестр — нет).

Capability-honest: пустой кэш = честный ``tts_unreachable``, не
молчаливый «ОК» (см. ADR-0018 / task body §1 требования «Источник
истины — то, что провайдер реально отдал»).
"""

from __future__ import annotations

from typing import NamedTuple

# Контракт причин nack — стабильный wire-формат, читается ws_handler
# и клиентом Quest (см. docs/architecture/meta-quest-api.md §5.2,
# test_ws_server_voice.py).
VOICE_UNREACHABLE_REASON: str = "tts_unreachable"
VOICE_UNAVAILABLE_REASON: str = "voice_unavailable"


class VoiceChoice(NamedTuple):
    """Результат валидации voice_id по кэшу.

    * ``ok=True, reason=None, available=None`` — голос подтверждён.
    * ``ok=False, reason=VOICE_UNREACHABLE_REASON, available=None`` —
      кэш пуст/None (tts_node ещё не прислал latched-топик, или упал,
      или кэш протух по TTL). Это capability-honest nack.
    * ``ok=False, reason=VOICE_UNAVAILABLE_REASON, available=list[str]`` —
      кэш есть, но голоса в нём нет. ``available`` — список voice_id из
      кэша (для UI-подсказки).
    """

    ok: bool
    reason: str | None
    available: list[str] | None


def pick_voice(
    cache_voices: list[dict] | None,
    *,
    voice_id: str,
) -> VoiceChoice:
    """Валидировать ``voice_id`` по кэшу ``/voice/tts/voices``.

    Args:
        cache_voices: список словарей от ``voice_info_for`` (поля
            ``voice_id``, ``display_name`` и т.д.). ``None`` или ``[]``
            трактуются как «кэш пуст» → честный
            ``tts_unreachable``. Этот модуль НЕ проверяет TTL —
            за свежестью кэша следит вызывающий (см.
            ``QuestBridge.list_voices_snapshot``).
        voice_id: запрошенный клиентом voice_id.

    Returns:
        ``VoiceChoice`` с полями ``ok``, ``reason``, ``available``.

    Поведение (см. test_voice_picker.py):
        * ``cache_voices`` пуст/None → ``tts_unreachable``.
        * ``voice_id in cache_voices`` (точное равенство строк) →
            ``ok=True``.
        * ``voice_id not in cache_voices`` → ``voice_unavailable`` +
            ``available = [v["voice_id"] for v in cache_voices]``.
        * Дубликаты ``voice_id`` в кэше дедуплицируются в ``available``.
    """
    if not cache_voices:
        # Capability-honest: кэш пуст — не делаем вид, что голос валиден.
        return VoiceChoice(
            ok=False,
            reason=VOICE_UNREACHABLE_REASON,
            available=None,
        )

    available_ids = _voice_ids_dedup(cache_voices)

    if voice_id in available_ids:
        return VoiceChoice(ok=True, reason=None, available=None)

    return VoiceChoice(
        ok=False,
        reason=VOICE_UNAVAILABLE_REASON,
        available=available_ids,
    )


def _voice_ids_dedup(cache_voices: list[dict]) -> list[str]:
    """Достать voice_id'ы из кэша, дедуплицировать, сохранить порядок.

    Защита от двух случаев:
        1. dict без поля ``voice_id`` (мусор от кривого upstream) — пропускаем.
        2. Дубликаты ``voice_id`` в latched-payload (например, tts_node
           дважды прислал один и тот же голос с разными метаданными) —
           оставляем только первое вхождение, чтобы ``available`` не
           показывал один и тот же id дважды.
    """
    seen: set[str] = set()
    out: list[str] = []
    for entry in cache_voices:
        if not isinstance(entry, dict):
            continue
        vid = entry.get("voice_id")
        if not isinstance(vid, str) or not vid:
            continue
        if vid in seen:
            continue
        seen.add(vid)
        out.append(vid)
    return out


__all__ = [
    "VoiceChoice",
    "VOICE_UNREACHABLE_REASON",
    "VOICE_UNAVAILABLE_REASON",
    "pick_voice",
]
