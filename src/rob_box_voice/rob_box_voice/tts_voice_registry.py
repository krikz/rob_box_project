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
# * MiniMax T2A v2 — актуальные системные голоса из официального FAQ
#   https://platform.minimax.io/docs/faq/system-voice-id (20.08.2026).
#   Russian_* — текущий каталог русских голосов; male-qn-qingse /
#   female-shaonv — legacy-id, всё ещё работают на speech-02-hd, но в
#   актуальном списке отсутствуют.
# * Silero v5 — speaker id (aidar/baya/kseniya/xenia/eugene).
PROVIDER_VOICES: dict[str, list[str]] = {
    "yandex": [
        "anton", "alena", "filipp", "jane", "omazh", "zahar",
        "ermil", "madirus", "arina", "kostya", "rush",
    ],
    "minimax": [
        # текущие русские системные голоса (FAQ, 20.08.2026):
        "Russian_ReliableMan", "Russian_HandsomeChildhoodFriend",
        "Russian_AttractiveGuy", "Russian_Bad-temperedBoy",      # male
        "Russian_BrightHeroine", "Russian_AmbitiousWoman",
        "Russian_CrazyQueen", "Russian_PessimisticGirl",         # female
        # legacy (работают, но нет в актуальном списке):
        "male-qn-qingse", "female-shaonv",
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

# AV-27 / issue #1919 — метаданные голосов для wire payload
# (VoiceInfo в messages.ts:125-133). ``voice_id`` — ключ в PROVIDER_VOICES,
# всё остальное — обогащение для отображения в UI Quest voice-picker.
# Хранится ТАМ ЖЕ где PROVIDER_VOICES (один источник правды): при удалении
# voice_id из PROVIDER_VOICES запись станет висячей — ниже в
# ``voice_info_for`` есть явная проверка. Сознательно нет поля ``description``
# (UI берёт display_name, ``description`` остаётся зарезервирован под
# MCP-tool провайдерские метаданные в следующих карточках).
VOICE_METADATA: dict[str, "VoiceInfo"] = {
    # Yandex SpeechKit v3 (gRPC) — стандартные голоса API v3.
    "yandex:anton":           {"display_name": "Антон",  "language": "ru-RU", "gender": "male",   "presets": ["standard", "friendly"]},
    "yandex:alena":           {"display_name": "Алёна",  "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "yandex:filipp":          {"display_name": "Филипп", "language": "ru-RU", "gender": "male",   "presets": ["standard", "authoritative"]},
    "yandex:jane":            {"display_name": "Джейн",  "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "yandex:omazh":           {"display_name": "Омаж",   "language": "ru-RU", "gender": "neutral", "presets": ["standard"]},
    "yandex:zahar":           {"display_name": "Захар",  "language": "ru-RU", "gender": "male",   "presets": ["standard", "authoritative"]},
    "yandex:ermil":           {"display_name": "Ермил",  "language": "ru-RU", "gender": "male",   "presets": ["standard"]},
    "yandex:madirus":         {"display_name": "Мадирус","language": "ru-RU", "gender": "male",   "presets": ["standard", "whisper"]},
    "yandex:arina":           {"display_name": "Арина",  "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "yandex:kostya":          {"display_name": "Костя",  "language": "ru-RU", "gender": "male",   "presets": ["standard"]},
    "yandex:rush":            {"display_name": "Раш",    "language": "ru-RU", "gender": "neutral", "presets": ["standard"]},
    # MiniMax T2A v2 — каталог FAQ 20.08.2026 (male_qn/female_shaonv — legacy).
    "minimax:Russian_ReliableMan":         {"display_name": "Надёжный мужчина",     "language": "ru-RU", "gender": "male",   "presets": ["standard", "authoritative"]},
    "minimax:Russian_HandsomeChildhoodFriend":{"display_name": "Красивый друг детства", "language": "ru-RU", "gender": "male", "presets": ["standard", "friendly"]},
    "minimax:Russian_AttractiveGuy":       {"display_name": "Привлекательный парень", "language": "ru-RU", "gender": "male", "presets": ["standard", "friendly"]},
    "minimax:Russian_Bad-temperedBoy":     {"display_name": "Вспыльчивый парень",   "language": "ru-RU", "gender": "male",   "presets": ["authoritative", "standard"]},
    "minimax:Russian_BrightHeroine":       {"display_name": "Яркая героиня",         "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "minimax:Russian_AmbitiousWoman":      {"display_name": "Амбициозная женщина",   "language": "ru-RU", "gender": "female", "presets": ["standard", "authoritative"]},
    "minimax:Russian_CrazyQueen":          {"display_name": "Безумная королева",     "language": "ru-RU", "gender": "female", "presets": ["authoritative"]},
    "minimax:Russian_PessimisticGirl":     {"display_name": "Пессимистичная девушка","language": "ru-RU", "gender": "female", "presets": ["whisper", "standard"]},
    "minimax:male-qn-qingse":              {"display_name": "Qingse (legacy)",       "language": "zh-CN", "gender": "male",   "presets": ["standard"]},
    "minimax:female-shaonv":               {"display_name": "Shaonv (legacy)",       "language": "zh-CN", "gender": "female", "presets": ["standard"]},
    # Silero v5 — speaker id (aidar/baya/kseniya/xenia/eugene).
    "silero:aidar":   {"display_name": "Айдар (мужской)",     "language": "ru-RU", "gender": "male",   "presets": ["standard", "authoritative"]},
    "silero:baya":    {"display_name": "Бая (женский)",       "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "silero:kseniya": {"display_name": "Ксения",              "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "silero:xenia":   {"display_name": "Ксения (v5)",         "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "silero:eugene":  {"display_name": "Евгений (v5)",        "language": "ru-RU", "gender": "male",   "presets": ["standard", "authoritative"]},
}


def voices_for(provider: str) -> list[str]:
    """Список голосов провайдера (пустой список для неизвестного)."""
    return list(PROVIDER_VOICES.get(provider, []))


def default_voice_for(provider: str) -> str:
    """Дефолтный голос провайдера; для неизвестного — пустая строка."""
    return DEFAULT_VOICES.get(provider, "")


def voice_info_for(provider: str, voice_id: str) -> "VoiceInfo | None":
    """Wire-payload ``VoiceInfo`` (meta-quest-api.md §4.1) для одного голоса.

    Возвращает ``None`` если voice_id не в каталоге провайдера. Используется
    и для UI-валидации set_voice (``voices_for(provider)`` для unknown
    даст nack), и для публикации /voice/tts/voices из tts_node.

    Условие «voice_id знает провайдер» двойное: id обязан быть и в
    ``PROVIDER_VOICES[provider]`` (SoT списка голосов), и в ``VOICE_METADATA``
    (обогащение). Если id есть в PROVIDER_VOICES но нет в метаданных —
    fallback в display_name=voice_id, language="ru-RU", gender="neutral",
    presets=[] (UI не сломается, но лучше так не оставлять — добавление
    записи в VOICE_METADATA лучше).
    """
    voices = voices_for(provider)
    if voice_id not in voices:
        return None
    meta = VOICE_METADATA.get(f"{provider}:{voice_id}")
    if meta is None:
        return {
            "voice_id": voice_id,
            "display_name": voice_id,
            "language": "ru-RU",
            "gender": "neutral",
            "presets": [],
        }
    out = {"voice_id": voice_id, **meta}
    return out


def voices_info_for(provider: str) -> list["VoiceInfo"]:
    """Полный список VoiceInfo для провайдера (для wire payload).

    Возвращает ``[]`` если провайдер не знает ни одного голоса — это
    ЧЕСТНЫЙ пустой список, который UI отрисует как «провайдер не отдаёт
    список голосов» (issue #1919 acceptance, design t_5b9d5d0c §52-87).
    Никаких хардкод-fallback'ов в вызывающем коде.
    """
    return [info for vid in voices_for(provider) if (info := voice_info_for(provider, vid)) is not None]


# Late-import типа для typing-аннотаций (Python 3.11+ позволяет from __future__
# import annotations, поэтому runtime-импорта VOICE_INFO_FIELDS не нужно —
# только type-checker'у).
VoiceInfo = dict  # noqa: E305 — зарезервировано под TypedDict в следующих карточках.


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


def effective_provider(
    provider_chain: list[str] | tuple[str, ...] | None,
    is_dead,
) -> str | None:
    """Первый «живой» провайдер в цепочке (issue #1229).

    Цепочка приоритетов TTS (minimax → yandex → silero) с кэшем
    «мёртвых» провайдеров. ``is_dead(provider) -> bool`` — колбэк,
    возвращающий True для провайдера в кэше «мёртвых» (квота/сеть).

    Args:
        provider_chain: упорядоченная цепочка провайдеров (может быть
            None/пустой — тогда возвращаем None).
        is_dead: callable(provider) -> bool.

    Returns:
        Первый провайдер цепочки, который НЕ мёртв. Если все мёртвы —
        последний провайдер цепочки (Silero всегда последний и офлайн,
        это осознанный «аварийный» выбор). Для пустой цепочки — None.
    """
    if not provider_chain:
        return None
    for provider in provider_chain:
        if not is_dead(provider):
            return provider
    return provider_chain[-1]


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
