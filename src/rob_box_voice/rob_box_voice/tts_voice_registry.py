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
* Голос называют id или ИМЕНЕМ: русское ``display_name`` («Весельчак»), хвост id («Jovialman»), запись из
  ``VOICE_ALIASES`` («джовиалмен»). Имя в id переводит код (``voice_id_by_name``, #4086), а не модель.

Дефолтные голоса зеркалят ``src/rob_box_voice/config/tts_node.yaml``:
yandex → anton, minimax → male-qn-qingse, silero → aidar.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

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
        # Не-русские голоса SpeechKit. У Yandex язык прибит к ГОЛОСУ (в
        # отличие от MiniMax, где язык задаёт language_boost при любом
        # голосе), поэтому «сказать по-немецки» = «взять lea». Каталог
        # aistudio.yandex.ru/docs/en/speechkit/tts/voices: кроме ru есть
        # en-US (john), de-DE (lea), he-IL, kk, uz. Французского,
        # китайского и хинди у Yandex нет вовсе.
        "john", "lea",
    ],
    "minimax": [
        # текущие русские системные голоса (FAQ, 20.08.2026):
        "Russian_ReliableMan", "Russian_HandsomeChildhoodFriend",
        "Russian_AttractiveGuy", "Russian_Bad-temperedBoy",      # male
        "Russian_BrightHeroine", "Russian_AmbitiousWoman",
        "Russian_CrazyQueen", "Russian_PessimisticGirl",         # female
        # голоса каста марафона (#3992): ими харнесс e2e озвучивает людей (``map_tts_voice`` в
        # ``.github/workflows/scripts/e2e_voice_lib.sh``). В каталоге робота они с #4086: «говори голосом Весельчак»
        # без записи здесь честно отвечало «такого голоса нет». Нерусские: русский текст читают с акцентом.
        "English_Jovialman", "English_WiseScholar", "Chinese (Mandarin)_Gentle_Senior",   # male, male, female
        "English_Whispering_girl", "Portuguese_StrictBoss",                               # female, female
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

#: Голос робота «по умолчанию» (#3803): провайдер, на котором он говорит «обычным голосом». Одна запись; голос
#: берётся из ``DEFAULT_VOICES``. «Верни свой обычный голос» разворачивает код (``robot_default_voice``), не LLM.
ROBOT_DEFAULT_PROVIDER = "minimax"


def robot_default_voice() -> tuple[str, str]:
    """``(провайдер, голос)`` обычного голоса робота; не дефолт ТЕКУЩЕГО провайдера."""
    return ROBOT_DEFAULT_PROVIDER, DEFAULT_VOICES[ROBOT_DEFAULT_PROVIDER]


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
    "yandex:john":            {"display_name": "John (English)", "language": "en-US", "gender": "male",   "presets": ["standard"]},
    "yandex:lea":             {"display_name": "Lea (Deutsch)",  "language": "de-DE", "gender": "female", "presets": ["standard"]},
    # MiniMax T2A v2 — каталог FAQ 20.08.2026 (male_qn/female_shaonv — legacy).
    "minimax:Russian_ReliableMan":         {"display_name": "Надёжный мужчина",     "language": "ru-RU", "gender": "male",   "presets": ["standard", "authoritative"]},
    "minimax:Russian_HandsomeChildhoodFriend":{"display_name": "Красивый друг детства", "language": "ru-RU", "gender": "male", "presets": ["standard", "friendly"]},
    "minimax:Russian_AttractiveGuy":       {"display_name": "Привлекательный парень", "language": "ru-RU", "gender": "male", "presets": ["standard", "friendly"]},
    "minimax:Russian_Bad-temperedBoy":     {"display_name": "Вспыльчивый парень",   "language": "ru-RU", "gender": "male",   "presets": ["authoritative", "standard"]},
    "minimax:Russian_BrightHeroine":       {"display_name": "Яркая героиня",         "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "minimax:Russian_AmbitiousWoman":      {"display_name": "Амбициозная женщина",   "language": "ru-RU", "gender": "female", "presets": ["standard", "authoritative"]},
    "minimax:Russian_CrazyQueen":          {"display_name": "Безумная королева",     "language": "ru-RU", "gender": "female", "presets": ["authoritative"]},
    "minimax:Russian_PessimisticGirl":     {"display_name": "Пессимистичная девушка","language": "ru-RU", "gender": "female", "presets": ["whisper", "standard"]},
    # Голоса каста марафона (#3992, #4086): язык - НАТИВНЫЙ язык голоса (русский он читает с акцентом), поэтому в роли
    # сказки (``role_voices_for``, #3998) они не идут; пол - по замеру живого канала (docs/dialog/role_voices_biometrics_2026-10-26.md).
    "minimax:English_Jovialman":           {"display_name": "Весельчак",             "language": "en", "gender": "male",   "presets": ["standard", "friendly"]},
    "minimax:English_WiseScholar":         {"display_name": "Мудрый учёный",         "language": "en", "gender": "male",   "presets": ["standard", "authoritative"]},
    "minimax:Chinese (Mandarin)_Gentle_Senior": {"display_name": "Добрая бабушка",   "language": "zh-CN", "gender": "female", "presets": ["standard", "friendly"]},
    "minimax:English_Whispering_girl":     {"display_name": "Шёпот",                 "language": "en", "gender": "female", "presets": ["whisper", "standard"]},
    "minimax:Portuguese_StrictBoss":       {"display_name": "Строгая начальница",    "language": "pt", "gender": "female", "presets": ["authoritative", "standard"]},
    "minimax:male-qn-qingse":              {"display_name": "Qingse (legacy)",       "language": "zh-CN", "gender": "male",   "presets": ["standard"]},
    "minimax:female-shaonv":               {"display_name": "Shaonv (legacy)",       "language": "zh-CN", "gender": "female", "presets": ["standard"]},
    # Silero v5 — speaker id (aidar/baya/kseniya/xenia/eugene).
    "silero:aidar":   {"display_name": "Айдар (мужской)",     "language": "ru-RU", "gender": "male",   "presets": ["standard", "authoritative"]},
    "silero:baya":    {"display_name": "Бая (женский)",       "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "silero:kseniya": {"display_name": "Ксения",              "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "silero:xenia":   {"display_name": "Ксения (v5)",         "language": "ru-RU", "gender": "female", "presets": ["standard", "friendly"]},
    "silero:eugene":  {"display_name": "Евгений (v5)",        "language": "ru-RU", "gender": "male",   "presets": ["standard", "authoritative"]},
}

#: Другие имена голосов, которыми их называют люди и STT: кириллическая запись латинского имени. Остальные имена код берёт
#: сам - сам id, его хвост после первого «_» («Jovialman») и ``display_name``. Ключ - как у ``VOICE_METADATA``.
VOICE_ALIASES: dict[str, tuple[str, ...]] = {
    "minimax:English_Jovialman": ("Джовиалмен", "Джовиалмэн"),
    "minimax:English_WiseScholar": ("Вайз Скулар",),
    "minimax:Chinese (Mandarin)_Gentle_Senior": ("Джентл Сениор",),
    "minimax:English_Whispering_girl": ("Виспериг гёрл",),
    "minimax:Portuguese_StrictBoss": ("Стрикт Босс",),
}

_NAME_NOISE = re.compile(r"[\W_]+")


def _name_key(text: object) -> str:
    """Ключ сравнения имён голосов: без регистра, ``ё`` = ``е``, без пробелов, скобок, дефисов и подчёркиваний."""
    return _NAME_NOISE.sub("", str(text or "").casefold().replace("ё", "е"))


def _names_of(provider: str, voice_id: str) -> set[str]:
    """Все ключи имён одного голоса: id, хвост id, ``display_name``, ``VOICE_ALIASES``."""
    key = f"{provider}:{voice_id}"
    meta = VOICE_METADATA.get(key) or {}
    names = (voice_id, voice_id.split("_", 1)[-1], meta.get("display_name", ""), *VOICE_ALIASES.get(key, ()))
    return {k for k in map(_name_key, names) if k}


def _build_name_index() -> dict[str, dict[str, set[str]]]:
    index: dict[str, dict[str, set[str]]] = {}
    for provider, ids in PROVIDER_VOICES.items():
        by_name = index.setdefault(provider, {})
        for voice_id in ids:
            for name in _names_of(provider, voice_id):
                by_name.setdefault(name, set()).add(voice_id)
    return index


#: ``провайдер -> ключ имени -> id голосов с таким именем``; строится кодом из каталога, отдельной таблицы имён нет.
_NAME_INDEX: dict[str, dict[str, set[str]]] = _build_name_index()


def voice_id_by_name(provider: str, name: object) -> str:
    """id голоса каталога провайдера по id или по имени («Весельчак», «Jovialman», «джовиалмен»); не нашёлся или имя
    у двух голосов - ``""``. Угадывать не пытаемся: решение «этого голоса нет» - за кодом, не за моделью (#4086)."""
    ids = _NAME_INDEX.get(provider, {}).get(_name_key(name), set())
    return next(iter(ids)) if len(ids) == 1 else ""


def language_of(provider: str, voice_id: str) -> str:
    """Короткий код языка голоса ("ru", "en", "de"); "" если голос неизвестен.

    В метаданных язык лежит в BCP-47 ("ru-RU"), а ROS-параметр
    ``voice_output_language`` и wire-поле ``language`` — двухбуквенные
    (meta-quest-api.md §P7). Режем до первого дефиса.
    """
    meta = VOICE_METADATA.get(f"{provider}:{voice_id}")
    if not meta:
        return ""
    return str(meta.get("language") or "").split("-")[0].lower()


def languages_for(provider: str) -> set[str]:
    """Какие языки провайдер реально умеет — ПО КАТАЛОГУ ГОЛОСОВ.

    Источник правды один — ``VOICE_METADATA``: отдельной таблицы
    «провайдер → языки» нет и быть не должно, она бы разъехалась с
    каталогом ровно так же, как разъехались whitelist'ы AV-28.

    Осторожно: для MiniMax этот ответ НЕ полон. Там язык задаёт
    ``language_boost`` при любом голосе, поэтому MiniMax умеет всё из
    ``minimax_tts._LANGUAGE_ALIASES``, а не только языки своих голосов
    (в каталоге у него ru и zh). Вызывающий обязан учесть это отдельно —
    см. ``speak_helpers.unsupported_language_notice``.
    """
    langs = {language_of(provider, voice_id) for voice_id in voices_for(provider)}
    langs.discard("")
    return langs


def voice_for_language(provider: str, language: str) -> str:
    """Голос провайдера, говорящий на ``language``; "" если такого нет.

    Нужен там, где язык прибит к голосу (Yandex): просить у ``anton``
    немецкий бесполезно — надо брать ``lea``. Порядок обхода —
    ``PROVIDER_VOICES``, то есть первый подходящий голос каталога.
    """
    lang = str(language or "").split("-")[0].lower()
    if not lang:
        return ""
    default = default_voice_for(provider)
    if default and language_of(provider, default) == lang:
        return default
    for voice_id in voices_for(provider):
        if language_of(provider, voice_id) == lang:
            return voice_id
    return ""


def voices_for(provider: str) -> list[str]:
    """Список голосов провайдера (пустой список для неизвестного)."""
    return list(PROVIDER_VOICES.get(provider, []))


def named_voices_for(provider: str) -> list[tuple[str, ...]]:
    """``(voice_id, display_name, *алиасы)`` голосов провайдера - то, по чему Tier-1 «включи голос <имя>» находит голос
    в реестре (``rob_box_dialog.grammar``, #4086). ``display_name`` - русское название из ``VOICE_METADATA`` (нет записи
    - сам id); алиасы - записи ``VOICE_ALIASES`` («Джовиалмен», «Джовиалмэн»): кириллическая запись латинского имени,
    как её слышит STT, тоже имя голоса (ревью PR #4114: «говори голосом джовиалмен»). Название и каждый алиас -
    отдельные элементы кортежа, не склейка: ``rob_box_dialog.turn.command_form`` ищет в реплике каждое имя целым
    словом, а Tier-1 читает алиасы вторым кругом, после названия и хвоста id (``grammar._voice``). Хвост id
    («Jovialman») грамматика берёт сама. Одно знание об именах - ``VOICE_ALIASES`` (его же читает
    ``voice_id_by_name``); для v1 (``tier1_host``) и v2 (``dialog_node``) список один; провайдер неизвестен - пустой
    список."""
    out: list[tuple[str, ...]] = []
    for vid in voices_for(provider):
        key = f"{provider}:{vid}"
        shown = (VOICE_METADATA.get(key) or {}).get("display_name", vid)
        out.append((vid, shown, *VOICE_ALIASES.get(key, ())))
    return out


def known_provider(name: object) -> str:
    """Провайдер синтеза из каталога (``yandex``/``minimax``/``silero``); не провайдер - ``""``.

    Одно правило для всех, кто читает ``/voice/tts/provider_state``: метка вроде ``prefetch`` (звучание готового
    пред-синтеза) или мусор не становится «фактическим провайдером», и каталог голосов от неё не пустеет (#4024).
    """
    key = str(name or "").strip().lower()
    return key if key in PROVIDER_VOICES else ""


#: Язык голосов, которыми звучат роли выступления: реплики русские, чужой язык у голоса - чужой акцент (#3998).
ROLE_VOICE_LANGUAGE = "ru"


def role_voices_for(provider: str, narrator: str = "") -> list[str]:
    """Голоса для РОЛЕЙ выступления (сказка, анекдот, басня): русские голоса каталога звучащего провайдера, кроме
    голоса рассказчика.

    Язык - ``VOICE_METADATA.language``: legacy-китайские ``male-qn-qingse``/``female-shaonv`` MiniMax и
    английский/немецкий ``john``/``lea`` Yandex в роли не попадают (#3998: сказка звучала китайским голосом на русском
    тексте). Тем же фильтром в роли не идут голоса каста марафона (``English_Jovialman`` и др., #3992, #4086): ими
    харнесс e2e озвучивает людей, их язык в каталоге нативный (en/zh/pt), и герой сказки голосом Саши был бы «Сашей» для
    опознания по голосу (#4084). Отдельного списка каста нет - один источник, язык голоса; что каждый голос харнесса
    нерусский, держит ``test_issue_4084_role_voices.py``. ``narrator`` - текущий голос робота, им звучит рассказчик:
    роль не должна звучать так же. Порядок - каталога: роли голос даёт код (``rob_box_dialog.cast``), первый свободный
    нужного пола. ``voices_for`` не меняется: ``set_voice`` и обзоры голосов видят весь каталог.
    """
    return [v for v in voices_for(provider) if language_of(provider, v) == ROLE_VOICE_LANGUAGE and v != narrator]


def gender_of(provider: str, voice_id: str) -> str:
    """Пол голоса по каталогу (``male``/``female``/``neutral``); голос неизвестен - ``""``."""
    return str((VOICE_METADATA.get(f"{provider}:{voice_id}") or {}).get("gender") or "")


def default_voice_for(provider: str) -> str:
    """Дефолтный голос провайдера; для неизвестного — пустая строка."""
    return DEFAULT_VOICES.get(provider, "")


#: Как провайдера называют вслух (одна запись; латиницу TTS читает криво).
PROVIDER_LABELS: dict[str, str] = {"yandex": "Яндекс", "minimax": "MiniMax", "silero": "Silero"}

#: Сколько характерных голосов показывать у провайдера в обзоре (по одному на пол) и каких не показывать.
OVERVIEW_GENDERS: tuple[str, ...] = ("male", "female")


def providers_overview(active: str = "") -> list[dict]:
    """Обзор голосов по провайдерам для ответа «какими голосами умеешь» (#3883): провайдер, сколько у него голосов и по
    одному характерному русскому мужскому/женскому (без legacy). Единственный источник — ``PROVIDER_VOICES`` и
    ``VOICE_METADATA``; ``active`` помечает текущего провайдера."""
    out = []
    for provider, ids in PROVIDER_VOICES.items():
        samples = []
        for gender in OVERVIEW_GENDERS:
            for vid in ids:
                meta = VOICE_METADATA.get(f"{provider}:{vid}") or {}
                if (meta.get("gender") == gender and meta.get("language") == "ru-RU"
                        and "legacy" not in str(meta.get("display_name", ""))):
                    samples.append({"voice": vid, "name": meta.get("display_name", vid), "gender": gender})
                    break
        out.append({"provider": provider, "label": PROVIDER_LABELS.get(provider, provider), "voices_count": len(ids), "active": provider == active,
                    "samples": samples})
    return out


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
        requested: запрошенный голос - id или имя каталога (может быть None / пустой строкой).

    Returns:
        ``(voice_used, fell_back)``:
        * ``voice_used`` — голос, который реально будет использован (id каталога, даже если просили именем);
        * ``fell_back`` — True, если запрошенный голос неизвестен/недоступен
          у провайдера и пришлось откатиться на дефолт (Q6).
    """
    voices = voices_for(provider)
    if requested and requested in voices:
        return requested, False
    named = voice_id_by_name(provider, requested)
    if named:
        return named, False
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


# ── Пресеты пения (ADR-0158 §2.3, решение В8) ───────────────────────────────
#
# Песня поётся ТЕКУЩИМ голосом робота: TTS читает текст песни с пресетом,
# файл уходит в Suno ``add-instrumental`` как вокал. Таблица ниже — единая,
# строится КОДОМ из ``PROVIDER_VOICES`` + ``VOICE_METADATA`` (пол/характер) и
# возможностей провайдера. Голоса, добавленные в каталог, получают пресет без
# правок в двух местах; голос без пола в метаданных и без явной правки —
# громкий отказ при импорте (а не молчаливое «m»).
#
# Что умеет каждый провайдер (остальное в пресете остаётся нейтральным):
# * minimax — speed/volume/pitch/emotion (T2A v2 voice_setting);
# * yandex  — speed, role (только у части голосов, ``_YANDEX_ROLES``), pitch в Гц;
# * silero  — только выбор голоса (speed/pitch/volume/emotion не поддерживаются).

#: MiniMax T2A v2 допускает именно эти значения emotion.
MINIMAX_EMOTIONS: frozenset[str] = frozenset(
    {"happy", "neutral", "sad", "angry", "fearful", "disgusted", "surprised"}
)

#: Роли Yandex SpeechKit v3 по голосам (каталог SpeechKit). Голоса вне
#: таблицы роли не принимают — им emotion=None.
_YANDEX_ROLES: dict[str, frozenset[str]] = {
    "anton": frozenset({"neutral", "good"}),
    "alena": frozenset({"neutral", "good"}),
    "jane": frozenset({"neutral", "good", "evil"}),
    "omazh": frozenset({"neutral", "evil"}),
    "zahar": frozenset({"neutral", "good"}),
    "ermil": frozenset({"neutral", "good"}),
}

#: Голоса с gender="neutral" в метаданных, но с известным полом тембра —
#: Suno ``vocal_gender`` принимает только m/f.
_VOCAL_GENDER_OVERRIDE: dict[str, str] = {
    "yandex:omazh": "f",
    "yandex:rush": "m",
}

#: Проверенное на слух (Шифу, 13.10): Russian_ReliableMan + angry + weight 0.8.
_DEFAULT_AUDIO_WEIGHT = 0.8


@dataclass(frozen=True)
class SingPreset:
    """Подача голоса при чтении текста песни + подсказки для Suno."""

    provider: str
    voice: str
    speed: float                 # множитель скорости TTS (1.0 = норма)
    volume: float                # MiniMax vol; у остальных не применяется
    pitch: int                   # MiniMax полутона / Yandex Гц; 0 = не менять
    emotion: str | None          # MiniMax emotion; Yandex role; Silero — None
    vocal_gender: str            # "m" | "f" — для Suno add-instrumental
    audio_weight: float          # насколько Suno держится за вокал
    tags_hint: str               # добавка к тегам стиля
    language: str                # "ru"/"en"/"de"/"zh" — язык голоса (текст песни должен совпасть)


class SingPresetUnknown(KeyError):
    """Для (provider, voice) нет пресета пения — честный отказ, не фолбек."""


def _vocal_gender(provider: str, voice: str, gender: str) -> str:
    if gender == "male":
        return "m"
    if gender == "female":
        return "f"
    override = _VOCAL_GENDER_OVERRIDE.get(f"{provider}:{voice}")
    if override is None:
        raise ValueError(f"нет пола для пресета пения {provider}:{voice} (gender={gender!r})")
    return override


def _minimax_emotion(presets: list[str]) -> str:
    # Эталон: авторитетный Russian_ReliableMan поёт с angry (одобрено 13.10).
    # Остальные — по тегам характера из метаданных.
    if "authoritative" in presets:
        return "angry"
    if "friendly" in presets:
        return "happy"
    if "whisper" in presets:
        return "sad"
    return "neutral"


def _yandex_role(voice: str, presets: list[str]) -> str | None:
    roles = _YANDEX_ROLES.get(voice)
    if not roles:
        return None
    return "good" if "good" in roles and "friendly" in presets else None


def _tags_hint(gender: str, presets: list[str]) -> str:
    if "authoritative" in presets:
        mood = "deep powerful" if gender == "m" else "strong"
    elif "friendly" in presets:
        mood = "warm bright"
    elif "whisper" in presets:
        mood = "soft intimate"
    else:
        mood = "clear"
    return f"{mood} {'male' if gender == 'm' else 'female'} vocal"


def _build_sing_preset(provider: str, voice: str) -> SingPreset:
    meta = VOICE_METADATA.get(f"{provider}:{voice}")
    if meta is None:
        raise ValueError(f"голос {provider}:{voice} есть в каталоге, но нет в VOICE_METADATA")
    presets = list(meta.get("presets") or [])
    gender = _vocal_gender(provider, voice, str(meta.get("gender") or ""))
    if provider == "minimax":
        emotion: str | None = _minimax_emotion(presets)
    elif provider == "yandex":
        emotion = _yandex_role(voice, presets)
    else:
        emotion = None
    return SingPreset(
        provider=provider,
        voice=voice,
        speed=1.0,
        volume=1.0,
        pitch=0,
        emotion=emotion,
        vocal_gender=gender,
        audio_weight=_DEFAULT_AUDIO_WEIGHT,
        tags_hint=_tags_hint(gender, presets),
        language=language_of(provider, voice),
    )


SING_PRESETS: dict[tuple[str, str], SingPreset] = {
    (provider, voice): _build_sing_preset(provider, voice)
    for provider, voices in PROVIDER_VOICES.items()
    for voice in voices
}


def sing_preset_for(provider: str, voice: str) -> SingPreset:
    """Пресет пения голоса; неизвестный голос — ``SingPresetUnknown``.

    Тихого отката на дефолтный голос нет: песня чужим голосом без фразы
    «пою другим голосом» была бы враньём (ADR-0158 §2.3).
    """
    preset = SING_PRESETS.get((provider, voice))
    if preset is None:
        raise SingPresetUnknown(f"нет пресета пения для голоса {provider}:{voice}")
    return preset
