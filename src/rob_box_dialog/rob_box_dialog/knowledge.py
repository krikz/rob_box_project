"""Одна таблица знания диалога v2 (ADR-0150 §2.2, §2.1 п.10; ADR-0148).

Вейк-слова, слова тишины и её снятия, Tier-1 слова политики планировщика, слова и каналы громкости, классы
действий с таймаутами и прерываемостью, ключи фраз, словарь экспрессии (эмоции, жесты, earcon'ы, слоты сцены лица),
рефлексы по стадиям хода, календарь тем фар, кадры сцен сессии, бюджеты хода, матрица одновременности и пределы
команды LLM. Остальные модули — и старый путь v1 — только импортируют отсюда; копий нет. Из таблиц генерируются
фрагмент промпта (:func:`prompt_fragment`) и JSON-схема команды (:func:`command_schema`).

Сверка текста со словами (границы слов) — :mod:`rob_box_dialog.words`.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------------------------------------------
# Адресация (§4.2, решение В2: только по вейк-слову)
# ---------------------------------------------------------------------------------------------------------------

#: Вейк-слова личности. Порядок значим: ``strip_wake_word`` берёт первое совпадение альтернации, длинные и
#: специфичные варианты стоят раньше коротких («роб» не должен съесть «роб бокс»). Искажения STT добавляются по
#: логам e2e, не придумываются. В проде список читается из ``docker/sound/config/wake_words.yaml``
#: (:func:`load_wake_word_namespaces`), здесь — тот же список байт-в-байт для dev и тестов (держит
#: ``test/test_knowledge.py::test_wake_words_yaml_matches_code``).
DEFAULT_WAKE_WORDS = (
    "роб бокс",
    "роббокс",
    "робокос",
    "rob box",
    "робокс",
    # искажения «робокс» у vosk/yandex (e2e ww01): замены гласной
    "робэкс",
    "робекс",
    "робакс",
    "рабокс",
    "рубокс",
    "роблокс",
    "роберт",
    "роббос",
    "robbox",
    # «Робота, про меня что помнишь?» — vosk слышит винительный падеж; стоит раньше «робот»
    "робота",
    "робот",
    # «Робби, добрый вечер» — yandex перед звонкой согласной
    "робби",
    "робок",
    "рабок",
    "робак",
    "рыбок",
    "робик",
    "робо",
    "рома",
    "бот",
    "роб",
)
#: Вейк агента оператора: слышит только микрофон шлема (namespace привязан к источнику аудио).
DEFAULT_OPERATOR_WAKE_WORDS: Tuple[str, ...] = ("тарс", "tars")
WAKE_NAMESPACES: Tuple[str, ...] = ("personality", "operator")

# ---------------------------------------------------------------------------------------------------------------
# Tier-1 словарь (§4.2, §5.6.4, §6.3): ОСНОВЫ слов, матч по левой границе слова (``words.has_word``)
# ---------------------------------------------------------------------------------------------------------------

#: Тишина: «помолчи», «замолчите», «хватит». Матч по границе слова: «захватит» — не команда.
SILENCE_WORDS: Tuple[str, ...] = ("помолч", "замолч", "хватит")
#: Снятие тишины (в состоянии SILENCED): «говори», «включайся», «работай», «отвечай», «разговаривай».
UNSILENCE_WORDS: Tuple[str, ...] = ("говори", "включ", "работ", "отвеч", "разговар")


#: Политики BML при новой команде во время исполнения (§5.6.4) и границы, на которых они срабатывают.
POLICIES: Tuple[str, ...] = ("APPEND", "MERGE", "REPLACE", "REJECT")
BOUNDARIES: Tuple[str, ...] = ("now", "sentence", "segment", "bar", "phrase", "block")


@dataclass(frozen=True)
class PolicyWords:
    """Tier-1 переопределение политики планировщика (§5.6.4): закрытый список слов → политика и граница."""

    words: Tuple[str, ...]
    policy: str
    boundary: str


#: «стоп/хватит/замолчи» — заменить на границе предложения, «стой» — сразу, «потом» — после блока, «сейчас» —
#: на границе. Правила планировщика берут политику отсюда, а не из текста ответа LLM.
TIER1_POLICY: Mapping[str, PolicyWords] = MappingProxyType({
    "stop": PolicyWords(("хватит", "стоп", "замолчи", "отставить"), "REPLACE", "sentence"),
    "halt": PolicyWords(("стой",), "REPLACE", "now"),
    "later": PolicyWords(("потом", "после", "когда закончишь"), "APPEND", "block"),
    "now": PolicyWords(("сейчас", "прямо сейчас"), "REPLACE", "sentence"),
})

#: Громкость: (намерение, фразы) в порядке проверки — «максимальная громкость» раньше «громко». Фразы ищутся
#: подстрокой (как в v1, ``dialogue_helpers.detect_volume_intent``); канал выбирает код по снимку (§4.2).
VOLUME_WORDS: Tuple[Tuple[str, Tuple[str, ...]], ...] = (
    ("max", ("максимальная громкость", "максимум громк")),
    ("normal", ("нормальная громкость", "стандартная громкость", "обычная громкость")),
    ("max", ("говори громко",)),
    ("louder", ("громче", "громко")),
    ("quieter", ("тише", "потише")),
)
VOLUME_INTENTS: Tuple[str, ...] = ("max", "normal", "louder", "quieter")


@dataclass(frozen=True)
class VolumeChannel:
    """Канал громкости (§2.3): у каждого свой владелец, общей «громкости робота» нет."""

    owner: str
    tool: Optional[str]


#: Три канала у трёх владельцев. «Какой канал» решает код: играет музыка → music, звучит речь → tts.
VOLUME_CHANNELS: Mapping[str, VolumeChannel] = MappingProxyType({
    "tts": VolumeChannel(owner="tts_node", tool="set_volume"),
    "music": VolumeChannel(owner="PlayerOwner", tool="set_music_volume"),
    "sfx": VolumeChannel(owner="sound_node", tool=None),
})

# ---------------------------------------------------------------------------------------------------------------
# Классы действий (§5.1) и тулы по классам
# ---------------------------------------------------------------------------------------------------------------

#: Агенты на общем движке (§9): личность, оператор (ТАРС), телеграм.
AGENTS: Tuple[str, ...] = ("personality", "operator", "telegram")
#: Статусы результата исполнителя (§5.2, ``ActionResult.status``).
ACTION_STATUSES: Tuple[str, ...] = ("done", "rejected", "timeout", "cancelled", "pending")


@dataclass(frozen=True)
class ActionClass:
    """Класс действия: дедлайн события, прерываемость, источник события и ключи фраз по статусам.

    ``timeout_s=None`` — дедлайн задаёт не класс, а владелец (длина речи, план выступления). ``phrases`` —
    статус → ключ ``phrases/ru.yaml``; статуса нет в словаре — фразу о нём строит не шаблон (у ``query`` —
    LLM по данным). ``agents`` — кому класс доступен.
    """

    timeout_s: Optional[float]
    interruptible: bool
    event: str
    phrases: Mapping[str, str]
    agents: Tuple[str, ...] = AGENTS


def _keys(prefix: str, *statuses: str, **renamed: str) -> Mapping[str, str]:
    keys = {s: f"{prefix}.{s}" for s in statuses}
    keys.update({s: f"{prefix}.{name}" for s, name in renamed.items()})
    return MappingProxyType(keys)


ACTION_CLASSES: Mapping[str, ActionClass] = MappingProxyType({
    # данные → обратно LLM (≤ LLM_QUERY_ITERATIONS_MAX итераций), фраза — речь LLM; поиск 15 с, прерываемо
    "query": ActionClass(15.0, True, "tool_result", _keys("query", "rejected", "timeout")),
    # remember, set_volume, set_voice: событие владельца ≤ 3 с, мгновенно — не прерывается
    "action.local": ActionClass(3.0, False, "owner_event", _keys("local", "done", "rejected", "timeout")),
    # request_music, dj_set, play_sound: /voice/music/event started|rejected ≤ 6 с; старт не прерывается
    "action.media": ActionClass(6.0, False, "/voice/music/event", _keys("media", "done", "rejected", "timeout")),
    # Nav2 до 120 с, непрерываемо (кроме Tier-1 «стой»); «Еду к…» — обещание по pending
    "action.motion": ActionClass(
        120.0, False, "nav2_result",
        _keys("motion", "done", "rejected", "timeout", "cancelled", pending="started"),
    ),
    # событие speaker_id_node (registered); дедлайн в §5.1 не задан — взят как у action.local
    "action.identity": ActionClass(3.0, False, "speaker_id_node", _keys("identity", "done", "rejected", "timeout")),
    # say_aloud — только агенты оператора и телеграма; подтверждение — SpeechEvent finished
    "action.speak_aloud": ActionClass(
        None, True, "/voice/speech/event", _keys("speak_aloud", "done", "rejected", "cancelled"),
        agents=("operator", "telegram"),
    ),
    # выступление (§5.6.3): PlanEvent block_end; подложка started/rejected; прерывается на границе сегмента
    "action.perform": ActionClass(
        None, True, "/dialog/plan_event",
        _keys("perform", pending="backing_started", done="finished", rejected="no_backing"),
    ),
})

#: Тул каталога ``rob_box_core.tool_catalog`` → класс действия. Тул вне таблицы командой не исполняется
#: (валидатор отвергает); полноту против каталога держит тест.
TOOL_CLASSES: Mapping[str, str] = MappingProxyType({
    **{t: "query" for t in (
        "container_status", "estimate_tts_duration", "faq_search", "get_battery_level", "get_current_pose",
        "get_current_time", "get_music_state", "get_perception_context", "get_robot_status", "get_sound_info",
        "list_tts_voices", "list_waypoints", "listen_for_response", "lookup_melody", "memory_context",
        "memory_search", "read_logs", "ros2_node_status", "search_melody", "search_web", "show_metrics",
    )},
    **{t: "action.local" for t in (
        "add_music_material", "clear_waypoints", "delete_waypoint", "memory_save", "play_animation",
        "save_waypoint", "set_music_volume", "set_pitch", "set_speed", "set_tts_provider", "set_voice",
        "set_volume", "task_delta",
    )},
    **{t: "action.media" for t in (
        "dj_set", "generate_music", "music_library", "play_sound", "request_music", "stop_music",
    )},
    # долгие операции навигационного стека (Nav2/картирование): тот же дедлайн 120 с и непрерываемость
    **{t: "action.motion" for t in (
        "continue_mapping", "finish_mapping", "load_map", "move_direction", "navigate_to_coordinates",
        "navigate_to_waypoint", "optimize_map", "start_mapping", "stop_navigation",
    )},
    "register_speaker": "action.identity",
    "say": "action.speak_aloud",
})
#: Тулы-транспорт речи, а не действия: канал речи выбирает бейк-офф (§4.4), командой как действие не идут.
SPEECH_CHANNEL_TOOLS: Tuple[str, ...] = ("speak_text",)
#: Классы, действий которых в одной команде не больше одного (каждого): один трек и одна цель движения за ход.
SINGLE_ACTION_CLASSES: Tuple[str, ...] = ("action.media", "action.motion")
#: Ключи фраз, особые для тула (поверх ключа класса): «Запомнил: …» строится по событию saved владельца памяти.
TOOL_PHRASES: Mapping[str, Mapping[str, str]] = MappingProxyType({
    "memory_save": MappingProxyType({"done": "memory.saved"}),
})
#: Виды провайдеров (``ProviderState.kind``, ADR §8): ``music_gen`` - генерация музыки (KIE/Suno, ADR-0158).
PROVIDER_KINDS: Tuple[str, ...] = ("llm", "stt", "tts", "music_gen")
#: Классы здоровья провайдера (``ProviderState.health``). Единственная таблица: ``degraded`` - ещё пробуем (лимит
#: частоты), остальные - провайдер пропускаем; причина (quota/network/timeout/auth) - класс, а не подстрока ответа.
PROVIDER_HEALTH: Tuple[str, ...] = ("ok", "degraded", "quota", "network", "timeout", "auth", "down", "unknown")
#: Записи ``ProviderState`` старше этого срока без переиздания писателем считаются устаревшими (``unknown``).
PROVIDER_STALE_S: float = 10.0
#: Классы здоровья, при которых провайдер ещё годится в цепочку.
PROVIDER_USABLE_HEALTH: Tuple[str, ...] = ("ok", "degraded", "unknown")
#: Особые ключи каталога фраз (не по классу действия): их обязан содержать ``phrases/ru.yaml``.
PHRASE_KEYS: Tuple[str, ...] = (
    "understand.invalid", "understand.timeout",
    *(f"provider.exhausted.{reason}" for reason in ("quota", "network", "timeout", "auth", "error")),
    "provider.switched", "provider.no_fallback",
    "voice.applied", "voice.unavailable", "voice.rejected", "voice.timeout",
    *(f"perform.no_backing.{reason}" for reason in ("quota", "network", "timeout", "error")),
    "perform.ask_replace_music", "identity.ask_name", "identity.confirm",
)

# ---------------------------------------------------------------------------------------------------------------
# Экспрессия (§5.5): эмоция → сцена лица, жесты, earcon'ы, слоты сцены для LLM
# ---------------------------------------------------------------------------------------------------------------

#: Анимации с манифестом, которые умеет сегодняшний плеер (``rob_box_mcp_tools.animations.KNOWN_ANIMATIONS``),
#: плюс сцены лаборатории ``tools/led_face_lab`` (§5.5.2): ими именуются сцены лица в v1 и v2.
LAB_SCENES: Tuple[str, ...] = (
    "idle_stare", "idle_bored", "listening", "thinking_teeth", "thinking_orbit", "wink",
    "music_groove", "music_shades", "music_vibe", "new_year_eve",
)
#: Эмоция команды → сцена лица. Ключи — enum для LLM; ``neutral`` — покой.
EMOTIONS: Mapping[str, str] = MappingProxyType({
    "neutral": "idle",
    "happy": "happy",
    "sad": "sad",
    "angry": "angry",
    "surprised": "surprised",
    "thinking": "thinking",
    "victory": "victory",
    "sleepy": "sleep",
})
#: Короткие однократные жесты лица поверх эмоции.
GESTURES: Tuple[str, ...] = ("wink", "wakeup", "victory", "surprised")
#: Earcon'ы рефлексов: «услышал» (сегодняшний ранний клик ``boop → button_click``), «готово», «не вышло».
EARCONS: Tuple[str, ...] = ("heard", "done", "fail")

#: DSL сцены лица для LLM (§5.5.9): только enum-слоты, без цветов, форм и кода. ``speaker`` — взгляд по DOA,
#: ``talk`` — рот по огибающей TTS.
FACE_SCENE_SLOTS: Mapping[str, Tuple[str, ...]] = MappingProxyType({
    "eyes": ("open", "half", "squint", "wide", "closed", "shades", "x"),
    "gaze": ("speaker", "left", "right", "up", "down", "wander"),
    "blink": ("normal", "rapid", "none", "wink_left", "wink_right"),
    "mouth": ("grin", "frown", "flat", "o", "teeth", "eq", "talk"),
    "tint": ("warm", "cold", "red", "green", "blue", "purple", "rainbow"),
    "tempo": ("slow", "normal", "fast"),
    "rear": ("off", "markers", "hazard", "party", "new_year", "halloween", "heart", "rocket"),
})
FACE_SCENE_DEFAULT_S = 5.0
FACE_SCENE_MAX_S = 30.0

#: Режимы кольца LED ReSpeaker в рефлексах.
RING_MODES: Tuple[str, ...] = ("off", "breath", "doa_sector", "spin_slow", "flash", "flash_red", "beat_pulse")


@dataclass(frozen=True)
class Reflex:
    """Рефлекс кода на стадию хода/состояние робота (§5.5.3): сцены лица, кольцо, earcon — 0 вызовов LLM.

    ``face`` — сцены по очереди/на выбор (``idle_stare ↔ idle_bored``); пусто — лицо не трогается. ``after_ms`` —
    через сколько после события включается, ``deadline_ms`` — не позже чем. Минимальная длительность сцены и
    ``ttl`` подбираются в PR-3b по видео и добавляются сюда же.
    """

    face: Tuple[str, ...]
    ring: Optional[str] = None
    earcon: Optional[str] = None
    after_ms: Optional[int] = None
    deadline_ms: Optional[int] = None


#: Стадия хода / состояние → рефлекс. ``thinking``: зубы или орбита — выбор Шифу (В13), старт с зубов.
REFLEXES: Mapping[str, Reflex] = MappingProxyType({
    "idle": Reflex(("idle_stare", "idle_bored"), ring="breath", after_ms=3000),
    "addressed": Reflex(("listening",), ring="doa_sector", earcon="heard", deadline_ms=300),
    "thinking": Reflex(("thinking_teeth",), ring="spin_slow"),
    "executing": Reflex(("thinking_teeth",)),
    "speaking": Reflex(("talking",), ring="breath"),
    "done": Reflex(("happy", "wink"), ring="flash", earcon="done"),
    "failed": Reflex(("error",), ring="flash_red", earcon="fail"),
    "timeout": Reflex(("sad",), ring="flash_red", earcon="fail"),
    "silenced": Reflex(("sleep",), ring="off"),
    "music.playing": Reflex(("music_groove", "music_shades", "music_vibe"), ring="beat_pulse"),
    "not_addressed": Reflex(()),
    "battery.low": Reflex(("low_battery",)),
    "charging": Reflex(("charging",)),
})
#: Сцена лица на стадии ``executing`` по классу действия; класса нет — ``REFLEXES["executing"]``.
EXECUTING_FACE: Mapping[str, Tuple[str, ...]] = MappingProxyType({
    "action.motion": ("turn_left", "turn_right", "accelerating"),
    "action.media": ("music_groove", "music_shades", "music_vibe"),
})
#: Музыка (§5.5.5): верхняя граница BPM (не включая) → сцена; на слух Шифу.
MUSIC_SCENE_BPM: Tuple[Tuple[float, str], ...] = ((100.0, "music_vibe"), (125.0, "music_groove"),
                                                  (float("inf"), "music_shades"))
#: Стадии хода (``TurnEvent.stage``, §2.3), на которые есть рефлекс.
TURN_STAGES: Tuple[str, ...] = ("addressed", "thinking", "executing", "speaking", "done", "failed", "timeout")


@dataclass(frozen=True)
class CalendarTheme:
    """Тема задних фар на дату (§5.5.6); ``scene`` — сцена на всё лицо, ``scene_s`` — сколько."""

    month: int
    day: int
    rear: str
    scene: Optional[str] = None
    scene_s: Optional[float] = None


#: Календарь тем фар: имена тем — ``tools/led_face_lab/rear.py:THEMES``. Окно — один день; дни рождения робота
#: и Шифу добавляются, когда будут известны даты (в ADR их нет).
CALENDAR: Tuple[CalendarTheme, ...] = (
    CalendarTheme(1, 1, "new_year"),
    CalendarTheme(2, 14, "valentine"),
    CalendarTheme(2, 23, "defender_day"),
    CalendarTheme(3, 8, "womens_day"),
    CalendarTheme(4, 12, "cosmonautics"),
    CalendarTheme(10, 31, "halloween"),
    CalendarTheme(12, 31, "new_year", scene="new_year_eve", scene_s=60.0),
)

# ---------------------------------------------------------------------------------------------------------------
# Сессия и сцены (§7.1–7.2), бюджеты хода (§8)
# ---------------------------------------------------------------------------------------------------------------

#: Кадры стека сцен сессии.
SCENE_KINDS: Tuple[str, ...] = ("Silence", "Persona", "DJ", "OperatorHold", "Identity")
#: Кто кладёт кадр ``Silence``/``OperatorHold``: пользовательскую тишину снимает не оператор, и наоборот.
SCENE_SETTERS: Tuple[str, ...] = ("user", "operator", "code")
#: Пользовательская тишина «хватит/помолчи» — 10 минут (В3, Шифу 02.10).
SILENCE_TTL_S = 600
#: Кадр ``Identity(ask)`` — ответ ждётся 15 с (§7.2, §7.5).
IDENTITY_TTL_S = 15
#: Бюджет времени хода (В5): истёк — отмена ожиданий и ``understand.timeout``.
TURN_DEADLINE_S = 12
#: Query-цикл (§4.1): не больше двух итераций данных → LLM на ход.
LLM_QUERY_ITERATIONS_MAX = 2

# ---------------------------------------------------------------------------------------------------------------
# Матрица одновременности (§5.6.2) — данные; compose() читает её в PR-7b
# ---------------------------------------------------------------------------------------------------------------

#: Что приходит (строка матрицы): реплика, выступление, экспрессия, SFX, старт музыки, движение, «стой», query.
COMPOSITION_NEW: Tuple[str, ...] = ("speech", "perform", "expression", "sfx", "music", "motion", "halt", "query")
#: Что занято (столбец): голос-реплика, голос-выступление, лицо/кольцо, SFX, подложка, трек/сет, движение.
COMPOSITION_BUSY: Tuple[str, ...] = ("speech", "perform", "face", "sfx", "backing", "track", "motion")


@dataclass(frozen=True)
class Rule:
    """Политика BML для пары «новое × занятое» и граница, на которой она срабатывает; ``ask`` — ключ вопроса."""

    policy: str
    boundary: Optional[str] = None
    ask: Optional[str] = None


_M = Rule("MERGE")


def _row(**cells: Rule) -> Dict[str, Rule]:
    return {busy: cells.get(busy, _M) for busy in COMPOSITION_BUSY}


_ROWS: Dict[str, Dict[str, Rule]] = {
    # речь во время речи (О7): текущий сегмент дочитывается, на границе — обновлённый план
    "speech": _row(speech=Rule("MERGE", "segment"), perform=Rule("MERGE", "segment")),
    # смена подложки на такте; трек/сет — не вытеснять молча, а спросить (О9 (б))
    "perform": _row(speech=Rule("MERGE", "segment"), perform=Rule("MERGE", "segment"),
                    backing=Rule("REPLACE", "bar"), track=Rule("REJECT", ask="perform.ask_replace_music")),
    "expression": _row(),
    # SFX на SFX — после текущего, всегда с событием, не молча
    "sfx": _row(sfx=Rule("APPEND", "block")),
    # подложка на такте, речь выступления не рвётся; трек/сет — на фразе (ADR-0149)
    "music": _row(perform=Rule("REPLACE", "bar"), backing=Rule("REPLACE", "bar"), track=Rule("REPLACE", "phrase")),
    # «направо» во время песни — параллельно; движение во время движения — preempt Nav2
    "motion": _row(motion=Rule("REPLACE", "now")),
    # Tier-1 «стой»: речь, выступление, движение, музыка — сразу
    "halt": _row(speech=Rule("REPLACE", "now"), perform=Rule("REPLACE", "now"), backing=Rule("REPLACE", "now"),
                 track=Rule("REPLACE", "now"), motion=Rule("REPLACE", "now")),
    # query параллельно со всем; отменяется перебиванием (interrupt.py)
    "query": _row(),
}
COMPOSITION: Mapping[Tuple[str, str], Rule] = MappingProxyType({
    (new, busy): rule for new, row in _ROWS.items() for busy, rule in row.items()
})

# ---------------------------------------------------------------------------------------------------------------
# Команда LLM (§4.1): пределы и перечисления; схема и промпт генерируются ниже
# ---------------------------------------------------------------------------------------------------------------

SPEECH_MAX_CHARS = 280
ACTIONS_MAX = 3
PERFORM_KINDS: Tuple[str, ...] = ("rap", "song", "tale", "poem")
PERFORM_BACKINGS: Tuple[str, ...] = ("beat", "calm", "none")
SEGMENTS_MIN = 2
SEGMENTS_MAX = 12
SEGMENT_MAX_CHARS = 400
WHEN: Tuple[str, ...] = ("after_current", "now")
ASK_EXPECTS: Tuple[str, ...] = ("yes_no", "name", "free")
PASSED: Tuple[str, ...] = ("not_addressed", "nothing_to_say", "unclear")
#: Режимы формы ответа (§4.5): ``tool`` — один обязательный тул ``command``; ``json`` — схема для
#: structured_output/JSON-режима и для JSON в тексте.
COMMAND_MODES: Tuple[str, ...] = ("tool", "json")
COMMAND_TOOL = "command"


def _enum(values: Sequence[str]) -> Dict[str, Any]:
    return {"type": "string", "enum": list(values)}


def _object(props: Mapping[str, Any], required: Sequence[str] = ()) -> Dict[str, Any]:
    return {"type": "object", "properties": dict(props), "required": list(required), "additionalProperties": False}


def _expression_schema() -> Dict[str, Any]:
    face = {slot: _enum(values) for slot, values in FACE_SCENE_SLOTS.items()}
    face["duration_s"] = {"type": "number", "minimum": 0, "maximum": FACE_SCENE_MAX_S}
    return _object({
        "emotion": _enum(EMOTIONS), "gesture": _enum(GESTURES), "earcon": _enum(EARCONS), "face": _object(face),
    })


def command_schema(mode: str = "json") -> Dict[str, Any]:
    """JSON-схема команды LLM из таблиц этого модуля: ``json`` — сама схема, ``tool`` — тул ``command``.

    :raises ValueError: незнакомый режим.
    """
    if mode not in COMMAND_MODES:
        raise ValueError(f"command_schema: mode must be one of {COMMAND_MODES}, got {mode!r}")
    expression = _expression_schema()
    schema = _object({
        "speech": {"type": "string", "maxLength": SPEECH_MAX_CHARS},
        "ask": _object({"expects": _enum(ASK_EXPECTS), "text": {"type": "string", "maxLength": SPEECH_MAX_CHARS}},
                       required=("expects", "text")),
        "actions": {"type": "array", "maxItems": ACTIONS_MAX,
                    "items": _object({"tool": {"type": "string"}, "args": {"type": "object"}}, required=("tool",))},
        "perform": _object({
            "kind": _enum(PERFORM_KINDS),
            "segments": {"type": "array", "minItems": SEGMENTS_MIN, "maxItems": SEGMENTS_MAX,
                         "items": {"type": "string", "maxLength": SEGMENT_MAX_CHARS}},
            "backing": _enum(PERFORM_BACKINGS),
            "expression_per_segment": {"type": "array", "maxItems": SEGMENTS_MAX, "items": expression},
            "from_segment": {"type": "integer", "minimum": 0},
        }, required=("kind", "segments")),
        "when": _enum(WHEN),
        "edit_pending": {"type": "boolean"},
        "expression": expression,
        "passed": _enum(PASSED),
    })
    if mode == "json":
        return schema
    return {"type": "function", "function": {
        "name": COMMAND_TOOL, "description": "Одна команда хода: речь, вопрос, действия, экспрессия.",
        "parameters": schema,
    }}


def prompt_fragment() -> str:
    """Фрагмент системного промпта о форме ответа — из тех же таблиц, что схема (не вручную)."""
    def _set(values: Sequence[str]) -> str:
        return "{" + ", ".join(values) + "}"

    face = "; ".join(f"{slot} ∈ {_set(v)}" for slot, v in FACE_SCENE_SLOTS.items())
    lines: List[str] = [
        "Ответ — одна команда (JSON-объект) с полями:",
        f"- speech: реплика человеку, ≤ {SPEECH_MAX_CHARS} символов. Если есть actions, speech не озвучивается:"
        " о результате скажет код по событию исполнителя.",
        f"- ask: вопрос с ожиданием ответа, expects ∈ {_set(ASK_EXPECTS)}.",
        f"- actions: ≤ {ACTIONS_MAX} вызовов тулов {{tool, args}}; не больше одного на класс"
        f" {' и '.join(SINGLE_ACTION_CLASSES)}.",
        f"- perform: выступление kind ∈ {_set(PERFORM_KINDS)}, {SEGMENTS_MIN}–{SEGMENTS_MAX} сегментов"
        f" по ≤ {SEGMENT_MAX_CHARS} символов, backing ∈ {_set(PERFORM_BACKINGS)}; вместо speech.",
        f"- when ∈ {_set(WHEN)}: after_current — после текущего (по умолчанию), now — сейчас.",
        "- edit_pending: true — заменить ещё не начатые сегменты текущего выступления, начиная с from_segment.",
        f"- expression: emotion ∈ {_set(EMOTIONS)}; gesture ∈ {_set(GESTURES)}; earcon ∈ {_set(EARCONS)};"
        f" face: {face}; duration_s ≤ {FACE_SCENE_MAX_S:g}.",
        f"- passed ∈ {_set(PASSED)} — честно промолчать.",
    ]
    return "\n".join(lines)


# ---------------------------------------------------------------------------------------------------------------
# Загрузка вейк-слов из YAML (прод: /config/wake_words.yaml)
# ---------------------------------------------------------------------------------------------------------------


def load_wake_word_namespaces(path: "str | os.PathLike[str] | None") -> Dict[str, List[str]]:
    """Прочитать ``wake_words.yaml`` в ``{namespace: [words]}``, порядок — как в файле.

    Пустой путь, нет файла, битый YAML, чужая структура — ``{}`` (не падает): вызывающий берёт дефолты кода.
    """
    if not path:
        return {}
    import yaml  # noqa: PLC0415 — лениво: без пути модуль не тянет YAML

    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: Dict[str, List[str]] = {}
    for namespace in WAKE_NAMESPACES:
        words = data.get(namespace)
        if isinstance(words, list):
            cleaned = [w for w in words if isinstance(w, str) and w.strip()]
            if cleaned:
                out[namespace] = cleaned
    return out


def resolve_wake_word_namespaces(
    path: "str | os.PathLike[str] | None",
    personality_fallback: Sequence[str] = DEFAULT_WAKE_WORDS,
    operator_fallback: Sequence[str] = DEFAULT_OPERATOR_WAKE_WORDS,
) -> Tuple[List[str], List[str]]:
    """``(personality, operator)``: из файла, а где namespace нет — из переданных фолбеков (порядок важен)."""
    loaded = load_wake_word_namespaces(path)
    personality = loaded.get("personality") or list(personality_fallback)
    operator = loaded.get("operator") or list(operator_fallback)
    return personality, operator
