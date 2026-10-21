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
from typing import Any, Dict, FrozenSet, List, Mapping, Optional, Sequence, Tuple

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
)
#: Рискованные вейки (решение В18): «рома» — обычное имя в разговоре (живой лог 20.10 «рома епта…» разбудил
#: робота), «бот»/«роб» — короткие и звучат внутри чужих фраз. В ``wake_words.yaml`` — записи ``risky: true``,
#: по умолчанию выключены (``risky_enabled: false``); включаются одной строкой. Стоят в конце: «роб» не должен
#: съесть «роб бокс».
RISKY_WAKE_WORDS: Tuple[str, ...] = ("рома", "бот", "роб")
#: Голое «робот» (В12): после вейка нет слов, кроме этих, — фразы нет, это обращение «слушай».
ADDRESS_FILLER_WORDS: Tuple[str, ...] = ("ну", "а", "эй", "слушай", "так", "вот", "это", "ой", "да", "давай")
#: Бэклог неадресованных фраз (§7.3): живёт TTL. Голое «робот» забирает ВСЕ фразы не старше ``BACKLOG_PICKUP_S``
#: (решение Шифу 21.10, #3767: «бэклог должен уходить по любому всегда») вместе с более ранними кусками того же
#: рассказа (audio_node режет речь по 15 с, #3763): кусок примыкает, если кончился не раньше чем за
#: ``BACKLOG_STORY_GAP_S`` до начала следующего. Не больше ``BACKLOG_PICKUP_MAX`` фраз за раз.
BACKLOG_TTL_S = 120.0
BACKLOG_PICKUP_S = 20.0
BACKLOG_STORY_GAP_S = 3.0
BACKLOG_PICKUP_MAX = 6
BACKLOG_MAX_ENTRIES = 30
#: Метка фразы бэклога, когда голос не узнан уверенно (#3767): «неопознанный» + что есть по голосу.
UNIDENTIFIED_SPEAKER = "неопознанный"
#: «Тот же собеседник» (#3695): ИЛИ по каналам — голос длинной фразы узнан, тот же человек в кадре, (слабо) DOA.
#: С #3767 это не фильтр подхвата, а поле ``same_as_caller`` метки и отбор ≤ 1 фразы в снимок адресованного хода.
PICKUP_CHANNELS: Tuple[str, ...] = ("voice", "face", "doa")
#: Сектор DOA ReSpeaker (±градусы): замер 21.10 — размах 304°…65° у неподвижного источника, ±30° промахивался.
PICKUP_DOA_SECTOR_DEG = 60.0
#: DOA — слабый признак: засчитывается, только когда в кадре кто-то есть («в кадре никого → только голос»).
PICKUP_DOA_NEEDS_FACE = True
#: Голосу фразы верим от этой длины (с): короткое «робот» биометрия не различает (Шифу 21.10).
VOICE_MIN_PHRASE_S = 2.0
#: Сколько секунд помнить лица из кадра (``/vision/hailo/events``) для сверки с фразой.
FACE_HISTORY_S = 60.0
#: Голое «робот» без свежей фразы в бэклоге: кадр ожидания (лицо «слушаю»), потом снимается молча.
ADDRESS_WAIT_S = 8.0
#: Префикс обращения к координатору (Claude Code) в Telegram: такая реплика роботу не адресована (#3518).
COORDINATOR_NAMES: Tuple[str, ...] = ("клод", "claude")
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
    """Tier-1 переопределение политики планировщика (§5.6.4): закрытый список слов → политика и граница.

    ``lead_only`` — формы, которые считаются только ПЕРВЫМ словом команды (после вейка и ``STOP_LEAD_WORDS``)."""

    words: Tuple[str, ...]
    policy: str
    boundary: str
    lead_only: Tuple[str, ...] = ()


#: «стоп/хватит/замолчи» — заменить на границе предложения, «стой» — сразу, «потом» — после блока, «сейчас» —
#: на границе. Правила планировщика берут политику отсюда, а не из текста ответа LLM (``words.policy_word``).
#: Голое «сейчас» — частое слово вопросов («который сейчас час»): только первым словом команды («сейчас спой»).
TIER1_POLICY: Mapping[str, PolicyWords] = MappingProxyType({
    "stop": PolicyWords(("хватит", "стоп", "замолчи", "отставить"), "REPLACE", "sentence"),
    "halt": PolicyWords(("стой", "стойте"), "REPLACE", "now"),
    "later": PolicyWords(("потом", "после", "когда закончишь"), "APPEND", "block"),
    "now": PolicyWords(("сейчас", "прямо сейчас"), "REPLACE", "sentence", lead_only=("сейчас",)),
})

#: Стоп-слова Tier-1 (§6.2, В19) — ТОЧНЫЕ формы, сверка по границам слов (``words.stop_verdict``): «стопка» и
#: «стоит» — не стоп. Состав — слова политик ``stop``/``halt`` из :data:`TIER1_POLICY` и повелительные формы основ
#: :data:`SILENCE_WORDS` (держит ``test_knowledge.py``).
STOP_WORDS: Tuple[str, ...] = (
    "стоп", "стой", "стойте", "хватит", "замолчи", "замолчите", "помолчи", "помолчите", "отставить",
)
#: Слова, которые стоят перед стоп-словом, не делая фразу другой командой: «всё, хватит», «ну стоп», «да хватит
#: уже». Стоп-слово после любого другого слова («спой песню про стоп») — спорный случай: его решает PR-6 (Chooser).
STOP_LEAD_WORDS: Tuple[str, ...] = ("всё", "все", "ну", "так", "эй", "да", "ладно", "а", "уже", "слушай")
#: Частица после стоп-слова делает его вопросом, а не командой: «хватит ли нам сил» — спорный случай.
STOP_QUESTION_PARTICLES: Tuple[str, ...] = ("ли",)

#: Громкость Tier-1 (§4.2, ``grammar``): ОСНОВЫ слов направления по намерению; матч — слово начинается с основы.
#: «кромче» — STT вместо «громче» (живой лог #3125). Тул канала получает намерение как ``action``.
VOLUME_STEMS: Mapping[str, Tuple[str, ...]] = MappingProxyType({
    "louder": ("громче", "погромче", "кромче", "покромче", "прибав", "подкрут", "выкрут"),
    "quieter": ("тише", "потише", "убав", "приглуш"),
    "max": ("максимум", "максимальн", "полную"),
    "normal": ("нормальн", "обычн", "стандартн"),
})
VOLUME_INTENTS: Tuple[str, ...] = tuple(VOLUME_STEMS)


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
#: Слово, которое само называет канал громкости: «музыку громче» — музыка, «говори тише» — голос робота.
VOLUME_CHANNEL_STEMS: Mapping[str, Tuple[str, ...]] = MappingProxyType({
    "music": ("музык", "музон", "трек", "трей", "бит", "песн", "мелоди", "сет", "дидж", "играй", "сыграй", "поиграй"),
    "tts": ("голос", "говори", "говорить", "разговарива", "себя"),
})
#: Слова «про громкость», не меняющие ни намерения, ни канала: «сделай громкость потише», «звук погромче».
VOLUME_TOPIC_STEMS: Tuple[str, ...] = ("громк", "звук", "уровень", "сделай", "можешь", "можно")

# ---------------------------------------------------------------------------------------------------------------
# Tier-1 закрытая грамматика (§4.2, ``grammar.match``): виды команд, служебные слова, голос, новая сессия
# ---------------------------------------------------------------------------------------------------------------

#: Вид Tier-1 команды → звучит ли её исполнение. В SILENCED (§6.3) грамматика отдаёт только незвучащие виды:
#: стоп, тишина, её снятие, громкость, новая сессия (сброс снимает и тишину, §7.1). Смена голоса звучит —
#: тул ``set_voice`` сам говорит «Говорю голосом X» (PR-4).
TIER1_KINDS: Mapping[str, bool] = MappingProxyType({
    "stop_all": False, "silence": False, "unsilence": False, "volume": False, "new_session": False,
    "set_voice": True, "answer": False,
})
#: Служебные слова закрытой грамматики: команда «покрыта целиком», если вне них в ней только слова команды.
#: Этот же список — служебные слова медиа-грамматики v1 (``media_command_grammar._COMMON_FILLER``).
GRAMMAR_FILLER: Tuple[str, ...] = (
    "ну", "а", "и", "но", "же", "ка", "эй", "йо", "пожалуйста", "плиз",
    "можно", "давай", "сделай", "сделайте", "это", "так", "вот", "уже",
    "ещё", "еще", "сейчас", "там", "тут", "мне", "нам", "её", "ее", "его",
    "эту", "этот", "всё", "все", "всю", "ты", "вы", "чуть", "чуточку",
    "немного", "очень", "на", "в", "по", "раз", "два", "раза", "чтобы",
    "чтоб", "режим", "режима", "совсем",
)
#: Сверх служебных — слова просьбы о громкости («играй громче», «включи погромче»).
VOLUME_FILLER: Tuple[str, ...] = (
    "включи", "поставь", "сильнее", "больше",
    "меньше", "побольше", "поменьше", "тихо", "слишком", "громко",
)
#: Глаголы заказа: «сыграй <название> погромче» — заказ трека, не громкость (решает медиа-грамматика/LLM).
PLAY_VERBS: Tuple[str, ...] = ("играй", "сыграй", "играйте", "включи", "поставь", "запусти", "вруби", "врубай")
#: Голос робота (§4.2, #3744): «включи голос надёжного мужика» — ``set_voice`` кодом по реестру голосов
#: (``tts_voice_registry``, приходит снаружи). Слово «голос*» обязательно; кроме имени голоса — только эти слова.
VOICE_NOUN_STEM = "голос"
VOICE_FILLER: Tuple[str, ...] = (
    "включи", "поставь", "смени", "поменяй", "переключи", "переключись", "сделай", "говори", "заговори",
    "давай", "хочу", "на", "мне", "свой", "твой", "этот", "другой", "пожалуйста", "можешь", "ну", "а",
)
#: Сколько первых букв слова человека и слова имени голоса должны совпасть: «надёжного мужика» ↔ «Надёжный
#: мужчина» (``муж``), «Алёну» ↔ «Алёна».
VOICE_STEM_LEN = 3
#: «Новая сессия» (U8): фразы — подстрокой после снятия вейка (как в v1); Telegram «/clear» — правило хоста.
NEW_SESSION_PHRASES: Tuple[str, ...] = (
    "новая сессия", "новую сессию", "новый диалог", "новый разговор",
    "начать заново", "начни заново", "начнём заново", "начнем заново", "начать сначала", "начни сначала",
    "сбрось всё", "сбрось все", "сбросить всё", "сброс сессии", "сброс диалога", "сбросить диалог",
    "забудь всё", "забудь все", "забудь что было",
    "очисти историю", "очистить историю", "сотри историю", "стереть историю",
)
#: Ответ на вопрос робота ``Ask(expects="yes_no")`` (U9) — точные формы из :data:`SHORT_ANSWERS`.
YES_WORDS: Tuple[str, ...] = ("да", "ага", "угу", "ок", "ладно")
NO_WORDS: Tuple[str, ...] = ("нет", "не")
#: В19: Chooser спорного стопа — варианты ответа, дедлайн (таймаут/ошибка → ``part_of_command``, речь не рвётся).
#: 1,5 с — решение Шифу 21.10 08:22: DeepSeek (``llm_harness``) отвечает за 1,1 с, при 1 с Chooser всегда опаздывал.
STOP_CHOICES: Tuple[str, ...] = ("stop", "part_of_command")
STOP_CHOOSER_DEADLINE_MS = 1500
#: #3765 (идея Шифу 21.10): Chooser «LLM в этом ходе уже спросила о том же?» перед вопросом личности кодом; дедлайн тот
#: же, что у спорного стопа. Таймаут/ошибка/нет ключа → ``not_said`` (лучше спросить дважды, чем не спросить).
IDENTITY_DEDUP_CHOICES: Tuple[str, ...] = ("already_said", "not_said")
#: Chooser спрашивается ПАРАЛЛЕЛЬНО с речью хода; ждём его не дольше конца этой речи + ``IDENTITY_DEDUP_AFTER_SPEECH_S``
#: (вопрос не звучит позже, чем без Chooser'а, если речь хода длиннее ответа модели). Дедлайн самой модели — с
#: запасом на длинную речь хода (живой робот 21.10: llm_harness 1.45–1.5 с, при дедлайне 1.5 с — 4 таймаута из 6).
IDENTITY_DEDUP_AFTER_SPEECH_S = 1.0
IDENTITY_DEDUP_DEADLINE_MS = 6000

# ---------------------------------------------------------------------------------------------------------------
# Классы действий (§5.1) и тулы по классам
# ---------------------------------------------------------------------------------------------------------------

#: Агенты на общем движке (§9): личность, оператор (ТАРС), телеграм.
AGENTS: Tuple[str, ...] = ("personality", "operator", "telegram")
#: Агент, чей ``speech`` - текст в чат, а не озвучка: при действиях он законен (§4.1 «в текстовый канал»).
TEXT_CHANNEL_AGENT = "telegram"
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
        "set_volume",
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
#: Срез тулов хода (§4.2, ``tools_view``): LLM видит ≤ ``TOOLS_VISIBLE_MAX`` схем, а не весь каталог. Всегда —
#: речь, память, время, поиск, анимация; остальное — группы по словам человека и по сценам снимка (кадр DJ, музыка).
TOOLS_VISIBLE_MAX = 12
TOOLS_ALWAYS: Tuple[str, ...] = (
    "speak_text", "memory_save", "memory_search", "get_current_time", "search_web", "play_animation",
)
#: Блоки одного вызова LLM (§7.3, §14 A5; ``context.measure``): персона, факты собеседника и сделанное в прошлых ходах,
#: окно прошлых ходов, снимок хода, реплика с тул-циклом хода, схемы тулов.
CONTEXT_BLOCKS: Tuple[str, ...] = ("system", "facts", "window", "snapshot", "turn", "tools")
#: Бюджет блоков в токенах (оценка ``rob_box_core.token_estimate``); ``facts`` и ``turn`` меряются без бюджета.
CONTEXT_BUDGET_TOKENS: Mapping[str, int] = MappingProxyType({
    "system": 4000, "snapshot": 1000, "tools": 4000, "window": 3000,
})
#: Разделы снимка хода ``<system_context>`` в порядке вывода (``context.build_context``): данные, а не правила.
#: ``user_profile`` — пока строки шва собеседника v1 (``turn_identity``), структурой — с контрактом §7.5 (PR-11).
#: ``speaker`` — кто сказал реплику (``speaker_adapter.speaker_block``): имя только при ``confirmed`` (§7.5, A30).
CONTEXT_SECTIONS: Tuple[str, ...] = (
    "user_profile", "speaker", "tts_context", "speech_gender", "position", "providers", "music_state", "recent_unaddressed",
)


@dataclass(frozen=True)
class ToolGroup:
    """Группа тулов среза: ``stems`` — основы слов человека (матч по левой границе слова), ``tools`` — ≤ 6 схем.

    ``weak`` — общие глаголы («включи», «поставь»): группа по ним одним уступает группам, названным прямо
    («включи голос …» — голос, а не музыка).
    """

    stems: Tuple[str, ...]
    tools: Tuple[str, ...]
    weak: Tuple[str, ...] = ()


#: Глаголы восприятия: «посмотри налево», «глянь что справа», «посмотри на меня» — взгляд лица
#: (``play_animation(face={gaze: …})``). Движение базы они НЕ открывают (#3779: «посмотри налево» ехало на 1 м).
GAZE_VERB_STEMS: Tuple[str, ...] = ("посмотр", "смотри", "глян", "взгляни", "взглян")
#: Единственная таблица глаголов движения базы: только названные ими тулы ``MOTION_TOOLS`` попадают в срез хода и
#: исполняются. Направления без глагола («налево», «вперёд») движение не открывают. Матч — по левой границе слова.
MOTION_VERB_STEMS: Tuple[str, ...] = (
    "поезжай", "поедь", "езжай", "едь", "приедь", "иди", "пойд", "пройд", "двигай",
    "подвинь", "отъед", "отъезж", "подъед", "подъезж", "проезжай", "поехали", "едем", "приезжай", "отвези", "вези",
    "вернись", "катись",
)
#: «Повернись/развернись» — движение базы, но «повернись глазами / взглядом / головой» — взгляд лица.
MOTION_TURN_STEMS: Tuple[str, ...] = ("поверн", "разверн")
MOTION_EYES_STEMS: Tuple[str, ...] = ("глаз", "взгляд", "взглян", "голов")
#: Тулы, двигающие базу по слову человека: без глагола движения в реплике не видны LLM и не исполняются.
MOTION_TOOLS: Tuple[str, ...] = ("move_direction", "navigate_to_waypoint", "navigate_to_coordinates")

#: Группы по приоритету: при пересечении групп срез режется по этому порядку (``TOOLS_VISIBLE_MAX``). Голос —
#: только по явной голосовой просьбе (#3744: LLM сама меняла голос посреди сказки, а голос персистентный, PR-4).
TOOL_GROUPS: Mapping[str, ToolGroup] = MappingProxyType({
    "music": ToolGroup(
        ("музык", "музон", "песн", "песен", "трек", "мелоди", "сыгра", "играй", "игра", "спой", "спеть", "споёшь",
         "споешь", "сочини", "сгенер", "дидж", "сет", "сэт", "бит", "библиотек", "композ", "послуша",
         "рэп", "вечеринк", "танц", "пати"),
        ("request_music", "generate_music", "music_library", "dj_set", "stop_music", "get_music_state"),
        weak=("включи", "поставь"),
    ),
    "volume": ToolGroup(("громк", "громче", "погромче", "тише", "потише"), ("set_volume", "set_music_volume")),
    "voice": ToolGroup(
        ("голос", "провайдер", "яндекс", "минимакс", "силеро", "silero", "yandex", "minimax", "быстр",
         "побыстр", "медленн", "помедленн", "скорост", "темп реч", "выше", "ниже", "тембр"),
        ("set_voice", "set_tts_provider", "list_tts_voices", "set_speed", "set_pitch", "estimate_tts_duration"),
    ),
    "navigation": ToolGroup(
        ("поезжай", "поедь", "езжай", "едь", "приедь", "иди", "двигай", "поверн", "разверн", "точк", "кухн",
         "комнат", "маршрут", "где ты", "вернись", "остановись", "отвези", "вези", "поехали", "едем", "приезжай",
         "туда", "сюда", "отъед", "подъед", "катись"),
        ("navigate_to_waypoint", "list_waypoints", "move_direction", "save_waypoint", "stop_navigation",
         "get_current_pose"),
    ),
    "waypoints_edit": ToolGroup(
        ("удали точк", "удали все точк", "очисти точк", "координат"),
        ("delete_waypoint", "clear_waypoints", "navigate_to_coordinates", "list_waypoints"),
    ),
    "mapping": ToolGroup(("карт",),
                         ("start_mapping", "continue_mapping", "finish_mapping", "optimize_map", "load_map")),
    "people": ToolGroup(("зовут", "помни", "запомни", "знаком", "обо мне", "кто я", "моё имя", "мое имя"),
                        ("memory_context", "register_speaker")),
    "sound": ToolGroup(("звук",), ("play_sound", "get_sound_info")),
    # анимация всегда видна (``TOOLS_ALWAYS``); группа — чтобы просьба «покажи анимацию …» была командой (В17)
    # глаголы восприятия («посмотри налево») — взгляд лица, а не движение базы (#3779, ``MOTION_VERB_STEMS``)
    "animation": ToolGroup(("анимац", "изобрази", *GAZE_VERB_STEMS), ("play_animation",)),
    "event": ToolGroup(("мероприят", "фестивал", "расписан", "программ", "умеешь"), ("faq_search",)),
    "status": ToolGroup(
        ("батар", "заряд", "статус", "видишь", "перед тобой", "слышишь", "как ты", "самочувств", "подожди"),
        ("get_robot_status", "get_battery_level", "get_perception_context", "listen_for_response"),
    ),
})
#: Ни одна группа не совпала со словами — тулы статуса (как раньше «core»-скилл каталога).
TOOL_GROUPS_DEFAULT: Tuple[str, ...] = ("status",)
#: Группы только по явной просьбе в ЭТОЙ реплике: в следующий нейтральный ход не переносятся (#3744 — голос
#: персистентный, «расскажи анекдот» после «какие у тебя голоса» не должен видеть set_voice; робот 21.10).
TOOL_GROUPS_EXPLICIT: Tuple[str, ...] = ("voice",)
#: Тулы каталога, которых нет в срезе LLM: их зовёт только код (Tier-1 «включи X» — lookup_melody медиароутера;
#: add_music_material — служебный тул старого аранжировщика, движку v2 не нужен).
TOOLS_CODE_ONLY: Tuple[str, ...] = ("lookup_melody", "search_melody", "add_music_material")
#: Классы, действий которых в одной команде не больше одного (каждого): один трек и одна цель движения за ход.
SINGLE_ACTION_CLASSES: Tuple[str, ...] = ("action.media", "action.motion")
#: Ключи фраз, особые для тула (поверх ключа класса): «Запомнил: …» строится по событию saved владельца памяти.
TOOL_PHRASES: Mapping[str, Mapping[str, str]] = MappingProxyType({
    "memory_save": MappingProxyType({"done": "memory.saved"}),
    # звук - не музыка: класс action.media, но фраза о SoundEvent, а не «Включаю музыку»
    "play_sound": MappingProxyType({"rejected": "sound.skipped", "timeout": "sound.timeout"}),
    # стоп - не запуск: «Выключил музыку», а не «Включаю музыку» класса action.media
    "stop_music": MappingProxyType({"done": "music.stop_ok", "rejected": "music.stop_fail"}),
})
#: Действия, чей успех - сам себе доказательство: фразы об исходе done нет (§4.1; Шифу 21.10: «включил щелчок и ещё
#: сказал, что он заиграл, бред же»). Неудача (rejected/timeout) говорится одной фразой кода.
SILENT_DONE_TOOLS: Tuple[str, ...] = ("play_sound", "play_animation")
#: Тулы, об исходе которых в v1 говорит не фраза исполнителя (``respond``), а другой код по тому же событию: голос -
#: сам тул (``voice_request``, SpeechEvent ``voice_changed``), запуск музыки и библиотека - музыкальный ход ноды
#: (``music_turn``, ADR-0157 R6). Режим команд v1 (PR-7 ч.7c) их фразу не дублирует; в v2 (PR-8) - фраза респонда.
OUTCOME_SPOKEN_ELSEWHERE: Mapping[str, str] = MappingProxyType({
    "set_voice": "tool", "set_tts_provider": "tool",
    "dj_set": "music_turn", "request_music": "music_turn", "generate_music": "music_turn", "music_library": "music_turn",
})
#: play_sound при занятом канале (#3698, просьба Шифу 21.10): короткий текущий звук (earcon ~1 с) - запрос ждёт в
#: очереди владельца звука и играет следом; ждать не дольше дедлайна события play_sound минус запас на запуск и
#: доставку события. Текущий звук длиннее (стингер 12 с) - честный skipped{busy}.
SOUND_QUEUE_MAX_WAIT_S: float = float(ACTION_CLASSES[TOOL_CLASSES["play_sound"]].timeout_s) - 1.0
#: Короткие ответы на вопросы робота («выключить и зачитать?», «это ты, Вася?», «верно?»): STT не отклоняет их как
#: «слишком короткую фразу» (порог ``min_text_chars``). Сравнение - по нормализованной форме (нижний регистр, без
#: пунктуации): «Да.» = «да». Единственная таблица; мусор («э», «ммм») в неё не входит.
SHORT_ANSWERS: Tuple[str, ...] = ("да", "нет", "не", "ага", "угу", "ок", "ладно", "стоп")
#: Виды провайдеров (``ProviderState.kind``, ADR §8): ``music_gen`` - генерация музыки (KIE/Suno, ADR-0158).
PROVIDER_KINDS: Tuple[str, ...] = ("llm", "stt", "tts", "music_gen")
#: Классы здоровья провайдера (``ProviderState.health``). Единственная таблица: ``degraded`` - ещё пробуем (лимит
#: частоты), остальные - провайдер пропускаем; причина (quota/network/timeout/auth) - класс, а не подстрока ответа.
PROVIDER_HEALTH: Tuple[str, ...] = ("ok", "degraded", "quota", "network", "timeout", "auth", "down", "unknown")
#: Записи ``ProviderState`` старше этого срока без переиздания писателем считаются устаревшими (``unknown``).
PROVIDER_STALE_S: float = 10.0
#: Классы здоровья, при которых провайдер ещё годится в цепочку.
PROVIDER_USABLE_HEALTH: Tuple[str, ...] = ("ok", "degraded", "unknown")
#: Транзиентный отказ (таймаут/сеть) STT-провайдера переводит его в ``timeout``/``down`` лишь после стольких
#: отказов подряд НА РЕЧИ (фразу приняли другие провайдеры). Таймаут на фоновом шуме (итог фразы ``rejected``) не
#: считается: 21.10 MiniMax-фолбек раз в 10 мин «падал» на шуме, и робот врал вслух (#3755). Квота и ключ
#: (``quota``/``auth``) - сразу: они не бывают случайными. 3 = ступень лестницы TTL (30/120/300 с) в ``stt_fallback``.
STT_TRANSIENT_STRIKES_TO_DOWN: int = 3
#: Особые ключи каталога фраз (не по классу действия): их обязан содержать ``phrases/ru.yaml``.
PHRASE_KEYS: Tuple[str, ...] = (
    "understand.timeout",
    *(f"provider.exhausted.{reason}" for reason in ("quota", "network", "timeout", "auth", "error")),
    "speak_aloud.rejected.unsupported_script", "speak_aloud.timeout",
    "motion.rejected.not_asked", "provider.switched", "provider.no_fallback", "provider.switched.stt",
    *(f"provider.subject.{kind}" for kind in PROVIDER_KINDS),
    "voice.applied", "voice.unavailable", "voice.rejected", "voice.timeout",
    *(f"perform.no_backing.{reason}" for reason in ("quota", "network", "timeout", "error")),
    "perform.ask_replace_music", "identity.ask_name", "identity.confirm",
    # Tier-1 громкость по каналу (``grammar``): фраза после ответа тула владельца канала
    *(f"{channel}.volume.{intent}" for channel in ("music", "voice") for intent in VOLUME_INTENTS),
    "music.volume_fail", "voice.volume_fail",
)

# ---------------------------------------------------------------------------------------------------------------
# Экспрессия (§5.5): эмоция → сцена лица, жесты, earcon'ы, слоты сцены для LLM
# ---------------------------------------------------------------------------------------------------------------

#: Анимации с манифестом, которые умеет сегодняшний плеер (``rob_box_mcp_tools.animations.KNOWN_ANIMATIONS``),
#: плюс сцены лица ``rob_box_animations/face/states.py:SCENES`` (§5.5.2): ими именуются сцены лица в v1 и v2.
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
#: ``talk`` — рот по огибающей TTS. ``bg`` — фон под глазами и ртом, ``fx`` — разовый импульс в начале сцены, ``feat`` —
#: замена самих черт (#3780; рендер — ``rob_box_animations/face/effects.py``).
FACE_SCENE_SLOTS: Mapping[str, Tuple[str, ...]] = MappingProxyType({
    "eyes": ("open", "half", "squint", "wide", "closed", "shades", "x"),
    "gaze": ("speaker", "left", "right", "up", "down", "wander"),
    "blink": ("normal", "rapid", "none", "wink_left", "wink_right"),
    "mouth": ("grin", "frown", "flat", "o", "teeth", "eq", "talk"),
    "tint": ("warm", "cold", "red", "green", "blue", "purple", "rainbow"),
    "tempo": ("slow", "normal", "fast"),
    "rear": ("off", "markers", "hazard", "party", "new_year", "halloween", "heart", "rocket"),
    "bg": ("none", "rainbow", "fire", "matrix"),
    "fx": ("slash", "glitch", "boot", "power_down"),
    "feat": ("loading_eyes",),
})
FACE_SCENE_DEFAULT_S = 5.0
FACE_SCENE_MAX_S = 30.0
#: Описание схемы сцены для LLM (идёт в схему, не в промпт). ``left``/``right`` - со стороны человека напротив робота.
FACE_SCENE_HINT = ("Слоты лица; всё, что не задано, берётся из текущей сцены. left/right - со стороны человека"
                   " напротив робота. Только значения из списков, свободных цветов и форм нет. bg - фон под глазами"
                   " и ртом; fx - разовый импульс в начале сцены (slash - удар катаной, glitch - сбой, boot -"
                   " включение, power_down - выключение); feat=loading_eyes - спиннеры вместо глаз (жду, думаю).")
#: Стадии хода, чьи сцены лица снимают сцену LLM (``failed``/``silenced``: «радостно» о провале не бывает).
FACE_SCENE_CANCEL_STAGES: Tuple[str, ...] = ("failed", "silenced")

#: Режимы кольца LED ReSpeaker в рефлексах.
RING_MODES: Tuple[str, ...] = ("off", "breath", "doa_sector", "spin_slow", "flash", "flash_red", "beat_pulse")


@dataclass(frozen=True)
class Reflex:
    """Рефлекс кода на стадию хода/состояние робота (§5.5.3): сцены лица, кольцо, earcon — 0 вызовов LLM.

    ``face`` — сцены по очереди/на выбор (``idle_stare ↔ idle_bored``); пусто — лицо не трогается. ``after_ms`` —
    через сколько после события включается, ``deadline_ms`` — не позже чем. Минимальная длительность сцены и
    ``ttl`` подбираются в PR-3b по видео и добавляются сюда же. ``ttl_ms`` - сколько лицо держит сцену рефлекса, если
    следующей стадии не будет (страховка от зависшей сцены): терминальные стадии короткие, ожидание - до таймаута хода.
    """

    face: Tuple[str, ...]
    ring: Optional[str] = None
    earcon: Optional[str] = None
    after_ms: Optional[int] = None
    deadline_ms: Optional[int] = None
    ttl_ms: Optional[int] = None


#: Стадия хода / состояние → рефлекс. ``thinking``: зубы или орбита — выбор Шифу (В13), старт с зубов.
REFLEXES: Mapping[str, Reflex] = MappingProxyType({
    "idle": Reflex(("idle_stare", "idle_bored"), ring="breath", after_ms=3000),
    "addressed": Reflex(("listening",), ring="doa_sector", earcon="heard", deadline_ms=300, ttl_ms=8000),
    "thinking": Reflex(("thinking_teeth",), ring="spin_slow", ttl_ms=45000),
    "executing": Reflex(("thinking_teeth",), ttl_ms=60000),
    "speaking": Reflex(("talking",), ring="breath", ttl_ms=20000),
    "done": Reflex(("happy", "wink"), ring="flash", earcon="done", ttl_ms=3000),
    "failed": Reflex(("error",), ring="flash_red", earcon="fail", ttl_ms=4000),
    "timeout": Reflex(("sad",), ring="flash_red", earcon="fail", ttl_ms=4000),
    "silenced": Reflex(("sleep",), ring="off"),
    "music.playing": Reflex(("music_groove", "music_shades", "music_vibe"), ring="beat_pulse"),
    "not_addressed": Reflex(()),
    "battery.low": Reflex(("low_battery",)),
    "charging": Reflex(("charging",)),
})
#: Сцены рефлексов, которые снимают сцену LLM с лица (выводятся из ``REFLEXES``, второй таблицы нет).
FACE_SCENE_CANCEL_SCENES: Tuple[str, ...] = tuple(
    scene for stage in FACE_SCENE_CANCEL_STAGES for scene in REFLEXES[stage].face)
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


#: Календарь тем фар: имена тем — ``rob_box_animations/face/rear.py:THEMES``. Окно — один день; дни рождения робота
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
SCENE_KINDS: Tuple[str, ...] = ("Silence", "Persona", "DJ", "OperatorHold", "Identity", "GreetingReply")
#: Кто кладёт кадр ``Silence``/``OperatorHold``: пользовательскую тишину снимает не оператор, и наоборот.
SCENE_SETTERS: Tuple[str, ...] = ("user", "operator", "code")
#: Собеседник хода, который назвался сам, а голос похож на другого (#3765): личность выясняет робот, LLM — только имя.
NAME_STATE_INTRODUCED = "introduced"
#: Пользовательская тишина «хватит/помолчи» — 10 минут (В3, Шифу 02.10).
SILENCE_TTL_S = 600
#: Окно ответа на реплику робота — 15 с после её конца (§7.2, §7.5): одна константа для вопроса об имени
#: (``Identity``) и для приветствия при встрече (``GreetingReply``, #3776, решение Шифу 21.10).
REPLY_WINDOW_S = 15
#: Кадр ``Identity(ask)`` — ответ ждётся ``REPLY_WINDOW_S``.
IDENTITY_TTL_S = REPLY_WINDOW_S
#: Окно 15 с идёт от КОНЦА речи самого вопроса (``SpeechEvent finished``), а не от конца хода (#3771, В17′).
#: Пока вопрос не договорён, кадр ждёт; это страховка, если событие конца потерялось (кадр снимается без ответа).
IDENTITY_WAIT_QUESTION_S = 180
#: Микрофон после собственной речи глух: столько audio_node (``tts_grace_s``) не слушает. Окно ответа открывается
#: после неё. Значение сверять с ``audio_node.tts_grace_s``.
IDENTITY_MIC_GRACE_S = 2.5
#: В17(б), решение Шифу 21.10 (#3695): ответ на вопрос робота принимается и без вейка, пока жив кадр ``Identity``,
#: если фраза ПОХОЖА НА ОТВЕТ по грамматике (``reply_grammar``: имя, да/нет, отказ). DOA и голос — не условия (DOA в
#: комнате прыгает у неподвижной колонки: 50° → 343°), только признаки в логе. Шифу отвергнет — ``False``.
IDENTITY_ANSWER_WITHOUT_WAKE = True
#: Окно после приветствия при встрече: фраза без вейка от приветствованного — ход (``source=greeting_reply``), если
#: она по грамматике ответ на приветствие или лицо приветствованного в кадре. Шифу отвергнет — ``False``.
GREETING_REPLY_WITHOUT_WAKE = True
#: Ответ на приветствие/реплику: фраза начинается с одной из этих (нормализовано: нижний регистр, ё -> е).
GREETING_REPLIES: Tuple[str, ...] = (
    "привет", "приветик", "здорово", "здравствуй", "здравствуйте", "салют", "хай", "и тебе", "и вам", "взаимно",
    "доброе утро", "добрый день", "добрый вечер", "доброй ночи", "как дела", "как ты", "как жизнь", "что нового",
    "ага", "угу", "спасибо", "благодарю", "рад тебя видеть", "и я рад", "я тоже", "тоже рад", "давно не виделись",
)
GREETING_REPLY_MAX_WORDS = 6
#: Кадр ``Identity`` с таким ``kind`` спрашивает ИМЯ (голое имя в ответ — представление); иначе ответ — да/нет.
IDENTITY_ASKS_NAME: Tuple[str, ...] = ("unknown", "contested")
#: Отказ называться (закрывает кадр без регистрации и без повторного вопроса): фраза начинается с одной из этих.
REPLY_REFUSALS: Tuple[str, ...] = ("не скажу", "не хочу", "не буду", "секрет", "потом", "не важно", "неважно", "зачем тебе")
#: Отказ — короткая фраза: «не скажу тебе, зануда» ещё отказ, длинный рассказ с «потом» внутри — нет.
REPLY_REFUSAL_MAX_WORDS = 4
#: Целые фразы-ответы «это ты?»: да / нет (слова нормализованы: нижний регистр, ё → е).
REPLY_YES: Tuple[str, ...] = ("да", "ага", "угу", "конечно", "верно", "точно", "это я", "да это я", "да я", "я")
REPLY_NO: Tuple[str, ...] = ("нет", "не", "неа", "не я", "это не я", "нет не я", "нет это не я")
#: Кадр ``Identity`` с таким ``kind`` спрашивает «один это человек или двое?» после регистрации (#2828, ADR-0127):
#: ``twin`` — имя совпало, голос не дотянул; ``conflict`` — голос похож на знакомого, имя другое. Окно ответа то же —
#: ``REPLY_WINDOW_S`` (#3765, рекомендация координатора 21.10: одно правило с В17′ вместо прежних 180 с).
IDENTITY_MERGE_KINDS: Tuple[str, ...] = ("twin", "conflict")
#: Один голос — одна личность (#3801, решение Шифу 21.10): фраза УВЕРЕННО узнана биометрией (у результата есть имя —
#: порог 0.72 и отрыв от соседа проверил ``speaker_id_node``, здесь порогов нет), а человек называет другое имя —
#: второй профиль не заводится, робот говорит ``identity.same_voice``; кадр ``Identity`` этого вида ждёт ответа.
IDENTITY_SAME_VOICE_KIND = "same_voice"
#: Подтверждение голосом (#3801): короткий ответ биометрия не узнаёт (живой 21.10: «да, это я» 1.17 с → 0.673 < 0.72,
#: длинная фраза 6.2 с → 0.95) — робот просит сказать шаблон ``identity.template``; кадр этого вида ждёт шаблон.
IDENTITY_VOICE_CHECK_KIND = "voice_check"
#: Кадры ``Identity``, ответ на которые решает код (фраза из ``phrases``, 0 LLM); по-представлению их ответ не регистрируется.
IDENTITY_CODED_KINDS: Tuple[str, ...] = IDENTITY_MERGE_KINDS + (IDENTITY_SAME_VOICE_KIND, IDENTITY_VOICE_CHECK_KIND)
#: Шаблон подтверждения — речь не короче этого (с запасом от 1.17 с короткого ответа, который биометрия не берёт).
IDENTITY_TEMPLATE_MIN_VOICE_S = 3.0
#: Слов в шаблоне ``identity.template`` не меньше: при обычном темпе ~2.5 слова/с выходит ``IDENTITY_TEMPLATE_MIN_VOICE_S``.
IDENTITY_TEMPLATE_MIN_WORDS = 8
#: Голос «неуверенный» — повод просить шаблон, а не отказ: узнан без имени (гипотеза/спор), биометрия не оценила
#: (мало речи) или лучший кандидат без порога не ниже этого (живой 21.10: 0.673/0.654). Ниже — чужой голос (#2829).
IDENTITY_TEMPLATE_MIN_SCORE = 0.6
#: Смена имени фразой (#3801): «зови меня Y», «переименуй меня в Y»; поправка «меня зовут не X, а Y» / «я не X, а Y» —
#: голова из ``REPLY_INTRO_PHRASES`` или «я», затем «не X а Y». Слова нормализованы (нижний регистр, ё → е).
REPLY_RENAME_PHRASES: Tuple[str, ...] = (
    "зови меня", "называй меня", "переименуй меня в", "переименуй меня", "запиши меня как", "запомни меня как",
)
#: Ответ «один человек» на вопрос склейки: обороты ищутся по границам слов (нижний регистр, ё → е).
REPLY_SAME_PHRASES: Tuple[str, ...] = (
    "это я", "я и есть", "тот самый", "та самая", "тот же", "та же", "один и тот же", "одно и то же", "один человек",
    "одно лицо",
)
#: Ответ «разные люди»: слово или оборот по границам слов. Оба сигнала сразу («не тот самый») — непонятно.
REPLY_DIFFERENT_WORDS: FrozenSet[str] = frozenset({
    "разные", "разных", "другой", "другая", "другие", "другого", "обознался", "обозналась",
})
REPLY_DIFFERENT_PHRASES: Tuple[str, ...] = ("не я", "не он", "не она", "не тот", "не та", "не один", "не одно")
#: Целые фразы «это я» на вопрос склейки (ответ на вторую половину «…или это ты?»): только точное совпадение — «да я не
#: знаю» не ответ.
REPLY_SAME_WHOLE: FrozenSet[str] = frozenset({"это я", "да это я", "да я", "ага я", "я", "да это же я"})
#: Сколько ход ждёт исход регистрации по представлению (ack ``speaker_id_node``) до вызова LLM: эмбеддинг фразы
#: ждётся до 1.5 с, запись в БД — миллисекунды (живой робот 21.10: 50 мс). Не дождались — ход идёт как раньше.
REGISTER_OUTCOME_WAIT_S = 2.5
#: Ответ на вопрос склейки — короткая фраза: «это я вчера включал тебе эту музыку» (7 слов) без вейка ответом не считается.
REPLY_SAME_MAX_WORDS = 6
#: Явное представление: «меня зовут X», «зови меня X» — X любое слово; «я X» — только имя из словаря.
REPLY_INTRO_PHRASES: Tuple[str, ...] = ("меня зовут", "зовут меня", "зови меня", "называй меня", "мое имя", "имя мне")
#: Слова, из которых не бывает голого имени: служебные, местоимения, короткие реплики, глаголы-связки.
REPLY_NOT_A_NAME: FrozenSet[str] = frozenset({
    "только", "что", "как", "это", "вот", "ну", "да", "нет", "не", "и", "а", "но", "или", "я", "мы", "ты", "он", "она",
    "оно", "они", "вы", "тут", "там", "здесь", "уже", "еще", "ещё", "все", "всё", "так", "тоже", "очень", "просто",
    "привет", "здравствуй", "здравствуйте", "спасибо", "пожалуйста", "хорошо", "ладно", "ок", "окей", "понятно",
    "ясно", "отлично", "пока", "стоп", "хватит", "помолчи", "робот", "робби", "роббокс", "слушай", "смотри", "давай",
    "давайте", "блин", "ого", "ой", "эй", "ага", "угу", "если", "когда", "потом", "сейчас", "почему", "зачем",
    "кто", "где", "куда", "какой", "какая", "какое", "один", "два", "три", "наше", "наш", "мой", "моя", "твой",
    "время", "утро", "день", "вечер", "ночь", "музыка", "музыку", "песню", "секрет",
})
#: Окончания глаголов (и повелительных «расскажи», «включи»): голое «имя» с таким окончанием — не имя (словарные имена проверка не касается).
REPLY_VERB_ENDINGS: Tuple[str, ...] = ("ть", "ти", "ли", "ал", "ил", "ел", "ют", "ут", "ет", "ит", "ешь", "ем", "ете",
                                       "ите", "ся", "сь", "ат", "ят", "ую", "ю",
                                       "чи", "жи", "ши", "ри", "ай", "яй", "ей")
#: Словарь имён (одна таблица): короткая форма «я X» (#2925, ``self_intro``) и грамматика ответа на вопрос об имени
#: (``reply_grammar``). Именительный падеж, нижний регистр, ё → е. Полные имена и частые уменьшительные.
KNOWN_FIRST_NAMES: FrozenSet[str] = frozenset({
    # мужские
    "александр", "саша", "шура", "алексей", "леша", "алеша", "анатолий",
    "толя", "андрей", "антон", "аркадий", "арсений", "артем", "артур",
    "борис", "боря", "вадим", "валентин", "валера", "валерий", "василий",
    "вася", "виктор", "витя", "виталий", "владимир", "вова", "володя",
    "владислав", "влад", "всеволод", "вячеслав", "слава", "геннадий",
    "гена", "георгий", "гоша", "жора", "глеб", "григорий", "гриша",
    "давид", "даниил", "данила", "даня", "денис", "денчик", "дэнчик",
    "дмитрий", "дима", "евгений", "женя", "егор", "иван", "ваня", "игорь",
    "илья", "кирилл", "константин", "костя", "лев", "лева", "леонид",
    "леня", "макар", "максим", "макс", "марк", "матвей", "михаил", "миша",
    "никита", "николай", "коля", "олег", "павел", "паша", "петр", "петя",
    "роман", "рома", "руслан", "семен", "сергей", "сережа", "степан",
    "степа", "тимофей", "тимур", "федор", "федя", "филипп", "эдуард",
    "юрий", "юра", "ярослав",
    # женские
    "алена", "алина", "алиса", "алла", "анастасия", "настя", "ангелина",
    "анна", "аня", "валентина", "валя", "варвара", "варя", "вера",
    "вероника", "виктория", "вика", "галина", "галя", "дарья", "даша",
    "диана", "ева", "евгения", "екатерина", "катя", "елена", "лена",
    "елизавета", "лиза", "жанна", "зоя", "инна", "ирина", "ира", "кира",
    "кристина", "ксения", "ксюша", "лариса", "лидия", "любовь", "люба",
    "людмила", "люда", "маргарита", "рита", "марина", "мария", "маша",
    "милана", "надежда", "надя", "наталья", "наталия", "наташа", "нина",
    "оксана", "олеся", "ольга", "оля", "полина", "светлана", "света",
    "софия", "софья", "соня", "таисия", "тамара", "татьяна", "таня",
    "ульяна", "юлия", "юля", "яна",
})

#: Вопрос об имени (В17, уточнение Шифу 21.10): не сразу — копить признаки; спрашивать не раньше N-й фразы
#: собеседника или T секунд разговора с ним (и не меньше двух фраз), один раз на ``person_ref``.
IDENTITY_ASK_MIN_PHRASES = 3
IDENTITY_ASK_MIN_S = 60.0
IDENTITY_ASK_MIN_PHRASES_BY_TIME = 2
#: Незнакомец без ``person_ref``: не чаще раза в 10 мин (§7.5 п.2).
IDENTITY_ASK_STRANGER_GAP_S = 600.0
#: Лицо с именем в кадре за эти секунды — собеседник опознан лицом, «Как тебя зовут?» не нужен.
IDENTITY_FACE_RECENT_S = 10.0
#: Бюджет времени хода (В5): истёк — отмена ожиданий и ``understand.timeout``.
TURN_DEADLINE_S = 12
#: Query-цикл (§4.1): не больше двух итераций данных → LLM на ход.
LLM_QUERY_ITERATIONS_MAX = 2
#: Невалидная команда LLM возвращается модели перечнем нарушений (tool-результат на её вызов ``command``) — столько
#: повторов на ответ; дальше невалидна — ход закрывается как ``failed`` без речи (рефлекс ``REFLEXES["failed"]``:
#: earcon fail, красное кольцо), следующий ход идёт обычно (решения Шифу 21.10 по #3698, ADR-0148 п.2).
COMMAND_RETRIES_MAX = 1
#: Причина нарушения «speech при непустых actions»: такая реплика не звучит (§4.1), ответ по сути пропал бы молча
#: (живой прогон #3698: 2 из 15). Это нарушение команды — модель получает повтор (решение Шифу 22.10, вариант 2).
VIOLATION_SPEECH_WITH_ACTIONS = "speech_with_actions"
#: Поле аргументов вызова ``command``, которым мост отдаёт исполнителю хода невалидную команду (значение — причина).
COMMAND_INVALID_FIELD = "invalid"

# ---------------------------------------------------------------------------------------------------------------
# Матрица одновременности (§5.6.2) — данные; читает ``plan.composition.compose``
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
#: Строка матрицы по классу действия (§5.1 → §5.6.2). Память, громкость, голос, знакомство ресурсов не занимают —
#: параллельно со всем, как ``query``; ``action.media`` — старт/смена музыки.
COMPOSITION_KIND_BY_CLASS: Mapping[str, str] = MappingProxyType({
    "query": "query", "action.local": "query", "action.identity": "query", "action.media": "music",
    "action.motion": "motion", "action.speak_aloud": "speech", "action.perform": "perform",
})
#: Тулы, чья строка не следует из класса: звук — SFX, анимация — экспрессия, тул речи — реплика.
COMPOSITION_KIND_BY_TOOL: Mapping[str, str] = MappingProxyType({
    "play_sound": "sfx", "play_animation": "expression", "speak_text": "speech",
})
#: Какой ресурс (столбец) занимает начатый блок строки; ``halt`` и ``query`` ничего не занимают. Подложку
#: выступления (``backing``) запускает выступление (PR-7c), а не строка ``music``.
COMPOSITION_OCCUPIES: Mapping[str, str] = MappingProxyType({
    "speech": "speech", "perform": "perform", "expression": "face", "sfx": "sfx", "music": "track", "motion": "motion",
})
#: Граница, на которой занятый ресурс можно сменить (§5.6.1, §5.6.4): реплика — конец предложения, выступление —
#: сегмента, подложка — такт, трек/сет — фраза (ADR-0149), остальное — сразу.
RESOURCE_BOUNDARY: Mapping[str, str] = MappingProxyType({
    "speech": "sentence", "perform": "segment", "face": "now", "sfx": "now", "backing": "bar", "track": "phrase",
    "motion": "now",
})
#: Столбцы без события конца (у ``ExpressionEvent`` только started/skipped): ждать их нечего, Tier-1 их не трогает.
COMPOSITION_NO_END: Tuple[str, ...] = ("face",)
#: Непрерываемое занятое (§5.6.2): движение вытесняет только «стой» и новое движение (строка таблицы), не «хватит».
COMPOSITION_UNINTERRUPTIBLE: Tuple[str, ...] = ("motion",)
#: Строки, которые Tier-1 «потом» не откладывает: «потом» — про просьбу («а потом включи музыку»), а не про ответ
#: робота — реплика, экспрессия и запрос данных хода звучат/идут сразу (реплика за репликой — очередь tts_node).
TIER1_LATER_SKIP_ROWS: Tuple[str, ...] = ("speech", "expression", "query")
#: Старшинство политик: итог по нескольким занятым ресурсам — самая строгая клетка.
POLICY_ORDER: Tuple[str, ...] = ("REJECT", "APPEND", "REPLACE", "MERGE")
#: Блок из события владельца (речь, SFX, движение) без события конца дольше этого срока считается потерянным
#: (finished не дошёл) — ресурс свободен, план не ждёт вечно. Музыка из снимка плеера не стареет: он переиздаётся.
RESOURCE_STALE_S = 180.0
#: Скорость речи для ETA ``queued{eta_ms}``, пока нет замера по ``SpeechEvent progress`` (spoken_chars/seconds).
#: Живой робот 21.10 23:20Z (minimax male-qn-qingse, SpeechEvent started→finished): 55/4,7 = 11,8; 64/6,1 = 10,5;
#: 41/4,0 = 10,2 симв/с — без времени синтеза (~2–3 с на реплику), его добавляет запас ожидания.
SPEECH_CHARS_PER_S_GUESS = 11.0
#: Ключи фраз плана (§5.6.4): «Поставлю после …» по ``PlanEvent queued`` — глагол по строке матрицы, «после чего» —
#: по столбцу; их обязан содержать ``phrases/ru.yaml`` (как :data:`PHRASE_KEYS`).
PLAN_PHRASE_KEYS: Tuple[str, ...] = (
    "plan.queued", "plan.queued.eta", "plan.timeout",
    *(f"plan.verb.{kind}" for kind in COMPOSITION_NEW if kind != "halt"),
    *(f"plan.after.{busy}" for busy in COMPOSITION_BUSY if busy not in COMPOSITION_NO_END),
)
#: Очереди, которые ведёт сам владелец ресурса (строка, столбец): реплика за репликой — tts_node, короткий звук следом —
#: sound_node (``SOUND_QUEUE_MAX_WAIT_S``, длинный — честный ``skipped{busy}``). Планировщик их не ждёт: запрос — сразу.
OWNER_QUEUES: Tuple[Tuple[str, str], ...] = (("speech", "speech"), ("sfx", "sfx"))
#: Потолок ожидания действия в очереди плана: оценка ETA занятого + запас, но не больше максимума; ETA неизвестна —
#: максимум. Пока владелец сообщает о занятом ресурсе (события, свежая ETA), срок продлевается до now + ETA + запас,
#: но не дальше максимума от начала ожидания. Не дождался — действие не исполняется, исход ``timeout`` с причиной
#: :data:`PLAN_WAIT_REASON`, и об этом говорит код (а не тихий старт поверх и не молчание).
PLAN_WAIT_MAX_S = 60.0
PLAN_WAIT_SLACK_S = 5.0
PLAN_WAIT_REASON = "plan_wait"


def plan_wait_ceiling_s(eta_s: Optional[float]) -> float:
    """Сколько ждать занятый ресурс при оценке ``eta_s`` (``None`` — неизвестно)."""
    return PLAN_WAIT_MAX_S if eta_s is None else min(PLAN_WAIT_MAX_S, eta_s + PLAN_WAIT_SLACK_S)


#: ETA короче этого во фразе не называется: «примерно через 1 с» — шум, а не информация.
PLAN_SAY_ETA_MIN_S = 3

# ---------------------------------------------------------------------------------------------------------------
# Команда LLM (§4.1): пределы и перечисления; схема и промпт генерируются ниже
# ---------------------------------------------------------------------------------------------------------------

SPEECH_MAX_CHARS = 280
#: Имя голоса реплики/сегмента (``Command.voice``, ``Performance.voice_per_segment``) - как ``speak_text.voice``.
VOICE_MAX_CHARS = 64
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


def face_schema() -> Dict[str, Any]:
    """JSON-схема сцены лица (§5.5.9): одна на ``Command.expression.face`` и ``play_animation(face=...)``."""
    face = {slot: _enum(values) for slot, values in FACE_SCENE_SLOTS.items()}
    face["duration_s"] = {"type": "number", "minimum": 0, "maximum": FACE_SCENE_MAX_S}
    schema = _object(face)
    schema["description"] = FACE_SCENE_HINT
    return schema


def _expression_schema() -> Dict[str, Any]:
    return _object({
        "emotion": _enum(EMOTIONS), "gesture": _enum(GESTURES), "earcon": _enum(EARCONS), "face": face_schema(),
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
        "voice": {"type": "string", "maxLength": VOICE_MAX_CHARS},
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
            "voice_per_segment": {"type": "array", "maxItems": SEGMENTS_MAX,
                                  "items": {"type": "string", "maxLength": VOICE_MAX_CHARS}},
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
    """Фрагмент системного промпта о форме ответа — из тех же таблиц, что схема (не вручную).

    Об итоге действий говорит код по событию исполнителя (§4.1, PR-7): ``speech`` при действиях — нарушение
    (``VIOLATION_SPEECH_WITH_ACTIONS``), ответ по сути рядом с действиями («включи звук и расскажи шутку») —
    ``perform``, он звучит всегда.
    """

    def _set(values: Sequence[str]) -> str:
        return "{" + ", ".join(values) + "}"

    face = "; ".join(f"{slot} ∈ {_set(v)}" for slot, v in FACE_SCENE_SLOTS.items())
    lines: List[str] = [
        "Ответ — одна команда (JSON-объект) с полями:",
        f"- speech: реплика человеку, ≤ {SPEECH_MAX_CHARS} символов. С actions speech не пиши (команда невалидна):"
        " о результате действий скажет код по событию исполнителя; ответ по сути (шутка, стих) — в perform.",
        "- voice: голос этой реплики (имя голоса, как у speak_text); голос робота не меняет.",
        f"- ask: вопрос с ожиданием ответа, expects ∈ {_set(ASK_EXPECTS)}.",
        f"- actions: ≤ {ACTIONS_MAX} вызовов тулов {{tool, args}}; не больше одного на класс"
        f" {' и '.join(SINGLE_ACTION_CLASSES)}.",
        f"- perform: выступление kind ∈ {_set(PERFORM_KINDS)}, {SEGMENTS_MIN}–{SEGMENTS_MAX} сегментов"
        f" по ≤ {SEGMENT_MAX_CHARS} символов, backing ∈ {_set(PERFORM_BACKINGS)}; вместо speech (speech при нём не"
        " пиши, даже с actions). Звучит и при"
        " actions: ответ по сути (сказка, стих, анекдот), не отчёт о действиях. voice_per_segment — голос на"
        " сегмент (сказка разными голосами); set_voice меняет голос робота насовсем — только когда об этом просят.",
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


def _wake_entry(item: Any, risky_enabled: bool) -> str:
    """Строка — вейк; ``{word, risky: true}`` — рискованный вейк (В18), берётся только при ``risky_enabled``."""
    if isinstance(item, str):
        return item.strip()
    if isinstance(item, dict) and isinstance(item.get("word"), str):
        return item["word"].strip() if (risky_enabled or not item.get("risky")) else ""
    return ""


def load_wake_word_namespaces(path: "str | os.PathLike[str] | None") -> Dict[str, List[str]]:
    """Прочитать ``wake_words.yaml`` в ``{namespace: [words]}``, порядок — как в файле.

    Записи ``{word: …, risky: true}`` входят, только если в файле ``risky_enabled: true`` (В18).
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
    risky_enabled = data.get("risky_enabled") is True
    for namespace in WAKE_NAMESPACES:
        words = data.get(namespace)
        if isinstance(words, list):
            cleaned = [w for w in (_wake_entry(item, risky_enabled) for item in words) if w]
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
