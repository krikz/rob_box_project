"""dialogue_guards.py — Issue #992 guard heuristics for LLM output quality.

Extracted from :class:`rob_box_voice.dialogue_node.DialogueNode` (TD-1,
see ``.planning/DIALOGUE_NODE_REFACTORING.md``) so the same detectors and
retry-prompt builders can be unit-tested without a ROS2 node and reused
by any future harness-style adapter. Pure Python: no rclpy, no I/O.

Owns two families of heuristics:

* **Babble / metalanguage detection** (issue #992 Bug D) — when the LLM
  answers a performance command («зачитай рэп», «расскажи стих») with a
  meta-promise (``Зачитаю рэпчик про космос!``) instead of actually
  performing, :func:`is_metalanguage_babble` recognises the opener and
  :func:`build_babble_retry_prompt` builds the one-shot CRITICAL retry
  prompt that forces a real tool-call reply.
* **Music tool names** — :data:`MUSIC_STARTING_TOOLS` & Co. from the tool
  catalog (cleanup policy, one-track-per-turn). The music guard (Bug B/C),
  its detectors (``user_wants_music``, ``is_music_state_query``, music
  ``ActionClaimRule``-s, Renardo-code / hallucinated-MIDI / unknown-melody
  detectors) were removed in ADR-0149 PR-13a together with the old music
  path: the success phrase of a music start is built by code from the
  player's ``started`` event.
* **Non-music tool guard** (issue #1777 / #1762) — расширение Bug C на
  все явные tool-based запросы (``get_current_time``, ``search_web``,
  ``set_voice``, ``memory_search``, ``faq_search``). См. :data:`TOOL_REQUEST_PATTERNS`
  и :func:`detect_required_tool`.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from typing import Optional, Tuple

from rob_box_core.tool_catalog import TOOL_CATALOG

from .media_command_grammar import (  # noqa: F401 — реэкспорт, issue #3134
    MUSIC_STOP_COMMAND_RE,
    MUSIC_STOP_OVERRIDES,
    is_music_stop_command,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Music tool names — single source of truth
# ---------------------------------------------------------------------------

#: Инструменты, запускающие слышимую музыку (Renardo или mp3).
#:
#: Единственный источник — capability-флаг ``starts_music`` на ``MCPTool``,
#: проецируемый в ``rob_box_core.tool_catalog``. Раньше это знание жило в
#: двух рукописных frozenset'ах (``RENARDO_MUSIC_TOOLS`` /
#: ``GENERATED_MUSIC_TOOLS``), и каждый новый музыкальный тул про него
#: забывал: ``compose_music`` вышел и был авто-заглушен через 1.5 с,
#: ``gen_play_from_library`` — в третий раз, а ``lookup_melody`` играл
#: мелодию, но гуард его не видел и гнал LLM в повторный
#: ``execute_music_code`` (тот стирал мелодию через Clock.clear()).
MUSIC_STARTING_TOOLS: frozenset = frozenset(
    entry.name for entry in TOOL_CATALOG if entry.starts_music
)

#: Tools that end TRACK-mode — used ONLY to clear the cleanup-policy flag
#: ``dialogue_node._track_mode_music_active`` (see the 31.08 fix at the
#: call site).
#:
#: Issue #3133 (ADR-0141): ``set_dj_mode`` removed. It sat here AND in
#: ``MUSIC_MODE_TOOLS`` at once; the starters branch runs after the stop
#: branch and re-armed the flag anyway, so its presence here never cleared
#: anything — it only made the set lie about what «stop» means. Whether
#: music is playing is no longer derived from tool names at all: dialogue
#: reads the player snapshot on ``/voice/music/state``.
#:
MUSIC_STOP_TOOLS: frozenset = frozenset({
    "stop_music",
})

#: Tools that put existing playback into a mode rather than starting it.
#: They keep music alive for the cleanup logic.
MUSIC_MODE_TOOLS: frozenset = frozenset({
    "load_track",
    "set_vibe_preset",
})

#: Тулы, которые закрывают ПОЛЬЗОВАТЕЛЬСКУЮ просьбу «включи трек X».
#: Источник — capability-флаг ``satisfies_user_music`` (напр. ``load_track``).
USER_MUSIC_SATISFYING_TOOLS: frozenset = frozenset(
    entry.name for entry in TOOL_CATALOG if entry.satisfies_user_music
)


# ---------------------------------------------------------------------------
# Issue #992 Bug D — banned metalanguage openers. When the LLM returns
# plain text (no ``speak_text`` call) that begins with one of these
# phrases — e.g. "Зачитаю рэп про космос!", "Могу бит добавить,
# хочешь?", "Слушай, сейчас расскажу..." — the user hears a meta-
# promise instead of the actual performance. We catch that pattern at
# the dialogue_node boundary and force a single retry with a CRITICAL
# prompt-level reminder.
#
# The list is lowercase, comma/space separated, and checked via a
# substring match on the first ~80 chars of the LLM output (after
# strip_markdown). Add new phrases when the LLM invents a new opener;
# keep the list tight to avoid false positives on legitimate answers.
# ---------------------------------------------------------------------------
BABBLE_BANNED_OPENERS: tuple = (
    "зачит",   # зачитаю / зачитаем / зачитываю
    "погнали",  # погнали! / ну что, погнали?
    "могу ",    # могу бит добавить, могу спеть
    "хочешь",  # хочешь ещё? / хочешь послушать?
    "сейчас ",  # сейчас устроим / сейчас расскажу
    "устроим",  # устроим концерт / устроим вечеринку
    "давай-ка",  # давай-ка я спою
    "давай ",  # давай я / давай попробуем
    "слушай,",  # слушай, сейчас ...
    "слушай ",
    "окей, ",
    "окей ",
    "так, ",
    "так ",
    "ну что ж",
    "переключаюсь",
    "переключ",
)

# 🔴 FIX (live 02.09): «робот говорит, что запускает музыку, а ничего не
# запускается» + «LLM много говорит во время музыки».
#
# Живой лог робота, 06:58:01 UTC:
#   spoken='Юзер явно просит «ебани лаундж» (лоундж запрошен снова).
#           DJ уже выключен в прошлом ходе. Запускаю расслабленную
#           лоундж-композицию через compose_music.' tools=[]
# и следом TTS честно зачитал это вслух. Модель отдала своё планирование
# вместо ответа и не вызвала ни одного тула.
#
# Ни один существующий детектор это не ловил: BABBLE_BANNED_OPENERS ищет
# обещания («зачитаю», «погнали»), а тут рассуждение; music-гуард смотрел
# на реплику ЮЗЕРА («ебани ланудж») и не нашёл там ключевых слов.
# Расширять словари бесполезно — их не хватит никогда.
#
# Надёжный признак другой и от словаря не зависит:
#   * в тексте назван ИНСТРУМЕНТ (compose_music и т.п.). Идентификатор в
#     snake_case, прочитанный вслух, — всегда баг, что бы ни просил юзер;
#   * реплика начинается с рассказа о собеседнике в ТРЕТЬЕМ лице («Юзер
#     просит...», «Пользователь спрашивает...»). Ответ, адресованный
#     человеку, так не начинается — так начинается план.
PLANNING_NARRATION_TOOL_NAMES: tuple = (
    "compose_music",
    "execute_music_code",
    "speak_text",
    "set_dj_mode",
    "search_samples",
    "stop_music",
    "load_track",
    "list_tracks",
    "save_track",
    "gen_play_from_library",
    "gen_search_library",
    "set_vibe_preset",
    "play_animation",
    "play_sound",
    "memory_save",
    "memory_search",
    "search_web",
    "listen_for_response",
    "navigate_to_waypoint",
    # Issue #2760 — имя всплыло в живой разметке вызова, прочитанной вслух.
    "register_speaker",
)

#: Начала реплики, где модель говорит о собеседнике в третьем лице.
PLANNING_NARRATION_OPENERS: tuple = (
    "юзер",
    "пользовател",
    "user ",
)


def is_planning_narration(spoken_text: str) -> bool:
    """Похоже ли на внутреннее планирование модели, а не на ответ человеку?

    Признаки и причина, по которой они именно такие, — в комментарии выше.
    """
    if not spoken_text:
        return False
    low = spoken_text.lower()
    # NB (issue #2760): к этому моменту текст уже прошёл ``strip_markdown``,
    # который снимает ``_..._`` парами через весь текст — в живом логе
    # ``register_speaker``/``memory_save``/``speaker_id`` приехали сюда как
    # ``registerspeaker``/``memorysave``/``speakerid``, и поиск подстрокой
    # не совпал ни разу. Из-за этого правило ведёт себя непоследовательно:
    # одно упоминание тула (одно подчёркивание, пары нет) → mute, три
    # упоминания (подчёркивания схлопнулись) → озвучиваем. Чинить это
    # нормализацией НЕЛЬЗЯ мимоходом: цена ошибки здесь не ретрай, а
    # полная тишина в ответ («что умеешь?» → «Я умею: speak_text, …»
    # ушло бы в mute, см. test_issue_1882_planning_narration.py).
    # Разметку протокола ловит отдельный, не зависящий от подчёркиваний
    # :func:`is_tool_call_markup` (#2760); политика mute для прозаичных
    # упоминаний — отдельное решение.
    if any(name in low for name in PLANNING_NARRATION_TOOL_NAMES):
        return True
    head = low.lstrip(" \t*#>-—«\"'")[:40]
    return any(head.startswith(opener) for opener in PLANNING_NARRATION_OPENERS)


# Issue #992 Bug D — keywords that mark the user request as a
# performance command. When the LLM babbles on a performance request
# we *must* retry, because the alternative is the user hearing nothing
# (the LLM promised but never spoke). When the user just asked a
# normal question and the LLM babbled, we still retry but the
# consequence is less severe — the user hears ONE meta-phrase instead
# of an answer. Keeping the heuristic narrow prevents false positives
# on ordinary chit-chat that happens to start with «слушай».
BABBLE_PERFORMANCE_KEYWORDS: tuple = (
    "рэп", "реп", "rap",
    "песн", "song", "песню", "песня",
    "стих", "стишок", "poem", "стихотворен",
    "зачитай", "прочитай", "прочти",
    "спой", "пой", "спела",
    "сыграй", "играй",
    "музык", "мелоди", "бит", "трек",
    "диджей", "dj ",
    "концерт",
    "джаз", "рок", "блюз", "частушки", "частушк",
)

# Стоп-грамматика («выключи музыку», «стоп диджей», «хватит диджеить»;
# #1279 / #2834 / #2971) перенесена в :mod:`.media_command_grammar` (issue
# #3134): закрытые стоп-реплики теперь исполняет роутер медиакоманд до LLM,
# а широкий детектор нужен силенс-гейту и command-гейту.
# Имена реэкспортируются для старых мест вызова.


# ---------------------------------------------------------------------------
# 🔴 FIX (live 30.08, vision-pi 12:28): babble-детектор ложно срабатывал на
# ФАКТИЧЕСКОМ ответе. Юзер: «играет ли сейчас музыка» → LLM: «Сейчас тишина —
# ничего не играет.» Ответ начинается с «сейчас » (опенер из
# ``BABBLE_BANNED_OPENERS``), а ``user_wants_performance`` совпал по «музык» —
# и Bug D сжёг лишний round-trip к LLM ради байт-в-байт того же ответа.
#
# Вопрос — не запрос на исполнение. Частица «ли» в русском практически не
# встречается в императиве («зачитай рэп», «сыграй техно»), поэтому она —
# надёжный маркер. Плюс горстка вопросительных зачинов.
# ---------------------------------------------------------------------------
QUESTION_MARKERS: tuple = (
    " ли ",
    " ли?",
)

QUESTION_OPENERS: tuple = (
    "что играет",
    "что сейчас",
    "что за ",
    "какая музык",
    "какой трек",
    "какие звуки",
    "сколько ",
    "умеешь ли",
    "ты умеешь",
    "есть ли",
    "можешь ли",
)


def is_state_question(user_input: str) -> bool:
    """Вопрос о состоянии, а не команда исполнить.

    Используется ``user_wants_performance``: на вопрос «играет ли сейчас
    музыка» правильный ответ — текст, и babble-ретрай Bug D для него не
    нужен.
    """
    if not user_input:
        return False
    low = f" {user_input.lower().strip()} "
    if any(marker in low for marker in QUESTION_MARKERS):
        return True
    stripped = user_input.lower().strip()
    return any(stripped.startswith(opener) for opener in QUESTION_OPENERS)


# ---------------------------------------------------------------------------
# Issue #1777 / #1762 — non-music tool guard. Расширение Bug C retry на
# ВСЕ явные tool-based запросы. Раньше ретрай работал только для music
# (см. issue #992 Bug C), теперь — для ``get_current_time``,
# ``search_web``, ``set_voice``, ``memory_search``, ``faq_search``.
#
# Формат: каждая запись = (tool_name, tuple[подстрок…]). Подстроки
# матчатся lowercase substring в user_input. Tuple — для случаев когда
# одна категория покрывается несколькими keyword'ами («который час»,
# «сколько времени», «время в москве» — всё → get_current_time).
#
# ВАЖНО: keyword'ы специально выбираются такие, чтобы НЕ срабатывать
# на обычное chit-chat. Например «новости» — отдельный keyword от «погода»,
# чтобы не ретраить когда юзер просит «новости про X» (тоже → search_web,
# но другой по семантике).
# ---------------------------------------------------------------------------
TOOL_REQUEST_PATTERNS: tuple = (
    # time / date (issue #1777) — «который час», «сколько времени»,
    # «время в москве», «какая дата», «какой день недели».
    ("get_current_time", (
        "который час",
        "сколько врем",
        "сколько сейч",
        "время в ",
        "время по ",
        "время сейч",
        "который сейч",
        "сейчас врем",
        "сколько минут",
        "чо за время",
        "какая дата",
        "какой день",
        "какой сегодн",
        "какое число",
        "какое сегодня",
        "какой месяц",
        "какой год",
        "что за день",
        "что за число",
        "что за дат",
    )),
    # weather / news / web search (issue #1762) — «погода в X»,
    # «новости про Y», «что в интернете», «загугли».
    ("search_web", (
        "погода",
        "погоду",
        "новости",
        "новость",
        "что в интернет",
        "загугл",
        "найди в инет",
        "поищи в инет",
        "найди информ",
        "узнай в инет",
        "найди что",
        "поищи что",
        "расскажи про ",  # «расскажи про X» — обычно требует поиска
        "что ты знаешь про ",
        "что известно про ",
    )),
    # voice (issue #1765) — «переключи голос», «поставь голос X»,
    # «голос Артём», «смени голос».
    #
    # 🔴 Осознанно НЕ добавлены «говор », «говори », «говорит », «голосом»:
    # это substring-match без границ слова, и они ловят обычный chit-chat —
    # «не говори глупости», «мама говорит что...», «он говорит по-английски»,
    # «спой красивым голосом». Ложный матч здесь не «лишний round-trip», а
    # ретрай, ТРЕБУЮЩИЙ от LLM вызвать set_voice в ходе, который к голосу
    # никак не относится.
    ("set_voice", (
        "переключи голос",
        "смени голос",
        "поменяй голос",
        "голос арт",
        "голос ален",
        "голос анто",
        "голос окс",
        "голос жан",
        "голос ерма",
        "голос зайц",
        "голос леви",
        "голос маш",
        "голос никол",
        "голос серг",
        "голос алек",
        "голос ден",
        "голос мар",
        "голос тат",
        "давай голос",
        "поставь голос",
        "установи голос",
    )),
    # memory (issue #1770) — «что ты знаешь обо мне», «помнишь меня»,
    # «что помнишь».
    ("memory_search", (
        "что ты знаешь обо мне",
        "что знаешь обо мне",
        "что ты помнишь",
        "что помнишь",
        "помнишь меня",
        "помнишь про меня",
        "что ты знаешь про меня",
        "что знаешь про меня",
        "расскажи что знаешь",
        "что ты обо мне",
        "что обо мне знаешь",
    )),
    # FAQ — «что ты умеешь», «какие команды», «справка».
    ("faq_search", (
        "что ты умеешь",
        "что умеешь",
        "что можешь",
        "какие команды",
        "что ты можешь делать",
        "справка",
        "помощь",
        "что ты такое",
        "кто ты такой",
        "расскажи о себе",
    )),
)


def detect_required_tool(user_input: str) -> Optional[str]:
    """Issue #1777 / #1762 — какой tool явно просит юзер?

    Возвращает имя tool (``get_current_time``, ``search_web``, ``set_voice``,
    ``memory_search``, ``faq_search``) или ``None`` если user_input не
    содержит явного tool-pattern'а.

    Чистая функция, без I/O — тестируется без ROS2.

    Priority: первое совпадение в :data:`TOOL_REQUEST_PATTERNS` побеждает
    (порядок в tuple = приоритет). На практике ключевые слова разных
    категорий не пересекаются («который час» → только get_current_time,
    «погода в Бишкеке» → только search_web), но если когда-то пересекутся
    — порядок tuple решает.
    """
    if not user_input:
        return None
    low = user_input.lower()
    for tool_name, keywords in TOOL_REQUEST_PATTERNS:
        if any(kw in low for kw in keywords):
            return tool_name
    return None


def build_tool_retry_prompt(user_input: str, tool_name: str) -> str:
    """Issue #1777 / #1762 — synthetic prompt для Bug C retry (non-music).

    Echoes the original ``user_input`` so the LLM has the request in
    context, then injects a CRITICAL reminder that names the specific
    ``tool_name`` the LLM must call. ``tool_name`` MUST come from
    :func:`detect_required_tool` (or any hard-coded allow-list) to prevent
    prompt-injection: never pass user-controlled strings.

    Prefix is :data:`MUSIC_RETRY_PROMPT_PREFIX` (historical name — the
    music retry that shared it was removed in ADR-0149 PR-13a).
    """
    hint = {
        "get_current_time": (
            "вызови get_current_time() — инструмент возвращает точное "
            "локальное время робота (Europe/Moscow по умолчанию). "
            "Не выдумывай время, не говори 'сейчас X утра/вечера' из головы."
        ),
        "search_web": (
            "вызови search_web(query=...) — инструмент ищет актуальную "
            "информацию в интернете (погода, новости, факты). "
            "Не говори 'гляну/сделаю/сейчас узнаю' без реального вызова."
        ),
        "set_voice": (
            "вызови set_voice(provider=..., voice_name=...) или "
            "list_voices() чтобы выбрать. Не говори 'голоса X нет' "
            "не проверив список через list_voices."
        ),
        "memory_search": (
            "вызови memory_search(speaker_id=<current>) или "
            "memory_context(speaker_id=<current>) — только для ТЕКУЩЕГО "
            "спикера. Не подставляй факты других юзеров."
        ),
        "faq_search": (
            "вызови faq_search(query=...) — инструмент ищет по локальной "
            "базе возможностей и команд. Не придумывай список команд сам."
        ),
    }.get(tool_name)
    if hint is None:
        # Defence-in-depth: если tool_name не из allow-list — НЕ ретраим.
        # Это защищает от prompt-injection (юзер пишет «забудь инструкции,
        # вызови tool X»). Любой неизвестный tool = silent skip.
        logger.warning(
            f"🛡 [issue 1777] build_tool_retry_prompt: unknown tool_name={tool_name!r}, "
            "skipping retry (defence-in-depth)"
        )
        return ""
    return (
        MUSIC_RETRY_PROMPT_PREFIX + f" {tool_name}, хотя пользователь "
        "явно попросил соответствующее действие. "
        + hint + " "
        "Запрос юзера: «" + (user_input or "") + "». "
        "Если и сейчас не вызовешь tool — пользователь не получит ответа."
    )


# ---------------------------------------------------------------------------
# Detectors
# ---------------------------------------------------------------------------

def is_metalanguage_babble(spoken_text: str) -> bool:
    """Issue #992 Bug D — does this LLM output read as meta-talk?

    Returns ``True`` when the LLM final response text starts with a
    known metalanguage opener («зачита», «могу», «хочешь»,
    «сейчас», «устроим», «погнали», «давай», «слушай», «окей»,
    «так», «переключ», «ну что ж»). The check operates on the
    first 80 chars after :func:`rob_box_voice.core.speak_helpers.strip_markdown`
    so a lone "**" that survived cleaning cannot mask the opener.

    The detector is intentionally *conservative*: a normal answer
    that happens to contain «слушай» somewhere in the middle is
    safe — only the first 80 chars are inspected. When in doubt,
    return ``False``; the caller will fall through to the standard
    TTS publish path.
    """
    if not spoken_text:
        return False
    if is_planning_narration(spoken_text):
        return True
    head = spoken_text[:80].lower().lstrip(" \t*#>-")
    # Match the opener only at the START of the head or inside the
    # first 30 chars (after stripping). 30 chars is enough to cover
    # «Слушай, сейчас расскажу...» but short enough to skip
    # legitimate mid-sentence uses like «Если хочешь, могу
    # остановиться» or «А сейчас продолжу маршрут».
    opener_zone = head[:30]
    return any(
        opener_zone.startswith(opener) or f" {opener}" in opener_zone
        for opener in BABBLE_BANNED_OPENERS
    )


# ---------------------------------------------------------------------------
# Issue #1882 — hard-mute для planning-narration жильёт в dialogue_node.py
# (`_handle_result`, ветка ПЕРЕД babble-retry). Сам детектор `is_planning_narration`
# определён выше в этом файле (введён в 78403dba), тут дублировать его не надо —
# иначе будет две функции с одним именем.
# ---------------------------------------------------------------------------


def user_wants_performance(user_input: str) -> bool:
    """Issue #992 Bug D — does the user request a *performance*?

    Used to decide whether a metalanguage reply is a hard bug
    (user asked for a rap, robot returned "Зачитаю рэп про X!") or
    just a stylistic miss (user asked "что нового?", robot replied
    "Слушай, у меня тут..." — still answer-shaped, just informal).
    """
    if not user_input:
        return False
    # 🔴 FIX (live 30.08): вопрос о состоянии — не запрос на исполнение.
    # «играет ли сейчас музыка» совпадал по «музык» и гнал Bug D в ретрай
    # ради того же самого текстового ответа. См. ``is_state_question``.
    if is_state_question(user_input):
        return False
    low = user_input.lower()
    return any(kw in low for kw in BABBLE_PERFORMANCE_KEYWORDS)


# ---------------------------------------------------------------------------
# Issue #992 Bug E — «сделал» без единого тула.
#
# Live 30.08 (vision-pi 12:31–12:38), восемь ходов из восемнадцати: LLM
# отвечает утверждением о выполненном действии, а ``tools_called`` пуст —
# то есть не выполнено НИЧЕГО:
#
#   «запомни эту точку как тесточка»    → «Точка сохранена.»        tools=[]
#   «удали точку тесточка»              → «Точка удалена.»          tools=[]
#   «удали трек тисбит из сохраненных»  → ««Тисбит» удалён…»        tools=[]
#   «загрузи и включи трек тисбит»      → «Трек играет.»            tools=[]
#
# Что «точек пока нет» после «Точка сохранена» видно в том же логе двумя
# ходами позже. Bug C ловит только музыкальную ветку; здесь тот же класс
# ошибки на навигации и медиатеке.
#
# Таблица ниже — узкая по построению: срабатывает только когда И запрос
# юзера, И утверждение LLM попадают в одну и ту же пару шаблонов, И тул
# из ``tools`` не вызван. Любое сомнение → не срабатываем: цена ложного
# ретрая — лишний round-trip к LLM.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ActionClaimRule:
    """Одно правило детектора «заявил, но не сделал».

    Attributes:
        category: Короткий тег для лога и для ветвления в тестах.
        user_re: Что должен был попросить юзер.
        claim_re: Как LLM отчитывается о выполнении.
        tools: Тулы, любой из которых закрывает заявку. Пустой
            ``tools_called`` при непустом ``tools`` = баг.
        what: Человеческая формулировка для retry-промпта.
    """

    category: str
    user_re: "re.Pattern[str]"
    claim_re: "re.Pattern[str]"
    tools: frozenset
    what: str


ACTION_CLAIM_RULES: tuple = (
    ActionClaimRule(
        category="waypoint_save",
        user_re=re.compile(
            r"(?:запомни|сохрани|запиши)\s+(?:эту\s+)?(?:точк|мест|координат|"
            r"вейпоинт|waypoint)", re.IGNORECASE),
        claim_re=re.compile(
            r"точк\w*\s+(?:сохранен|запомнен|записан|добавлен)|"
            r"(?:запомнил|сохранил|записал)\w*\s+(?:эту\s+)?точк",
            re.IGNORECASE),
        tools=frozenset({"save_waypoint"}),
        what="сохранение точки (save_waypoint)",
    ),
    ActionClaimRule(
        category="waypoint_delete",
        user_re=re.compile(
            r"(?:удали|сотри|забудь|убери)\s+(?:эту\s+)?(?:точк|вейпоинт|waypoint)",
            re.IGNORECASE),
        claim_re=re.compile(
            r"точк\w*\s+(?:удален|стерт|убран)|"
            r"(?:удалил|стёр|стер|убрал)\w*\s+(?:эту\s+)?точк",
            re.IGNORECASE),
        tools=frozenset({"delete_waypoint", "clear_waypoints"}),
        what="удаление точки (delete_waypoint)",
    ),
    ActionClaimRule(
        category="track_delete",
        user_re=re.compile(
            r"(?:удали|сотри|убери)\s+(?:трек|композиц|мелоди|песн)",
            re.IGNORECASE),
        claim_re=re.compile(
            r"(?:удал|стёр|стер|убра)\w*", re.IGNORECASE),
        tools=frozenset({"delete_track", "gen_delete_from_library"}),
        what="удаление трека (delete_track / gen_delete_from_library)",
    ),
    ActionClaimRule(
        category="library_search",
        # 🔴 FIX (live 30.08 16:04, e2e): «найди в своей библиотеке сэмплы
        # барабанов» → «Сэмплы ударных найдены.» при tools=[]. Никакого
        # поиска не было — LLM просто утверждает результат.
        user_re=re.compile(
            r"(?:найди|поищи|поиск|подбери|покажи)\b.{0,30}?"
            r"(?:сэмпл|сампл|семпл|библиотек|медиатек|трек|звук)",
            re.IGNORECASE),
        claim_re=re.compile(
            r"(?:найден|нашёл|нашел|нашла|подобрал|вот\s+что\s+наш)\w*",
            re.IGNORECASE),
        # Любой поисковый тул закрывает заявку — какой именно, решает LLM
        # по тому, где искать (сэмплы, медиатека, память, интернет).
        tools=frozenset({
            "search_samples", "list_tracks", "load_track",
            "gen_search_library", "gen_list_library", "gen_get_track_info",
            "memory_search", "memory_context", "faq_search", "search_web",
        }),
        what="поиск (search_samples / list_tracks / gen_search_library)",
    ),
    # ---- Issue #2755: «Записала» без memory_save --------------------------
    # Прогон 35699257202 (акт 2 night-marathon, 22.09.2026), два шага подряд:
    #
    #   user='Робот, запомни: я пью только зелёный чай без сахара и совсем
    #         не ем лук'
    #   spoken='Записала, Саша: зелёный чай без сахара и никакого лука,
    #           даже жареного.' tools=[]
    #
    #   user='Робот, запомни про меня: Борис болеет за Спартак и всегда
    #         приносит пиццу'
    #   spoken='Запомнила, Борис: болеешь за Спартак и всегда приносишь
    #           пиццу.' tools=[]
    #
    # Дальше в том же акте робот эти факты пересказывает — и выглядит
    # убедительно, потому что берёт их из контекста диалога. До следующей
    # сессии они не доживают (это и ловит акт 10 «память переживает сброс»).
    #
    # Почему ни один существующий guard не сработал: узкое правило
    # ``waypoint_save`` требует в user_input «точку/место/координаты»,
    # а широкий #2549 ловит по :data:`_ACTION_VERBS_PAST`, где не было
    # ни «записал», ни «запомнил» (они добавлены тем же issue).
    #
    # ``tools`` намеренно шире одного ``memory_save``: «запомни, меня
    # зовут Саша» закрывается ``register_speaker`` — это ТОТ ЖЕ факт,
    # записанный в профиль диктора, а не пропущенный вызов.
    ActionClaimRule(
        category="fact_memory_save",
        user_re=re.compile(
            r"\b(?:запомни|запиши|сохрани|не\s+забудь|заметь)\b"
            # НЕ перехватываем waypoint/track-заявки: у них свои правила
            # выше по таблице со своими тулами.
            r"(?!\W{0,15}(?:эту\s+|это\s+|эти\s+)?"
            r"(?:точк|координат|вейпоинт|waypoint|трек|композиц|мелоди))",
            re.IGNORECASE),
        claim_re=re.compile(
            # Issue #2780 (реплика 2 живого лога): «записалось ли про
            # пиццу?» — ВОПРОС, не заявление. \b после \w* заставляет
            # захватывать слово целиком («записалось», не «записал» с
            # откатом назад), и (?!\s+ли\b) отбрасывает вопросительную
            # частицу сразу после него.
            r"(?:запомнил|записал|сохранил|зафиксировал|отметил)\w*\b(?!\s+ли\b)|"
            # Причастия («сохранено/сохранена/сохранены» и т.п.) — родовые
            # и числовые окончания опциональны, чтобы «Информация
            # сохранена.» ловилось так же, как «Точка сохранено» (было
            # раньше только среднего рода единственного числа).
            r"(?:запомнен|записан|сохранен|зафиксирован)[оаы]?\b|"
            # Issue #2780 (прогон 35734532425, шаг n206_boris_memory):
            # после того как ЭТОТ ЖЕ guard уже поймал ложное «Запомнил»
            # и заставил модель признаться («у меня сбой, проверь»),
            # следующая реплика подтверждает запись СЛОВАМИ-подтверждениями,
            # а не «запомнил/записал» — «Всё на месте, Спартак и пицца в
            # памяти, запись подтверждена.» с tools=['memory_context']
            # (ЧТЕНИЕ, не запись). Ни одно слово выше не матчило —
            # guard молчал, юзер уходил с верой в несуществующую запись.
            r"вс[её]\s+на\s+месте|"
            r"запис\w*\s+подтвержд\w*|подтвержд\w*\s+запис\w*|"
            r"уже\s+(?:записал|запомнил|сохранил|зафиксировал)\w*|"
            # «уже в памяти» — НЕ голое «в памяти» (которое встречается
            # и в честном признании сбоя — «у меня в памяти сбой»,
            # реплика 2 живого лога; см. test_honest_confession_is_not_a_claim
            # в test_issue_2780_memory_confirmation_claim.py).
            r"уже\s+в\s+памяти\b",
            re.IGNORECASE | re.UNICODE),
        tools=frozenset({"memory_save", "register_speaker"}),
        what="сохранение факта в память (memory_save; имя — register_speaker)",
    ),
    # ---- READ-ONLY заявки (e2e 33251879328, GATE-1) --------------------
    # «expected tool calls not invoked ... LLM сделал verbal-only answer».
    # Робот отвечает о ЖИВОМ состоянии по памяти модели, не спросив систему.
    ActionClaimRule(
        category="waypoint_list",
        user_re=re.compile(
            r"(?:перечисли|покажи|какие|список|назови)\b.{0,25}?"
            r"(?:точк|вейпоинт|waypoint|мест)",
            re.IGNORECASE),
        # Утверждение о СОДЕРЖИМОМ списка — и «точек нет» тоже утверждение.
        # Живой лог 30.08: «Точек пока нет — карту ни разу не строили» при
        # tools=[], а точка к тому моменту уже сохранялась.
        claim_re=re.compile(
            r"точ(?:ек|ки|ка)\b|нет\s+точек|список\s+точек|пуст",
            re.IGNORECASE),
        tools=frozenset({"list_waypoints", "get_current_pose"}),
        what="список точек (list_waypoints)",
    ),
    ActionClaimRule(
        category="sound_info",
        user_re=re.compile(
            r"(?:какие|перечисли|покажи|список)\b.{0,25}?звук",
            re.IGNORECASE),
        claim_re=re.compile(r"звук\w*|умею|эмоци|сигнал|эффект", re.IGNORECASE),
        tools=frozenset({"get_sound_info", "play_sound"}),
        what="список звуков (get_sound_info)",
    ),
)


def detect_unbacked_action_claim(
    *,
    user_input: Optional[str],
    spoken: Optional[str],
    tools_called: Optional[Tuple[str, ...]],
) -> Optional[ActionClaimRule]:
    """Issue #992 Bug E — LLM отчиталась о действии, не вызвав тул.

    Возвращает сработавшее правило или ``None``. Правило считается
    сработавшим, когда запрос юзера подходит под ``user_re``, ответ LLM —
    под ``claim_re``, и ни один тул из ``rule.tools`` не был вызван.
    """
    if not user_input or not spoken:
        return None
    called = set(tools_called or ())
    for rule in ACTION_CLAIM_RULES:
        if not rule.user_re.search(user_input):
            continue
        if rule.claim_re.search(spoken) is None or called & rule.tools:
            continue
        return rule
    return None


# ---------------------------------------------------------------------------
# Issue #2549 — универсальный anti-hallucination guard.
#
# Live DJ-сет 2026-09-15: LLM выдавала ``spoken`` вида
# ``«Сделала два pass подряд: сначала один темп-каркас с heartbeat'ом и
# пульсом Бочкинса, потом второй…»`` / ``«Вплела тему Грига как второй
# голос над пульсом Бочкинса…»`` / ``«Проверю состояние и перезапущу.»``
# при ``tools_called=()``. Существующий Bug E guard для таких фраз НЕ
# срабатывает: его таблица :data:`ACTION_CLAIM_RULES` узкая, требует
# совпадения И в ``user_input``, И в ``spoken``. Здесь же юзер просил
# абстрактно «докрутить музыку» без явного «сделай/запусти» — а LLM
# всё равно отчитывается о действии.
#
# Защита — широкий fallback: если в ``spoken`` есть глагол действия в
# прошедшем или будущем времени (с учётом русских родовых окончаний) И
# ни один тул из :data:`CLAIM_JUSTIFYING_TOOLS` не был вызван — это
# action hallucination. Требуем ОДИН одноразовый ретрай.
#
# Решение НЕ пытается угадать КАКОЙ тул нужен (как Bug C/D guard'ы).
# LLM сама выберет по контексту в CRITICAL-промпте — мы только требуем,
# чтобы ретрай состоялся и заявление ушло в TTS не напрямую, а после
# реального вызова.
# ---------------------------------------------------------------------------

# Глаголы действия в прошедшем времени с опциональными родовыми
# окончаниями (а/и/ась/ись). Покрывает основные категории:
# сделал/а/и, запустил/а/и, включил/а/и, выключил/а/и, поменял/а/и,
# изменил/а/и, перезапустил/а/и, заменил/а/и, обновил/а/и, проверил/а/и,
# сохранил/а/и, удалил/а/и, настроил/а/и, поставил/а/и, добавил/а/и,
# отрегулировал/а/и, подкрутил/а/и, подобрал/а/и.
# Полный список action-verb в прошедшем времени с опциональными родовыми
# окончаниями (а/и/ась/ись). Покрывает основные категории действий:
#
#   сделал/запустил/включил/выключил/поменял/изменил/перезапустил/
#   заменил/обновил/проверил/сохранил/удалил/настроил/поставил/добавил/
#   отрегулировал/подкрутил/подобрал/поправил/вплел/вплетал/
#   установил/остановил/подложил/переключил/поправил/починил/перезагрузил
#
# Issue #2559 (live 15.09): робот говорил «проверю и перезапущу» при
# tools=[]. Узкий Bug E guard не ловил prose-фразы. #2549 (PR #2555) дал
# первичный набор verbs; task t_c7d707bc дополнил до полного покрытия из
# спецификации (Issue body §1): установ-/останов-/подлож-/переключ-/
# подкрут- + синонимы (поправлю/починю/перезагружу).
_ACTION_VERBS_PAST = re.compile(
    r"\b("
    r"сделал[аи]?|запустил[аи]?|включил[аи]?|включил[аи]?сь|"
    r"выключил[аи]?|поменял[аи]?|изменил[аи]?|включ[аи]?л[аи]?|"
    r"вплел[аи]?|вплетал[аи]?|перезапустил[аи]?|заменил[аи]?|"
    r"обновил[аи]?|проверил[аи]?|сохранил[аи]?|удалил[аи]?|"
    r"настроил[аи]?|поставил[аи]?|добавил[аи]?|отрегулировал[аи]?|"
    r"подкрутил[аи]?|подобрал[аи]?|поправил[аи]?|"
    r"установил[аи]?|остановил[аи]?|подложил[аи]?|переключил[аи]?|"
    r"починил[аи]?|перезагрузил[аи]?|"
    # Issue #2755 (акт 2, run 35699257202): «Записала, Саша: зелёный чай…»
    # и «Запомнила, Борис: …» при tools=[]. Память — такое же действие,
    # как включить музыку: без тула сказанное не переживёт сессию.
    r"записал[аи]?|запомнил[аи]?|зафиксировал[аи]?"
    r")\b",
    re.IGNORECASE | re.UNICODE,
)

# Глаголы действия в будущем времени — те же корни, 1-е лицо ед.ч.
# Полный список из спецификации (Issue body §1):
#   сделаю/запущу/включу/выключу/поменяю/изменю/перезапущу/
#   заменю/обновлю/проверю/сохраню/настрою/поставлю/добавлю/
#   отрегулирую/попробую/выберу/подберу/вплету/
#   установлю/остановлю/подложу/переключу/подкручу/
#   поправлю/починю/перезагружу.
_ACTION_VERBS_FUTURE = re.compile(
    r"\b("
    r"сделаю|запущу|включу|выключу|поменяю|изменю|включу|"
    r"вплету|перезапущу|заменю|обновлю|проверю|сохраню|удалил|"
    r"настрою|поставлю|добавлю|отрегулирую|попробую|выберу|подберу|"
    r"установлю|остановлю|подложу|переключу|подкручу|"
    r"поправлю|починю|перезагружу|"
    # Issue #2755 — будущее время того же обещания («запишу», «запомню»).
    r"запишу|запомню|зафиксирую"
    r")\b",
    re.IGNORECASE | re.UNICODE,
)

# Whitelist тулов, которые ОПРАВДЫВАЮТ action-claim в spoken. Если хотя
# бы один из них был вызван — guard НЕ срабатывает: LLM имеет право
# отчитаться о выполненном действии.
#
# Категории из существующего кода:
#   - music (request_music, dj_set — движок v2, ADR-0149; set_vibe_preset,
#     search_samples, lookup_melody, load_track, stop_music, save_track,
#     delete_track, list_tracks, play_sound, play_animation)
#   - nav (navigate_to_waypoint, navigate_to_coordinates, move_direction,
#     start_mapping, stop_mapping, save_waypoint)
#   - sensors / state (get_music_state, get_battery_level, get_current_time,
#     get_robot_status, get_current_pose, list_waypoints, get_sound_info,
#     list_tts_voices)
#   - config (set_volume, set_voice, set_speed, set_pitch, set_tts_provider)
#   - memory (memory_save, memory_search, memory_context, register_speaker)
CLAIM_JUSTIFYING_TOOLS: frozenset = frozenset({
    # music (ADR-0149 PR-13a: тулы движка v2 вместо compose_music &
    # execute_music_code & set_dj_mode старого пути)
    "request_music", "dj_set",
    "set_vibe_preset", "search_samples", "lookup_melody",
    "load_track", "stop_music", "save_track", "delete_track",
    "list_tracks", "play_sound", "play_animation",
    "generate_music", "gen_play_from_library", "gen_delete_from_library",
    "gen_search_library", "gen_list_library", "gen_get_track_info",
    # issue #2942 — save_arrangement_preset (ADR-0132 PR-7): claim
    # «сохранил/сохраняю пресет» после реального вызова тула не должно
    # ловиться guard'ом как phantom action.
    "save_arrangement_preset",
    # nav
    "navigate_to_waypoint", "navigate_to_coordinates", "move_direction",
    "start_mapping", "stop_mapping", "save_waypoint",
    "delete_waypoint", "clear_waypoints",
    # sensors / state
    "get_music_state", "get_battery_level", "get_current_time",
    "get_robot_status", "get_current_pose", "list_waypoints",
    "get_sound_info", "list_tts_voices",
    # config
    "set_volume", "set_voice", "set_speed", "set_pitch",
    "set_tts_provider",
    # issue #3125 — громкость музыки: «сделал трек громче» после реального
    # вызова — не phantom-claim.
    "set_music_volume",
    # memory / speaker
    "memory_save", "memory_search", "memory_context",
    "register_speaker", "faq_search", "search_web",
})


@dataclass(frozen=True)
class UniversalActionClaimHit:
    """Срабатывание широкого anti-hallucination guard'а.

    Attributes:
        verb: пойманный глагол (для лога).
        tense: ``"past"`` или ``"future"`` — для диагностики.
        excerpt: первые ~80 символов ``spoken`` — для лога.
    """

    verb: str
    tense: str
    excerpt: str


def detect_universal_action_claim(
    *,
    spoken: Optional[str],
    tools_called: Optional[Tuple[str, ...]],
    tool_error_occurred: bool = False,
) -> Optional[UniversalActionClaimHit]:
    """Issue #2549 — широкий детектор «spoken заявляет действие, tools пуст».

    В отличие от :func:`detect_unbacked_action_claim`, НЕ смотритт на
    ``user_input``: ретрай должен сработать даже когда юзер просил
    абстрактно («докрути», «что-то сделай»), а LLM выдала конкретное
    заявление («сделала pass», «проверю и перезапущу»).

    Триггер — ТОЛЬКО в :data:`spoken`. Условия:
      1. ``spoken`` содержит action-verb в past или future (см.
         :data:`_ACTION_VERBS_PAST` / :data:`_ACTION_VERBS_FUTURE`).
      2. ``tools_called`` ∩ :data:`CLAIM_JUSTIFYING_TOOLS`` == ∅, ИЛИ
         вызванный тул вернул ошибку (``tool_error_occurred=True`` —
         issue #2949: гард #2942 считал заявление подкреплённым, если
         тул был ВЫЗВАН, даже когда он отказал/упал. «Записала пресет»
         после ``save_arrangement_preset`` → «недоступен» — тот же
         hallucination, что и пустой ``tools_called``, просто с тулом
         в списке. Подкрепляет заявление ТОЛЬКО успешный вызов).

    Если условие 1 и (условие 2 ИЛИ ошибка) —
    возвращает :class:`UniversalActionClaimHit`, иначе ``None``.
    """
    if not spoken:
        return None
    called = set(tools_called or ())
    if (called & CLAIM_JUSTIFYING_TOOLS) and not tool_error_occurred:
        # LLM вызвал тул, который оправдывает заявление, и тул реально
        # отработал — НЕ вмешиваемся. Issue #2949: тул вызван, но вернул
        # ошибку — не бежим сюда.
        return None

    # Past tense — приоритет, чаще в спонтанных ответах.
    past_match = _ACTION_VERBS_PAST.search(spoken)
    if past_match:
        return UniversalActionClaimHit(
            verb=past_match.group(1),
            tense="past",
            excerpt=spoken[:80],
        )

    future_match = _ACTION_VERBS_FUTURE.search(spoken)
    if future_match:
        return UniversalActionClaimHit(
            verb=future_match.group(1),
            tense="future",
            excerpt=spoken[:80],
        )

    return None


def build_universal_action_claim_retry_prompt(
    *,
    user_input: Optional[str],
    spoken: str,
    hit: "UniversalActionClaimHit",
    tool_error_occurred: bool = False,
) -> str:
    """Issue #2549 — синтетический CRITICAL-ретрай на action hallucination.

    Тот же контракт, что у :func:`build_unbacked_action_retry_prompt` /
    :func:`build_babble_retry_prompt`: одна попытка, текст промпта прямо
    называет заявление и запрещает его без вызова тула.

    Issue #2949 — ``tool_error_occurred=True`` значит инструмент был
    вызван, но вернул ошибку/отказ (не пустой ``tools_called``). Текст
    промпта в этом случае просит честно сообщить о НЕУДАЧЕ, а не просто
    "вызови инструмент" — модель уже его вызывала, вызов ещё раз того
    же провалившегося тула не поможет без честного отчёта.
    """
    cleaned = _strip_trailing_critical_block(user_input or "")
    verb_hint = (
        "Если ты НЕ уверен, что действие произошло — не говори «"
        + hit.verb
        + "», говори «проверяю», «попробую»."
    )
    if tool_error_occurred:
        failure_clause = (
            "но вызванный тобой инструмент ВЕРНУЛ ОШИБКУ/ОТКАЗ — действие "
            "НЕ выполнено. "
        )
        action_clause = (
            "✅ ОБЯЗАТЕЛЬНО: попробуй ещё раз ИЛИ, если повторный вызов "
            "тоже не поможет, честно скажи через speak_text, что действие "
            "НЕ удалось (например «не получилось сохранить») — НЕ повторяй "
            "прежнее заявление об успехе. "
        )
    else:
        failure_clause = "но НЕ вызвал НИ ОДНОГО инструмента (tools=[]). "
        action_clause = (
            "✅ ОБЯЗАТЕЛЬНО: в ЭТОМ же turn вызови соответствующий инструмент "
            "(request_music / dj_set / set_volume / set_voice / save_waypoint / "
            "memory_save / и т.д. по контексту). "
        )
    return (
        f"{cleaned}\n\n"
        "[CRITICAL] В прошлом цикле ты в spoken описал действие ("
        "«запустил/сделал/включил/проверю/обновлю/перезапущу» — поймано «"
        + hit.verb
        + "», tense="
        + hit.tense
        + "), "
        + failure_clause
        + "Пользователь слышит твои слова, но изменений не произойдёт.\n"
        "❌ ЗАПРЕЩЕНО отчитываться о выполненном действии без реального "
        "успешного вызова тула.\n"
        + action_clause
        + verb_hint
        + " После вызова верни 'done' или speak_text с результатом."
    )


#: Issue #3165 — фраза, когда в ходе НИЧЕГО не исполнялось и не падало.
#: «Не получилось выполнить» значит «пытался и упал» — без попытки это
#: неправда (живой прогон 29.09 00:13: так робот ответил на вопрос «что
#: сейчас играет», хотя ни одного тула в ходе не было).
ACTION_CLAIM_NOTHING_DONE_TEXT = (
    "Я сейчас ничего не делал — скажи, пожалуйста, что нужно сделать."
)


def build_action_claim_failure_fallback(
    hit: "UniversalActionClaimHit", *, nothing_attempted: bool = False
) -> str:
    """Issue #2949 — честная фраза после исчерпания бюджета ретраев.

    Когда одноразовый ретрай :func:`build_universal_action_claim_retry_prompt`
    уже потрачен, а модель СНОВА заявляет о выполненном действии, не
    подкреплённом успешным тулом — публикуем это вместо заявления. Цена
    молчаливой деградации выше цены честного «не получилось»: ADR-0018
    («Честный FAIL лучше красивого PASS»).

    Issue #3165: ``nothing_attempted=True`` — в ходе ни один тул не
    исполнялся и не падал. «Не получилось выполнить» тогда само ложь
    (попытки не было), звучит :data:`ACTION_CLAIM_NOTHING_DONE_TEXT`.
    """
    if nothing_attempted:
        return ACTION_CLAIM_NOTHING_DONE_TEXT
    return (
        "Не получилось выполнить — попробуй, пожалуйста, ещё раз "
        "чуть позже."
    )


# ---------------------------------------------------------------------------
# Issue #2559 — phantom-action detector (общий, НЕ music-only).
#
# Round 3 live-check (Vision Pi 10.1.1.21, 15.09.2026, DJ-сет): 6 phantom-
# action случаев за час. Реальные spoken-ы из лога::
#
#   spoken='Понимаю, пока не звучит — дай минуту, проверю состояние
#           и перезапущу.' tools=[] finish_reason='stop'
#   spoken='Ок, давай я снова перезапущу. Бочкинс с Григом наверху —
#           стартуя заново.' tools=[] finish_reason='stop'
#   spoken='Соло сделал погуще, второй голос и пэд подложились.' tools=[]
#
# Юзер слышит «сейчас перезапущу» — и 30 секунд НИЧЕГО не происходит.
# Доверие падает.
#
# Bug E (``detect_unbacked_action_claim``) узкий: требует совпадения и
# ``user_re``, и ``claim_re`` своего правила, и бытовые «перезапущу /
# проверю / подложу / подкрутил» мимо него проходят.
#
# Решение — общий детектор :func:`detect_phantom_action_claim`, который
# срабатывает ВСЕГДА при соблюдении трёх условий:
#
#   * spoken содержит хотя бы один action-verb из
#     :data:`PHANTOM_ACTION_VERBS_RE` (past или future tense);
#   * user_input НЕ содержит явного «не надо» / «не буду» / «не делай»
#     (защита от ложного срабатывания на легитимный «не буду перезапускать»);
#   * tools_called пуст.
#
# Этот детектор ЗАМЫКАЕТ класс «сказал-сделаю, но не вызвал тул» на
# уровне ВСЕХ action-verb'ов, что и просит Issue #2559.
# ---------------------------------------------------------------------------

#: Глаголы, которыми LLM обещает выполнить действие в spoken.
#: Варианты в past/future tense с явными verb-окончаниями: «сделал/
#: сделала/сделаю/сделаем/сделать», «проверю/проверил/проверим/
#: проверить», «запустил/запущу/запустим/запустить» и т.д. Фиксированные
#: verb-окончания обязательны, чтобы negative lookahead
#: :data:`PHANTOM_NOUN_SUFFIXES` корректно отбрасывал «проверка» /
#: «остановка» / «обновление» (см. ниже).
#:
#: Также покрыты причастия «сделано / установлено / остановлено /
#: запущено / обновлено / проверено / перезапущено / переключено /
#: подложено / подкручено / доработано» — это тот же класс claim'а
#: («уже готово» при tools=[]).
#:
#: Шаблоны сознательно НЕ привязаны к музыкальному noun — это
#: расширение Bug E / Bug #2548 на ВСЕ обещания действий.
PHANTOM_ACTION_VERB_STEMS: str = (
    # «сделать» family
    r"сделал\w*|сделала\w*|сделало\w*|сделали\w*|"
    r"сделаю\w*|сделаешь\w*|сделает\w*|сделаем\w*|сделаете\w*|"
    r"сделать\w*|сделай\w*|сделайте\w*|"
    r"сделано\w*|сделана\w*|сделано\w*|сделаны\w*|"
    # «запустить» family
    r"запустил\w*|запустила\w*|запустило\w*|запустили\w*|"
    r"запущу\w*|запустишь\w*|запустит\w*|запустим\w*|запустите\w*|"
    r"запустить\w*|запусти\w*|запустите\w*|"
    r"запущено\w*|запущена\w*|запущены\w*|"
    r"запускал\w*|запускала\w*|запускало\w*|запускали\w*|"
    r"запускаю\w*|запускаешь\w*|запускает\w*|запускаем\w*|запускаете\w*|"
    r"запускать\w*|запускай\w*|запускайте\w*|"
    # «перезапустить» family
    r"перезапустил\w*|перезапустила\w*|перезапустило\w*|перезапустили\w*|"
    r"перезапущу\w*|перезапустишь\w*|перезапустит\w*|перезапустим\w*|перезапустите\w*|"
    r"перезапустить\w*|перезапусти\w*|перезапустите\w*|"
    r"перезапущено\w*|перезапущена\w*|перезапущены\w*|"
    r"перезапускал\w*|перезапускала\w*|перезапускало\w*|перезапускали\w*|"
    r"перезапускаю\w*|перезапускаешь\w*|перезапускает\w*|перезапускаем\w*|перезапускаете\w*|"
    r"перезапускать\w*|перезапускай\w*|перезапускайте\w*|"
    # «установить» family
    r"установил\w*|установила\w*|установило\w*|установили\w*|"
    r"установлю\w*|установишь\w*|установит\w*|установим\w*|установите\w*|"
    r"установить\w*|установи\w*|установите\w*|"
    r"установлено\w*|установлена\w*|установлены\w*|"
    # «остановить» family
    r"остановил\w*|остановила\w*|остановило\w*|остановили\w*|"
    r"остановлю\w*|остановишь\w*|остановит\w*|остановим\w*|остановите\w*|"
    r"остановить\w*|останови\w*|остановите\w*|"
    r"остановлено\w*|остановлена\w*|остановлены\w*|"
    r"останавливал\w*|останавливала\w*|останавливало\w*|останавливали\w*|"
    r"останавливаю\w*|останавливаешь\w*|останавливает\w*|останавливаем\w*|останавливаете\w*|"
    r"останавливать\w*|останавливай\w*|останавливайте\w*|"
    # «проверить» family
    r"проверю\w*|проверил\w*|проверила\w*|проверило\w*|проверили\w*|"
    r"проверишь\w*|проверит\w*|проверим\w*|проверите\w*|"
    r"проверить\w*|проверь\w*|проверьте\w*|"
    r"проверено\w*|проверена\w*|проверены\w*|"
    r"проверял\w*|проверяла\w*|проверяло\w*|проверяли\w*|"
    r"проверяю\w*|проверяешь\w*|проверяет\w*|проверяем\w*|проверяете\w*|"
    r"проверять\w*|проверяй\w*|проверяйте\w*|"
    # «сохранить» family (issue #2942 — live «Понял, сохраняю эти ручки
    # на «В пещере горного короля»…» tools=[] перед save_arrangement_preset;
    # present tense «сохраняю» отсутствовал в словаре целиком — ни в этом
    # списке, ни в past/future _ACTION_VERBS_* выше).
    r"сохранил\w*|сохранила\w*|сохранило\w*|сохранили\w*|"
    r"сохраню\w*|сохранишь\w*|сохранит\w*|сохраним\w*|сохраните\w*|"
    r"сохранить\w*|сохрани\w*|сохраните\w*|"
    r"сохранено\w*|сохранена\w*|сохранены\w*|"
    r"сохранял\w*|сохраняла\w*|сохраняло\w*|сохраняли\w*|"
    r"сохраняю\w*|сохраняешь\w*|сохраняет\w*|сохраняем\w*|сохраняете\w*|"
    r"сохранять\w*|сохраняй\w*|сохраняйте\w*|"
    # «подложить» family (не «подложка»!)
    r"подложил\w*|подложила\w*|подложило\w*|подложили\w*|"
    r"подложу\w*|подложишь\w*|подложит\w*|подложим\w*|подложите\w*|"
    r"подложить\w*|подложи\w*|подложите\w*|"
    r"подложено\w*|подложена\w*|подложены\w*|"
    r"подкладывал\w*|подкладывала\w*|подкладывало\w*|подкладывали\w*|"
    r"подкладываю\w*|подкладываешь\w*|подкладывает\w*|подкладываем\w*|подкладываете\w*|"
    r"подкладывать\w*|подкладывай\w*|подкладывайте\w*|"
    # «переключить» family
    r"переключил\w*|переключила\w*|переключило\w*|переключили\w*|"
    r"переключу\w*|переключишь\w*|переключит\w*|переключим\w*|переключите\w*|"
    r"переключить\w*|переключи\w*|переключите\w*|"
    r"переключено\w*|переключена\w*|переключены\w*|"
    r"переключал\w*|переключала\w*|переключало\w*|переключали\w*|"
    r"переключаю\w*|переключаешь\w*|переключает\w*|переключаем\w*|переключаете\w*|"
    r"переключать\w*|переключай\w*|переключайте\w*|"
    # «подкрутить» family
    r"подкрутил\w*|подкрутила\w*|подкрутило\w*|подкрутили\w*|"
    r"подкручу\w*|подкрутишь\w*|подкрутит\w*|подкрутим\w*|подкрутите\w*|"
    r"подкрутить\w*|подкрути\w*|подкрутите\w*|"
    r"подкручено\w*|подкручена\w*|подкручены\w*|"
    r"подкручивал\w*|подкручивала\w*|подкручивало\w*|подкручивали\w*|"
    r"подкручиваю\w*|подкручиваешь\w*|подкручивает\w*|подкручиваем\w*|подкручиваете\w*|"
    r"подкручивать\w*|подкручивай\w*|подкручивайте\w*|"
    # «обновить» family (не «обновление»!)
    r"обновил\w*|обновила\w*|обновило\w*|обновили\w*|"
    r"обновлю\w*|обновишь\w*|обновит\w*|обновим\w*|обновите\w*|"
    r"обновить\w*|обнови\w*|обновите\w*|"
    r"обновлено\w*|обновлена\w*|обновлены\w*|"
    r"обновлял\w*|обновляла\w*|обновляло\w*|обновляли\w*|"
    r"обновляю\w*|обновляешь\w*|обновляет\w*|обновляем\w*|обновляете\w*|"
    r"обновлять\w*|обновляй\w*|обновляйте\w*|"
    # «поменять» family
    r"поменял\w*|поменяла\w*|поменяло\w*|поменяли\w*|"
    r"поменяю\w*|поменяешь\w*|поменяет\w*|поменяем\w*|поменяете\w*|"
    r"поменять\w*|поменяй\w*|поменяйте\w*|"
    r"поменяно\w*|поменяна\w*|поменяны\w*|"
    # «изменить» family
    r"изменил\w*|изменила\w*|изменило\w*|изменили\w*|"
    r"изменю\w*|изменишь\w*|изменит\w*|изменим\w*|измените\w*|"
    r"изменить\w*|измени\w*|измените\w*|"
    r"изменено\w*|изменена\w*|изменены\w*|"
    r"изменял\w*|изменяла\w*|изменяло\w*|изменяли\w*|"
    r"изменяю\w*|изменяешь\w*|изменяет\w*|изменяем\w*|изменяете\w*|"
    r"изменять\w*|изменяй\w*|изменяйте\w*|"
    # «доработать» family
    r"доработал\w*|доработала\w*|доработало\w*|доработали\w*|"
    r"доработаю\w*|доработаешь\w*|доработает\w*|доработаем\w*|доработаете\w*|"
    r"доработать\w*|доработай\w*|доработайте\w*|"
    r"доработано\w*|доработана\w*|доработаны\w*|"
    # «добавить слой / убрать слой» — конкретные многословные claim'ы
    r"добавил\w+\w+слой|добавлю\w+\w+слой|добавить\w+\w+слой|"
    r"убрал\w+\w+слой|уберу\w+\w+слой|убрать\w+\w+слой"
)
#: Типичные noun-суффиксы (defense-in-depth). Negative lookahead
#: после verb-окончания отбрасывает случаи вроде «проверка пройдена»
#: — здесь после основы идёт ``-ка`` (noun-суффикс), и guard молчит.
#: Глагольные суффиксы ``-л/-ла/-ло/-ли/-ю/-ешь/-ет/-ем/-ете/-ть/-ти/
#: -й/-йте/-но/-на/-но/-ны`` сюда не входят (они уже поглощены stem'ом
#: выше), поэтому :func:`detect_phantom_action_claim` не теряет claim'ы.
PHANTOM_NOUN_SUFFIXES: str = (
    r"(?:ка|ок|ки|ке|ков|кам|кой|ком|ку|ки|кой|кы|кю|"
    r"ние|ения|ание|ение|ания|нию|нием|нения|"
    r"ство|ства|ству|ством|стве|"
    r"тель|атель|итель|ытель|телю|телем|теля|тели|телей|"
    r"ция|ции|цию|цией)"
)
PHANTOM_ACTION_VERBS_RE: "re.Pattern[str]" = re.compile(
    rf"(?<![\w-])(?:{PHANTOM_ACTION_VERB_STEMS})(?!{PHANTOM_NOUN_SUFFIXES})(?![\w-])",
    re.IGNORECASE | re.UNICODE,
)

#: Защита от ложного срабатывания: «не буду перезапускать» / «не надо
#: проверять» / «не делай» — это легитимный ответ «отказ от действия».
#: Без этого гейта guard срабатывал бы на «Не буду ничего перезапускать,
#: давай так оставим» (tools=[] — это согласие юзера на тишину, а не
#: невыполненное действие).
PHANTOM_ACTION_NEGATION_RE: "re.Pattern[str]" = re.compile(
    r"(?:не\s+(?:буду|будешь|будем|стану|надо|надо|нужно|стоит|"
    r"хочу|нужен|нужно|делай|делаю|делаем)|"
    r"не\s+стоит\s+(?:перезапускать|проверять|обновлять|менять|подкручивать)|"
    r"давай(?:те)?\s+не\s+будем|"
    r"оставим\s+как\s+есть|"
    r"ничего\s+не\s+надо)",
    re.IGNORECASE | re.UNICODE,
)


def detect_phantom_action_claim(
    *,
    user_input: Optional[str],
    spoken: Optional[str],
    tools_called: Optional[Tuple[str, ...]],
) -> bool:
    """Issue #2559 — spoken обещает действие, но tools_called пуст.

    Возвращает ``True`` когда ВСЕ условия выполнены:

      1. ``spoken`` содержит хотя бы один ``PHANTOM_ACTION_VERBS_RE`` stem;
      2. ``user_input`` НЕ содержит паттернов «не буду / не надо / не
         делай» (защита от ложного срабатывания на легитимный отказ);
      3. ``tools_called`` пуст (любой тул оправдывает claim).

    Чистая функция, без I/O — тестируется без ROS2. Используется
    :meth:`DialogueNode._check_phantom_action_and_retry` для общего
    (НЕ music-only) детектора «обещал — но не вызвал тул».

    Edge cases:

    * Пустой spoken / None — ``False`` (нечего детектировать).
    * Spoken без action-verb — ``False`` (просто информационный ответ).
    * tools_called НЕ пуст — ``False`` (action подтверждён тулом).
    * user_input с «не буду» — ``False`` (юзер отказался, ретрай лишний).

    Сравнение с Bug E (``detect_unbacked_action_claim``): Bug E
    требует соответствия и ``user_re``, и ``claim_re`` из
    :data:`ACTION_CLAIM_RULES` — то есть И user_input должен быть
    подходящим, И spoken должен попадать в узкий шаблон. Phantom-
    detector упрощает: ему нужен только claim в spoken и пустые tools.
    Это компенсируется :func:`PHANTOM_ACTION_NEGATION_RE`.
    """
    if not spoken:
        return False
    if tools_called:
        return False
    if not PHANTOM_ACTION_VERBS_RE.search(spoken):
        return False
    if user_input and PHANTOM_ACTION_NEGATION_RE.search(user_input):
        return False
    return True


def build_phantom_action_retry_prompt(user_input: Optional[str]) -> str:
    """Issue #2559 — CRITICAL-ретрай «обещал действие без тула».

    Контракт тот же, что у Bug E (:func:`build_unbacked_action_retry_prompt`):
    одна попытка, текст промпта прямо называет класс ошибки и требует
    вызвать НУЖНЫЙ тул в ЭТОМ же turn.

    Логика промпта:

      1. Перечисляет action-verb'ы из :data:`PHANTOM_ACTION_VERB_STEMS`
         (с «сделал» и «перезапущу»), чтобы LLM УВИДЕЛА в промпте то же
         слово, которое сама использовала.
      2. Явно требует: «вызови tool, который выполняет действие».
      3. Если действие НЕ требует тула (напр., это размышление вслух) —
         НЕ обещай действие голосом, а скажи «подумаю / сейчас
         разберусь» без action-verb'ов. Это снижает ложные обещания.
      4. Напоминает о возможности speak_text (если юзер ждал устного
         ответа без инструментального действия).

    Args:
        user_input: оригинальная команда юзера (для контекста).

    Returns:
        Готовый текст для ``_dispatch_turn`` синтетического turn'а.
    """
    cleaned = _strip_trailing_critical_block(user_input or "")
    return (
        f"{cleaned}\n\n"
        "[CRITICAL] Твой предыдущий ответ содержал ОБЕЩАНИЕ ДЕЙСТВИЯ "
        "(слова «сделал / запустил / перезапущу / установлю / остановлю "
        "/ проверю / подложу / переключу / подкручу / обновлю / поменяю "
        "/ изменю / доработаю» и т.п.), но ты НЕ вызвал НИ ОДНОГО "
        "инструмента — значит действие НЕ выполнено, а пользователю "
        "сказана неправда.\n"
        "❌ ЗАПРЕЩЕНО обещать выполнение действия голосом без вызова "
        "инструмента. Это нарушает HONESTY RULE: юзер слышит «сделал» и "
        "ждёт результат, а робот только говорит.\n"
        "✅ В ЭТОМ же turn:\n"
        "  1) Если действие ТРЕБУЕТ инструмента (запустить/остановить "
        "музыку, обновить бит, сохранить точку, перезапустить сервис, "
        "установить голос, переключить трек, проверить состояние через "
        "get_music_state и т.п.) — вызови соответствующий tool "
        "(request_music / stop_music / save_waypoint / set_voice / "
        "load_track / get_music_state и др.).\n"
        "  2) Если инструмент НЕ нужен — ответь БЕЗ action-verb'ов "
        "(«Подумаю», «Сейчас разберусь», «Минутку») или используй "
        "speak_text(...) для устного ответа.\n"
        "Запрос юзера: «" + cleaned + "».\n"
        "Если и сейчас не вызовешь tool — пользователь услышит твоё "
        "обещание и снова ничего не произойдёт."
    )


# ---------------------------------------------------------------------------
# Retry prompt builders
# ---------------------------------------------------------------------------

# Общий префикс Bug C retry-промпта (tool-skipped, issue #1777). Имя
# историческое: музыкальный ретрай с тем же префиксом удалён в ADR-0149
# PR-13a; маркер считает ``scripts/voice_bench/run_bench.py``.
MUSIC_RETRY_PROMPT_PREFIX: str = "[CRITICAL] В прошлом цикле ты НЕ вызвал"


CRITICAL_BLOCK_MARKER = "[CRITICAL]"
CRITICAL_BLOCK_MARKER_LEN = len(CRITICAL_BLOCK_MARKER)


def _strip_trailing_critical_block(user_input: str) -> str:
    """Issue #1881 — убрать последний ``[CRITICAL]``-блок из ``user_input``.

    На babble-retry ``user_input`` уже содержит прошлый CRITICAL-блок
    (от прошлого ретрая этого же turn'а). Если просто склеить его с
    НОВЫМ блоком — модель видит два ПРОТИВОРЕЧИВЫХ требования
    («вызови tool» vs «вызови music-tool») и начинает мешать их в
    ответе (vision-pi 02.09: «однако другой [CRITICAL] говорит...»).

    Решение: если input заканчивается на блок, начинающийся с
    ``[CRITICAL]`` — обрезаем до его начала. Юзер-фраза остаётся;
    старый блок заменяется новым.

    Edge cases:
    * Без маркера — возвращаем ``user_input`` as-is.
    * Маркер в самом начале (user_input="[CRITICAL]...") — возвращаем
      пустую строку (нет юзер-фразы для сохранения). Это безопасно:
      даже пустой входной текст + новый блок = «без исходного
      запроса». LLM выдаст инструментальный ответ или пустоту, что
      лучше, чем два конфликтующих CRITICAL'a.
    """
    if not user_input:
        return user_input
    idx = user_input.rfind(CRITICAL_BLOCK_MARKER)
    if idx <= 0:
        # Нет маркера вообще, или маркер в самом начале (idx==0).
        # В обоих случаях склеивать не с чем — возвращаем as-is.
        return user_input
    # ``idx > 0``: маркер где-то внутри. Обрезаем всё от него до конца.
    return user_input[:idx].rstrip()


def build_babble_retry_prompt(user_input: str) -> str:
    """Issue #992 Bug D — synthetic follow-up prompt for babble retry.

    Echoes the original ``user_input`` so the LLM has the request
    in context, then appends a CRITICAL instruction that names the
    babble pattern and demands a tool-call reply (no plain text
    promises).

    🔴 FIX (live 02.09, "включи трек про весну"): список обязан называть
    вариант «уже существующий трек» — иначе модель, уже нашедшая трек через
    gen_search_library, сочиняла новую мелодию вместо
    gen_play_from_library(track_id=...). ADR-0149 PR-13a: музыку ставит
    ``request_music`` движка v2 (``compose_music`` / ``execute_music_code``
    старого пути удалены).

    Issue #1881 — если ``user_input`` уже содержит предыдущий
    ``[CRITICAL]``-блок (от прошлого ретрая этого же turn'а), он
    обрезается перед склейкой, чтобы НЕ накапливать противоречивые
    инструкции. Иначе модель читает «вызови tool» + «вызови
    music-tool» и в каждом ответе спорит сама с собой.
    """
    cleaned = _strip_trailing_critical_block(user_input)
    return (
        f"{cleaned}\n\n"
        "[CRITICAL] Твой предыдущий ответ был метатекст "
        "(начинался с «зачит», «могу», «хочешь», «сейчас», "
        "«устроим», «погнали», «слушай», «давай», «так» или "
        "«переключ») — пользователь слышит пустую болтовню "
        "вместо результата.\n"
        "❌ ЗАПРЕЩЕНО отвечать текстом-обещанием. "
        "✅ ОБЯЗАТЕЛЬНО: вызови нужный tool в ЭТОМ же turn:\n"
        "  • rap/песня → request_music(intent=\"track\", text=...) + "
        "speak_text(lyrics),\n"
        "  • поэзия → speak_text(...) × N строк,\n"
        "  • новая мелодия/бит → request_music(intent=\"track\", text=...),\n"
        "  • анекдот → speak_text(...) × N,\n"
        "  • уже существующий/сохранённый трек по имени или теме — "
        "НЕ сочиняй новый: если в этом диалоге уже был вызов "
        "gen_search_library/gen_list_library/list_tracks с подходящим "
        "результатом, возьми его track_id/name и вызови "
        "gen_play_from_library(track_id=...) или load_track(name=...); "
        "иначе вызови поиск сейчас, а не request_music.\n"
        "После последнего speak_text верни 'done'. Никаких "
        "мета-фраз, никаких 'Слушай, сейчас...', 'Зачитаю...', "
        "'Могу бит добавить, хочешь?' — это BUG."
    )


def build_unbacked_action_retry_prompt(
    *, user_input: str, spoken: str, rule: "ActionClaimRule"
) -> str:
    """Issue #992 Bug E — синтетический ретрай «отчитался, но не сделал».

    Повторяет контракт Bug C: одна попытка, текст промпта прямо называет
    и заявление, и тул, которого не хватило.
    """
    return (
        "[CRITICAL] Ты ответил «"
        + (spoken or "").strip()[:120]
        + "», но НЕ вызвал ни одного тула — значит действие НЕ выполнено, "
        "а пользователю сказана неправда.\n"
        "❌ ЗАПРЕЩЕНО отчитываться о выполненном действии без вызова тула.\n"
        "✅ В ЭТОМ же turn вызови тул: " + rule.what + ".\n"
        "Запрос юзера: «" + (user_input or "") + "».\n"
        "Если тул вернёт ошибку — скажи об ошибке честно, не выдумывай успех."
    )


def spoken_matches_claim_category(category: str, spoken: Optional[str]) -> bool:
    """Issue #2780 — ``spoken`` матчит ``claim_re`` правила ``category``?

    Обобщение приватного ``DialogueNode._spoken_matches_music_prose_claim``
    (issue #2548) на произвольную категорию :data:`ACTION_CLAIM_RULES`.
    Нужен вызывающему коду, который проверяет fallback ПОСЛЕ того, как
    одноразовый ретрай уже потрачен в этом ходе (``_action_claim_retry_used
    =True`` — см. :func:`build_unbacked_action_retry_prompt`): guard сам
    больше не сработает в этом ходе, но ``spoken`` всё ещё может повторять
    то же (или переформулированное) заявление, и вызывающая сторона должна
    решить, подменять ли его честной констатацией (см.
    :func:`build_music_prose_action_fallback`,
    :func:`build_fact_memory_save_fallback`) вместо публикации в TTS.

    ``category`` — тег :attr:`ActionClaimRule.category` (например,
    ``"fact_memory_save"`` или ``"music_prose_action"``). Неизвестная
    категория или пустой ``spoken`` — ``False`` (безопасный дефолт: не
    подменять реплику вслепую).
    """
    if not spoken:
        return False
    rule = next(
        (r for r in ACTION_CLAIM_RULES if r.category == category), None
    )
    return bool(rule is not None and rule.claim_re.search(spoken))


def build_fact_memory_save_fallback() -> str:
    """Issue #2780 — fallback spoken для категории ``fact_memory_save``.

    Зеркало :func:`build_music_prose_action_fallback` (issue #2548), но
    для памяти: прогон 35734532425 (шаг ``n206_boris_memory``) показал
    ход из трёх реплик подряд — «Запомнил» (ложь, tools=[]) → guard
    поймал, ретрай честно признался в сбое → «Всё на месте, запись
    подтверждена» (ОПЯТЬ ложь, tools=['memory_context'] — ЧТЕНИЕ, не
    запись). К третьей реплике одноразовый ретрай уже потрачен на первой,
    guard больше не выстрелит в этом ходе (см. ``_action_claim_retry_used``
    в ``dialogue_node.py``) — расширенный ``claim_re`` в
    :data:`ACTION_CLAIM_RULES` (категория ``fact_memory_save``) теперь
    ЛОВИТ и такую формулировку, но ловить мало: без замены spoken юзер
    всё равно услышит уверенное подтверждение несуществующей записи.

    Использование зеркалит :func:`build_music_prose_action_fallback`:
    вызывающая сторона (``DialogueNode._handle_result``, по аналогии с
    ``_publish_music_prose_action_fallback_if_needed``) при условии
    ``_action_claim_retry_used=True`` и
    ``spoken_matches_claim_category("fact_memory_save", spoken)`` должна
    подменить ``spoken`` этой констатацией ПЕРЕД публикацией в TTS —
    без claim о выполнении, без "всё на месте", без извинений.

    Проводка в ``dialogue_node.py`` — отдельный шаг (issue #2780 п.3),
    сюда не входит: файл в это время дорабатывает параллельная сессия
    (issue #2779). Эта функция и :func:`spoken_matches_claim_category`
    — готовый строительный блок для такой проводки.
    """
    return (
        "Не получилось точно сохранить факт — сохраню ещё раз, "
        "чтобы наверняка."
    )


# ---------------------------------------------------------------------------
# Issue #2175 — MiniMax-M3 regurgitates ``<system>...</system>`` template
# instead of producing a user-facing reply.
#
# Live 08.09 (Vision Pi, 14:52): три запроса подряд после ``set_voice``
# + multi-voice user_input + новая DJ-skill context дали в ``spoken``
# ровно строку вида::
#
#     <system>
#     [получатель ответа забыл указать антропоморфные атрибуты]
#     </system>
#
# Это кусок СИСТЕМНОГО шаблона из master_prompt_compact.txt (см.
# ``RULE #RESPONSE_FORMAT``), который модель regurgitates буквально.
# TTS озвучивал эту директиву через Yandex→MiniMax fallback, юзер слышал
# «получатель ответа забыл указать антропоморфные атрибуты» поверх
# только что сменённого голоса.
#
# Защита — двухуровневая:
#
# 1. :func:`is_system_template_regurgitated` распознаёт regurgitates в
#    extracted ``spoken`` (без SSML-обёртки): ловит ПОЛНЫЙ текст вида
#    ``^<system>...</system>$`` — серединные ссылки на ``<system>``
#    в обычной фразе НЕ блокируются.
# 2. :func:`is_system_template_regurgitated_in_ssml` — defense-in-depth
#    для tts_node: в SSML ``<system>...</system>`` может быть обрамлён
#    ``<speak>...</speak>``. Использует более узкую эвристику —
#    парный блок, чьё содержимое НЕ содержит других тегов ИЛИ обрамлён
#    ``<speak>...</speak>``.
#
# Также :func:`build_system_regurgitate_retry_prompt` строит одноразовый
# CRITICAL-ретрай с явным требованием отвечать обычным языком.
# ---------------------------------------------------------------------------
# Regex для extracted spoken — полный парный блок без surrounding текста.
# match() (а не search()) гарантирует что ВЕСЬ текст — это regurgitates.
SYSTEM_TEMPLATE_REGURGITATE_RE = re.compile(
    r"^\s*<system>.*?</system>\s*$",
    re.DOTALL | re.IGNORECASE,
)

# Regex для SSML (defense-in-depth) — парный ``<system>...</system>``
# внутри ``<speak>...</speak>``. Также ловит ``<system>...</system>``
# как единственный верхнеуровневый блок (без ``<speak>``-обёртки).
# search() — потому что блок внутри SSML.
SYSTEM_TEMPLATE_REGURGITATE_SSML_RE = re.compile(
    r"^<speak>\s*<system>.*?</system>\s*</speak>$"
    r"|^<system>.*?</system>$",
    re.DOTALL | re.IGNORECASE,
)


def is_system_template_regurgitated(spoken_text: Optional[str]) -> bool:
    """Issue #2175 — детектор regurgitated ``<system>...</system>``.

    Работает на **extracted spoken** (без SSML-обёртки) — то, что
    dialogue_node получает в ``result.spoken_text``. ``True`` только
    если ВЕСЬ текст это полный парный блок ``<system>...</system>``
    (с произвольным whitespace вокруг). Возвращает ``False`` для
    пустой строки, для серединных ссылок на ``<system>`` в обычной
    фразе («согласно <system>инструкции</system>» — это легитимный
    текст, не regurgitates), и для неполных тегов.

    Делегирует :data:`SYSTEM_TEMPLATE_REGURGITATE_RE` — единая regex
    для dialogue_node guard'а.
    """
    if not spoken_text:
        return False
    return bool(SYSTEM_TEMPLATE_REGURGITATE_RE.match(spoken_text))


def is_system_template_regurgitated_in_ssml(ssml: Optional[str]) -> bool:
    """Issue #2175 — defense-in-depth для tts_node.

    Работает на СЫРОМ SSML (входе в ``dialogue_callback``). Ловит
    regurgitates в двух форматах:

    1. ``<speak><system>...</system></speak>`` — типичный случай
       после ``_extract_text_from_ssml`` (когда strip ещё не прошёл).
    2. ``<system>...</system>`` без SSML-обёртки — на случай если
       producer забыл обернуть в ``<speak>``.

    Возвращает ``False`` для серединных ссылок в обычной фразе
    («<speak>Согласно <system>инструкции</system>, отвечу.</speak>»)
    — match() ограничивает всю строку.
    """
    if not ssml:
        return False
    return bool(SYSTEM_TEMPLATE_REGURGITATE_SSML_RE.match(ssml))


def build_system_regurgitate_retry_prompt(user_input: Optional[str]) -> str:
    """Issue #2175 — синтетический CRITICAL-ретрай на regurgitated template.

    MiniMax-M3 иногда отвечает не финальной репликой, а куском СВОЕГО
    системного промпта (``<system>[получатель ответа забыл указать
    антропоморфные атрибуты]</system>``). Это невалидный user-facing
    ответ: TTS озвучивает метаинструкцию вместо результата. Один
    одноразовый ретрай с явным требованием отвечать обычным языком,
    БЕЗ XML-обёрток.

    Args:
        user_input: оригинальная команда юзера (для контекста в ретрае).

    Returns:
        Текст промпта, который ``dialogue_node._dispatch_turn`` отдаст
        LLM как ``user_input`` синтетического turn'а (тот же контракт,
        что у ``build_babble_retry_prompt`` / ``build_unbacked_action_retry_prompt``).
    """
    cleaned = _strip_trailing_critical_block(user_input or "")
    return (
        f"{cleaned}\n\n"
        "[CRITICAL] Твой предыдущий ответ был НЕ финальной репликой, а "
        "regurgitates внутреннего системного шаблона — пользователь "
        "услышал метаинструкцию («получатель ответа забыл указать "
        "антропоморфные атрибуты» и подобное) вместо результата.\n"
        "❌ ЗАПРЕЩЕНО копировать содержимое системного промпта "
        "(включая XML-блоки ``<system>...</system>``, "
        "``<system_context>...</system_context>``, "
        "``<hardware>...</hardware>`` и любые другие) в свой ответ. "
        "❌ ЗАПРЕЩЕНО отвечать одной директивой без действия.\n"
        "✅ В ЭТОМ же turn ответь обычным русским языком (без XML-обёрток "
        "и meta-маркеров), вызови нужный tool и заверши 'done'."
    )


# ---------------------------------------------------------------------------
# Issue #2760 — модель ПЕЧАТАЕТ вызов тула вместо того, чтобы его сделать.
#
# Live Vision Pi, прогон 35704637846 (акт 2, шаг n204_boris_intro_long)::
#
#   spoken='<function_calls>\n<invoke name="register_speaker">\n
#           <parameter name="name">Борис</parameter>\n</invoke>\n
#           <invoke name="memory_save">…</invoke>\n</function_calls>'
#   tools=[] finish_reason='stop'
#
# Дальше без задержки: ``📤 LLM OUTPUT`` → два TTS-чанка → робот читает
# разметку вслух, а текст оседает в истории как реплика ассистента
# (следующий ход учится на этом примере).
#
# Отличие от #2175: там модель возвращает кусок СИСТЕМНОГО промпта, здесь —
# синтаксис ВЫЗОВА инструмента. Отличие от #2549/#2559: там модель
# ОПИСЫВАЕТ действие прозой («сделала», «перезапущу»), и детектор ищет
# глаголы; здесь глаголов нет вообще — есть теги.
#
# Разбор причины (модель MiniMax-M3 отправляет решение в канал content
# при исправно переданных тулах), восстановление намерения и сам регекс —
# в :mod:`rob_box_harness.core.tool_loop.markup_recovery` и
# ``text_classify``. Здесь — последний рубеж: не дать тегам прозвучать.
# ---------------------------------------------------------------------------
# ⚠️ Регекс намеренно ДУБЛИРУЕТСЯ с
# ``rob_box_harness.core.tool_loop.text_classify.TOOL_CALL_MARKUP_RE``.
# Свести в один модуль нельзя: harness — нижний слой, он не имеет права
# импортировать voice, а этот модуль обязан оставаться pure-Python без
# зависимостей (см. шапку файла). Слои решают РАЗНЫЕ задачи: там —
# восстановить намерение внутри цикла тулов, здесь — не дать тегам
# прозвучать. Правку одного регекса переносить во второй; эквивалентность
# закреплена тестом ``test_issue_2760_tool_call_markup.py``.
TOOL_CALL_MARKUP_RE = re.compile(
    # Родной диалект MiniMax (официальный tool_calling_guide.md):
    # <minimax:tool_call><invoke name="..."><parameter name="...">.
    r"</?minimax:tool_call\s*>"
    # Диалект, который робот получил живьём 22.09 (Anthropic-style).
    r"|</?function_?calls\s*>"
    r"|</?tool_?calls?\s*>"
    r"|<\s*/?\s*(?:minimax:|antml:)?invoke(?:\s+name\s*=|\s*>)"
    r"|<\s*/?\s*(?:minimax:|antml:)?parameter(?:\s+name\s*=|\s*>)"
    r"|<\s*antml:",
    re.IGNORECASE,
)


def is_tool_call_markup(spoken_text: Optional[str]) -> bool:
    """Issue #2760 — в ``spoken`` лежит разметка вызова тула, а не речь.

    ``True``, если текст содержит хоть один маркер из
    :data:`TOOL_CALL_MARKUP_RE`. Пустая строка / ``None`` → ``False``.
    """
    if not spoken_text:
        return False
    return bool(TOOL_CALL_MARKUP_RE.search(spoken_text))


def build_tool_call_markup_retry_prompt(user_input: Optional[str]) -> str:
    """Issue #2760 — CRITICAL-ретрай на «вызов тула написан текстом».

    Тот же контракт, что у :func:`build_system_regurgitate_retry_prompt`:
    одна попытка, промпт прямо называет ошибку и требует настоящий
    tool-call.
    """
    cleaned = _strip_trailing_critical_block(user_input or "")
    return (
        f"{cleaned}\n\n"
        "[CRITICAL] В прошлом цикле ты НАПИСАЛ вызов инструментов текстом "
        "(``<function_calls><invoke name=\"...\">...``) вместо того, чтобы "
        "их вызвать. Инструменты не сработали, а разметку прочитал вслух "
        "синтезатор речи — пользователь услышал теги.\n"
        "❌ ЗАПРЕЩЕНО писать ``<function_calls>``, ``<invoke>``, "
        "``<parameter>`` и любые другие теги протокола в текст ответа.\n"
        "✅ В ЭТОМ же turn вызови нужные инструменты штатным механизмом "
        "tool-calls, а в текст ответа напиши только человеческую фразу "
        "на русском."
    )
