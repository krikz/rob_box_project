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
* **Music guard** (issue #992 Bug B/C) — :func:`user_wants_music`
  decides whether the user asked for a track, :func:`is_music_stop_command`
  recognises stop-commands that must NOT be treated as music requests,
  :func:`is_vocal_request` recognises «спой/пой/песня» where ``speak_text``
  alone is a valid outcome, and :func:`build_music_retry_prompt` builds
  the Bug-C retry prompt that demands ``execute_music_code``.
* **Non-music tool guard** (issue #1777 / #1762) — расширение Bug C retry на
  явные tool-based запросы (``get_current_time``, ``search_web``,
  ``set_voice``, ``memory_search``, ``faq_search``). Когда LLM отвечает
  текстом-обещанием и не вызывает нужный tool, диалог-NODE шлёт один
  CRITICAL retry с явным указанием имени инструмента. См.
  :data:`TOOL_REQUEST_PATTERNS`, :func:`detect_required_tool`,
  :func:`build_tool_retry_prompt`.
"""

from __future__ import annotations

import logging
import re
from typing import Optional

logger = logging.getLogger(__name__)


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

# Issue #992 Bug C — keywords that mark the user input as a request to
# play music / a track. Narrow by design: ordinary chit-chat that
# mentions "track" in passing must NOT trigger the music guard.
#
# Issue #1392 — added AI-generation triggers («сгенерируй песню/мелодию»,
# «сочини трек», «сделай музыку», «генератор музыки»). These MUST fire
# the music guard too: without it, the LLM bypasses handle_music and
# answers text-only "🎤 Меняю роль!" — exactly the live regression
# observed on Vision Pi 18.08.2026, 17:05 MSK.
MUSIC_GUARD_KEYWORDS: tuple = (
    "спой",
    "пой ",
    "рэп",
    "рап",
    "диджей",
    "диджея",
    "dj ",
    "dj-",
    "песня",
    "песню",
    "зачитай",
    "зачита",
    "зачитывай",
    "сыграй",
    "играй",
    "включи музык",
    "запусти музык",
    # Воспроизведение готового трека из библиотеки (live 20.08) —
    # «включи/поставь трек», «случайный/следующий трек», «мелодия»,
    # «включал ... через библиотеку». Без этих подстрок Bug C retry
    # молчал («user does NOT want music»), а LLM отвечал «Запускаю!»
    # с tools=[] — словами, а не вызовом тула.
    "включи трек",
    "включи мелоди",
    "поставь трек",
    "поставь мелоди",
    "поставь музык",
    "запусти трек",
    "запусти мелоди",
    "включал трек",
    "включал мелоди",
    "включал музык",
    "следующий трек",
    "случайный трек",
    "рандомн",
    "вруби",
    "трек из библиотек",
    "мелоди",
    # AI-generated tracks (issue #1392) — без них CRITICAL-retry ниже
    # не сработает, и LLM уйдёт в голосовой текст вместо generate_music.
    "сгенерируй песн",
    "сгенерируй мелоди",
    "сгенерируй музык",
    "сгенерируй трек",
    "сгенерируй композиц",
    "сгенерируй вокал",
    "сочини песн",
    "сочини музык",
    "сочини трек",
    "сделай песн",
    "сделай музык",
    "сгенерируем песн",
    "сгенерируем музык",
    "генератор музык",
    "генерация музык",
    "сгенерит музык",
    "миниmax-music",
)

# 🔴 FIX (live 06.08): «хватит диджеить/выключи музыку» — юзер просит
# остановить музыку/DJ, а НЕ замолчать робота. Подстрока «хватит»
# в silence_commands перехватывала такие команды до LLM. Эти фразы
# пробивают silence-гейт и идут в LLM (который вызовет stop_music +
# set_dj_mode(enabled=false)).
MUSIC_STOP_OVERRIDES: tuple = (
    "диджеить",
    "диджея",
    "диджей режим",
    "выключи музыку",
    "выключ музыку",
    "музыку выключ",
    "стоп музык",
    "останови музык",
    "убери музык",
)

# 🔴 FIX (live 10:00): для ГОЛОСОВЫХ запросов («спой/пой/песня»)
# speak_text достаточно — бит не обязателен (юзер мог попросить
# спеть ПОД уже играющую музыку, как «спой про мурку в этот
# момент» — Григ играл, LLM правильно не перезапустила трек).
# Bug C нудит только если LLM вообще НИЧЕГО не сделала (tools
# пуст). Для БИТО-обязательных («рэп/зачитай/диджей») — как было:
# нуднуть если нет execute_music_code.
MUSIC_GUARD_VOCAL_KEYWORDS: tuple = (
    "спой",
    "пой ",
    "песня",
    "песню",
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


def user_wants_performance(user_input: str) -> bool:
    """Issue #992 Bug D — does the user request a *performance*?

    Used to decide whether a metalanguage reply is a hard bug
    (user asked for a rap, robot returned "Зачитаю рэп про X!") or
    just a stylistic miss (user asked "что нового?", robot replied
    "Слушай, у меня тут..." — still answer-shaped, just informal).
    """
    if not user_input:
        return False
    low = user_input.lower()
    return any(kw in low for kw in BABBLE_PERFORMANCE_KEYWORDS)


def user_wants_music(user_input: str, *, logger: Optional[logging.Logger] = None) -> bool:
    """Heuristic: does the user request music / a track?

    Used by the Bug-C code-side fallback to decide whether a retry
    should fire. The check is intentionally narrow so we don't retry
    on ordinary chit-chat that happens to mention "track" in passing.

    ``logger`` is optional: when provided, the diagnostic debug/info
    lines that help spot missing keywords in production are emitted
    through it.
    """
    if not user_input:
        return False
    low = user_input.lower()
    matched = [kw for kw in MUSIC_GUARD_KEYWORDS if kw in low]
    if matched:
        if logger is not None:
            logger.debug(
                f"🎵 [music_guard] user_input={user_input!r} matched "
                f"keywords={matched!r} → wants_music=True"
            )
        return True
    # 💡 Diagnostic: log when input LOOKS music-related but no
    # keyword matched — helps spot missing keywords in production.
    # Check against the broader BABBLE_PERFORMANCE_KEYWORDS set
    # (which includes "сыграй", "играй", "музык", etc.) to catch
    # false-negatives without spamming on ordinary chit-chat.
    broad_match = [kw for kw in BABBLE_PERFORMANCE_KEYWORDS if kw in low]
    if broad_match and logger is not None:
        logger.info(
            f"🎵 [music_guard] user_input={user_input!r} matched "
            f"broad_performance={broad_match!r} but NOT in "
            f"MUSIC_GUARD_KEYWORDS → wants_music=False "
            f"(возможно, нужно добавить keyword в MUSIC_GUARD_KEYWORDS)"
        )
    return False


def is_music_stop_command(user_input: str) -> bool:
    """Issue #992 Bug C — is this a music/DJ stop-command?

    «хватит диджеить», «выключи музыку», «стоп музыку» — these are
    requests to STOP music, not to START it. The music guard must
    skip them entirely (otherwise a stop-command triggers a retry
    that re-enables music).
    """
    if not user_input:
        return False
    low = user_input.lower()
    return any(kw in low for kw in MUSIC_STOP_OVERRIDES)


def is_vocal_request(user_input: str) -> bool:
    """Issue #992 Bug C — is this a vocal («спой/пой/песня») request?

    For vocal requests ``speak_text`` alone is a valid outcome — the
    user may have asked to sing UNDER already playing music, so a
    missing ``execute_music_code`` is not necessarily a skip. Bug C
    only nudges when the LLM did literally nothing (tools empty).
    """
    if not user_input:
        return False
    low = user_input.lower()
    return any(kw in low for kw in MUSIC_GUARD_VOCAL_KEYWORDS)


# ---------------------------------------------------------------------------
# Retry prompt builders
# ---------------------------------------------------------------------------

# Общий префикс Bug C retry-промпта. dialogue_node._run_turn проверяет
# ``startswith`` этого префикса, чтобы НЕ сбрасывать retry-бюджет на
# синтетическом ретрае (иначе каждый ретрай считается «новым запросом»,
# бюджет сбрасывается бесконечно и Bug C зацикливается). Один источник
# правды: переименование тула внутри текста промпта больше не может
# молча сломать guard.
MUSIC_RETRY_PROMPT_PREFIX: str = "[CRITICAL] В прошлом цикле ты НЕ вызвал"


def build_babble_retry_prompt(user_input: str) -> str:
    """Issue #992 Bug D — synthetic follow-up prompt for babble retry.

    Echoes the original ``user_input`` so the LLM has the request
    in context, then appends a CRITICAL instruction that names the
    babble pattern and demands a tool-call reply (no plain text
    promises).
    """
    return (
        f"{user_input}\n\n"
        "[CRITICAL] Твой предыдущий ответ был метатекст "
        "(начинался с «зачит», «могу», «хочешь», «сейчас», "
        "«устроим», «погнали», «слушай», «давай», «так» или "
        "«переключ») — пользователь слышит пустую болтовню "
        "вместо результата.\n"
        "❌ ЗАПРЕЩЕНО отвечать текстом-обещанием. "
        "✅ ОБЯЗАТЕЛЬНО: вызови нужный tool в ЭТОМ же turn:\n"
        "  • rap/песня → execute_music_code + speak_text(lyrics),\n"
        "  • поэзия → speak_text(...) × N строк,\n"
        "  • мелодия → execute_music_code(...),\n"
        "  • анекдот → speak_text(...) × N.\n"
        "После последнего speak_text верни 'done'. Никаких "
        "мета-фраз, никаких 'Слушай, сейчас...', 'Зачитаю...', "
        "'Могу бит добавить, хочешь?' — это BUG."
    )


def build_music_retry_prompt(user_input: str) -> str:
    """Synthetic prompt for Bug C retry (user asked for music, LLM skipped
    the music tools).

    The LLM frequently concludes «музыка уже играет» from the dialogue
    history (previous runs/songs) and returns ``done`` without calling
    ``execute_music_code`` / ``generate_music``. This prompt explicitly
    resets that assumption and demands the tool call — pointing at BOTH
    engines directly (Renardo ``execute_music_code`` AND MiniMax
    ``generate_music``), because ``handle_music`` (the old Compositor
    skill facade) no longer has an executor after the harness migration.
    """
    return (
        MUSIC_RETRY_PROMPT_PREFIX + " ни один музыкальный тул, "
        "хотя пользователь ЯВНО попросил музыку/генерацию. "
        "Музыка сейчас НЕ играет — предыдущие треки уже остановлены. "
        "ОДИН ИЗ ЭТИХ инструментов ОБЯЗАТЕЛЕН (выбери по контексту): "
        "1) execute_music_code (Renardo/SuperCollider) — бит/DJ/ambient/instrumental (быстрый, ~1с); "
        "2) generate_music (MiniMax Music API, 40-160с) — песня с вокалом и лирикой; "
        "3) gen_search_library / gen_list_library / gen_play_from_library — для уже сохранённых треков. "
        "Запрос юзера: «"
        + (user_input or "")
        + "». "
        "Если это 'спой песню про X' / 'сгенерируй трек про X' / 'сочини музыку' — "
        "вызывай generate_music(...). Если 'бит/DJ/ambient' — execute_music_code(...). "
        "Если 'включи/сыграй/поставь трек/мелодию', 'случайный/следующий трек', "
        "'трек из библиотеки' — сначала gen_list_library(limit=5), выбери track_id, "
        "затем gen_play_from_library(track_id=...). "
        "Если и сейчас не вызовешь tool — цикл останется пустым."
    )


# ---------------------------------------------------------------------------
# Issue #1777 / #1762 — non-music tool guard. Расширение Bug C retry на
# ВСЕ явные tool-based запросы, когда юзер спросил про конкретный ресурс
# (время, погода, голос, память, FAQ), а LLM «забыл» вызвать нужный tool
# и ответил текстом-обещанием («сейчас расскажу», «гляну», «уже включил»).
#
# Формат: каждая запись = (tool_name, tuple[подстрок…]). Подстроки
# матчатся lowercase substring в user_input. Tuple — для случаев когда
# одна категория покрывается несколькими keyword'ами («который час»,
# «сколько времени», «время в москве» — всё → get_current_time).
#
# ВАЖНО: keyword'ы подобраны так, чтобы НЕ срабатывать на обычное chit-chat
# и НЕ конкурировать с другими guards (music / babble). Например, в
# категории ``set_voice`` намеренно нет просто «голос» — иначе триггерилось
# бы на «у тебя какой голос?» (это FAQ / chit-chat, а не set_voice).
#
# Приоритет (порядок в tuple) решает, если несколько категорий матчат
# один и тот же user_input. На практике категории непересекающиеся,
# но порядок — страховка на будущее.
# ---------------------------------------------------------------------------
TOOL_REQUEST_PATTERNS: tuple = (
    # time / date (issue #1777) — «который час», «сколько времени»,
    # «время в москве», «какая дата», «какой день недели», «чо за время».
    (
        "get_current_time",
        (
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
        ),
    ),
    # weather / news / web search (issue #1762) — «погода в X»,
    # «новости про Y», «что в интернете», «загугли».
    (
        "search_web",
        (
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
            "расскажи про ",
            "что ты знаешь про ",
            "что известно про ",
        ),
    ),
    # voice (issue #1765) — «переключи голос», «говори X голосом»,
    # «голос Артём», «смени голос». Узкий список, чтобы НЕ триггерить
    # на «у тебя какой голос?» (это FAQ).
    (
        "set_voice",
        (
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
            "говор ",
            "говори ",
            "говорит ",
            "голосом",
            "давай голос",
            "поставь голос",
            "установи голос",
        ),
    ),
    # memory (issue #1770) — «что ты знаешь обо мне», «помнишь меня»,
    # «что помнишь».
    (
        "memory_search",
        (
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
        ),
    ),
    # FAQ — «что ты умеешь», «какие команды», «справка».
    (
        "faq_search",
        (
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
        ),
    ),
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


# Issue #1777 — фиксированный набор tool_name, который build_tool_retry_prompt
# принимает. Не пересекается с публичным API, держим как module-private
# allow-list, чтобы defence-in-depth нельзя было обойти инъекцией через
# юзер-ввод (см. комментарий в build_tool_retry_prompt).
_TOOL_RETRY_HINTS: dict = {
    "get_current_time": (
        "вызови get_current_time() — инструмент возвращает точное "
        "локальное время робота (Europe/Moscow по умолчанию). "
        "Не выдумывай время, не говори «сейчас X утра/вечера» из головы."
    ),
    "search_web": (
        "вызови search_web(query=...) — инструмент ищет актуальную "
        "информацию в интернете (погода, новости, факты). "
        "Не говори «гляну / сделаю / сейчас узнаю» без реального вызова."
    ),
    "set_voice": (
        "вызови set_voice(provider=..., voice_name=...) или "
        "list_voices() чтобы выбрать. Не говори «голоса X нет» "
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
}


def build_tool_retry_prompt(user_input: str, tool_name: str) -> str:
    """Issue #1777 / #1762 — synthetic prompt для Bug C retry (non-music).

    Echoes the original ``user_input`` so the LLM has the request in
    context, then injects a CRITICAL reminder that names the specific
    ``tool_name`` the LLM must call. ``tool_name`` MUST come from
    :func:`detect_required_tool` (or any hard-coded allow-list) — never
    pass user-controlled strings. Неизвестный tool_name → ``""`` (caller
    пропускает retry).

    Prefix — тот же :data:`MUSIC_RETRY_PROMPT_PREFIX`, что и у music
    retry. ``dialogue_node._run_turn`` проверяет ``startswith`` этого
    префикса, чтобы НЕ сбрасывать retry-бюджет на синтетическом
    промпте (иначе каждый ретрай считался бы «новым запросом», бюджет
    сбрасывался бесконечно и Bug C зацикливался — см. issue #992
    Bug C root cause).
    """
    hint = _TOOL_RETRY_HINTS.get(tool_name)
    if hint is None:
        # Defence-in-depth: tool_name не из allow-list → prompt-injection
        # или ошибка вызывающего. НЕ ретраим, отдаём пустую строку;
        # dialogue_node проверит ``if not retry_prompt: return False``.
        logger.warning(
            f"🛡 [issue 1777 / 1762] build_tool_retry_prompt: "
            f"unknown tool_name={tool_name!r}, skipping retry (defence-in-depth)"
        )
        return ""
    return (
        MUSIC_RETRY_PROMPT_PREFIX + f" {tool_name}, хотя пользователь "
        "явно попросил соответствующее действие. "
        + hint + " "
        "Запрос юзера: «" + (user_input or "") + "». "
        "Если и сейчас не вызовешь tool — пользователь не получит ответа."
    )


def looks_like_time_question(user_input: str) -> bool:
    """Issue #1777 — узкий детектор «вопрос про время».

    Отдельная функция для случая, когда LLM ответил на time-вопрос
    текстом, но в тексте есть маркеры времени (часы, минуты, AM/PM),
    что говорит о галлюцинации. Используется в dialogue_node для
    дополнительной диагностики (``WARN time_question_no_tool_call``),
    даже если общий ``detect_required_tool`` не сработал (например, LLM
    сам вписал «время» в длинный chit-chat).

    Не используется для retry (для retry есть ``detect_required_tool``).
    """
    if not user_input:
        return False
    low = user_input.lower()
    return any(kw in low for kw in (
        "который час",
        "сколько врем",
        "время в ",
        "время сейч",
        "который сейч",
        "сейчас врем",
        "какая дата",
        "какое число",
        "какой день",
        "what time",
        "what date",
    ))


# Issue #1777 — паттерны маркеров времени в тексте LLM-ответа. Используется
# для диагностического WARN'а ``time_question_no_tool_call`` (LLM
# галлюцинирует время, отвечая текстом с маркерами часы/минуты вместо
# вызова get_current_time). Чистая функция — вынесена в guards, чтобы
# тестироваться без ROS2-ноды.
_TIME_MARKER_RE = re.compile(
    r"(?:"
    r"\b\d{1,2}[:\.]\d{2}\b"             # 12:34 / 12.34
    r"|\b\d{1,2}\s*(?:час|часа|часов)\b"  # 12 часов
    r"|\b(?:утра|вечера|дня|ночи)\b"     # период
    r"|\bam|pm\b"                        # AM/PM
    r"|\b\d{1,2}\s*(?:am|pm)\b"          # 12 am
    r")",
    re.IGNORECASE,
)


def spoken_text_contains_time_marker(spoken: "str | None") -> bool:
    """Issue #1777 — есть ли в тексте ответа LLM маркеры времени?

    Маркеры: ``HH:MM``, ``N часов``, ``утра/вечера/дня/ночи``, ``AM/PM``.
    Используется в ``DialogueNode._run_turn.finally`` для логирования
    галлюцинаций (LLM отвечает на «который час?» текстом с маркерами
    времени без вызова ``get_current_time``). Не используется для retry —
    для retry есть :func:`detect_required_tool`.
    """
    return bool(spoken) and bool(_TIME_MARKER_RE.search(spoken))
