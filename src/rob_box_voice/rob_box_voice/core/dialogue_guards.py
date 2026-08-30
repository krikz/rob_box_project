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
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Music tool names — single source of truth
# ---------------------------------------------------------------------------

#: MCP tools that start Renardo playback.
#:
#: SSoT on purpose. This list used to be hardcoded separately in
#: ``DialogueNode`` (``_music_starters``) and in ``MusicGuard``
#: (``_music_started``), and every new music tool had to be added to both.
#: It never was — which is how ``compose_music`` shipped and got
#: auto-stopped 1.5 s after it started: ``dialogue_node`` did not recognise
#: it as a music starter, armed ``_pending_music_cleanup``, then fired
#: ``music_cleanup`` at turn end because no TTS batch existed yet. The
#: comment above ``_music_starters`` already recorded the same accident
#: happening once before with ``load_track`` / ``set_dj_mode``.
#:
#: Add new Renardo-side music tools HERE and nowhere else.
RENARDO_MUSIC_TOOLS: frozenset = frozenset({
    "execute_music_code",
    "compose_music",
})

#: MiniMax mp3 playback. Separate set: these do not go through Renardo, so
#: the Renardo cleanup path does not apply to them, but for the retry guard
#: they still count as "music started".
GENERATED_MUSIC_TOOLS: frozenset = frozenset({
    "generate_music",
    "gen_play_from_library",
})

#: Tools that stop playback. A stop-command turn that calls none of these
#: has not stopped anything — see ``MusicGuardVerdictKind.FORCE_STOP``.
#: ``set_dj_mode`` counts because «хватит диджеить» is satisfied by turning
#: DJ mode off even when no Renardo pattern was running.
MUSIC_STOP_TOOLS: frozenset = frozenset({
    "stop_music",
    "set_dj_mode",
})

#: Tools that put existing playback into a mode rather than starting it.
#: They keep music alive for the cleanup logic but must NOT satisfy the DJ
#: retry guard — calling ``set_dj_mode`` without playing anything is
#: precisely the Bug-B failure it is there to catch.
MUSIC_MODE_TOOLS: frozenset = frozenset({
    "load_track",
    "set_dj_mode",
    "set_vibe_preset",
})


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


# 🔴 FIX (live 30.08, vision-pi 12:52-12:57): «продолжай развивать этот бит»,
# «переходи с лоу в небольшой джангл» — юзер просит РАЗВИТЬ уже играющую
# музыку. Ни одна подстрока из ``MUSIC_GUARD_KEYWORDS`` в них не встречается
# (там нет ни «сыграй», ни «включи трек»), поэтому Bug C молчал, а LLM
# отвечала «Добавил новые слои в техно-бит.» / «Бит перешёл в джангл.» с
# ``tools=[]`` — то есть НИЧЕГО не игралось, робот просто рассказывал про
# музыку словами.
#
# Ловим это парой «глагол-продолжения + музыкальное существительное» в
# любом порядке. Пара нужна именно как пара: отдельное «бит» ловит «битва»
# и «орбита», отдельное «продолжай» — «продолжай маршрут».
_MUSIC_CONTINUE_VERBS: str = (
    r"развива|разверни|продолж|переход|перейд|усил|добав|убер|смен|поменя|"
    r"ускор|замедл|раскач|наращ|нарасти|дораб|доработ"
)
_MUSIC_NOUNS: str = (
    r"бит|мелоди|музык|трек|ритм|грув|луп|бас|барабан|темп|аккорд|парти|"
    r"джангл|техно|хаус|дабстеп|эмбиент|амбиент|драм-?н-?бейс|драмн?бейс|"
    r"хип-?хоп|лоу-?фай|lofi|транс|фанк|регги|брейкбит|синт"
)

#: Пара «глагол + существительное» в обоих порядках. ``\w*`` после каждой
#: основы покрывает падежи/виды («развивай», «развивать», «развивая»;
#: «бит», «бита», «битом»).
MUSIC_CONTINUATION_RE = re.compile(
    rf"(?:(?:{_MUSIC_CONTINUE_VERBS})\w*.{{0,40}}?(?:{_MUSIC_NOUNS})\w*)"
    rf"|(?:(?:{_MUSIC_NOUNS})\w*.{{0,40}}?(?:{_MUSIC_CONTINUE_VERBS})\w*)",
    re.IGNORECASE,
)


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
    # 🔴 FIX (live 30.08): вопрос о состоянии — не запрос на исполнение.
    # «играет ли сейчас музыка» совпадал по «музык» и гнал Bug D в ретрай
    # ради того же самого текстового ответа. См. ``is_state_question``.
    if is_state_question(user_input):
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
    # 🔴 FIX (live 30.08): «продолжай развивать этот бит» / «переходи в
    # джангл» — просьба развить уже играющую музыку. Подстрочных ключей на
    # неё нет, поэтому сначала пробуем пару «глагол + муз. существительное».
    if MUSIC_CONTINUATION_RE.search(low):
        if logger is not None:
            logger.debug(
                f"🎵 [music_guard] user_input={user_input!r} matched "
                f"MUSIC_CONTINUATION_RE → wants_music=True"
            )
        return True
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
# 🔴 Issue #992 Bug E — «сделал» без единого тула.
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
        if not rule.claim_re.search(spoken):
            continue
        if called & rule.tools:
            continue
        return rule
    return None


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
