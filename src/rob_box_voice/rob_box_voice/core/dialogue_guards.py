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
    execute_music_code).

    The LLM frequently concludes «музыка уже играет» from the dialogue
    history (previous runs/songs) and returns ``done`` without calling
    ``execute_music_code``. This prompt explicitly resets that assumption
    and demands the tool call.
    """
    return (
        "[CRITICAL] В прошлом цикле ты НЕ вызвал execute_music_code, "
        "хотя пользователь ЯВНО попросил музыку/диджея. "
        "Музыка сейчас НЕ играет — предыдущие треки уже остановлены. "
        "Вызови execute_music_code (Renardo code) ДО любого speak_text. "
        "Запрос пользователя: "
        + (user_input or "")
        + " Если ты снова не вызовешь execute_music_code, "
        "цикл будет считаться пустым."
    )
