"""Сверка текста со словами :mod:`.knowledge` по границам слов (ADR-0150 §4.2, §6.3).

Граница слова здесь — не буква, не цифра и не дефис: «работает» не содержит вейка «бот», а «робота-диджея»
(речь о роботе, не к нему) не содержит вейка «робота». Подстрочного матча нет: «захватит» — не «хватит».
"""

from __future__ import annotations

import functools
import re
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from rob_box_dialog.knowledge import (
    DEFAULT_WAKE_WORDS,
    INTRO_ADJECTIVE_ENDINGS,
    INTRO_GREETING_WORDS,
    INTRO_NOT_A_NAME,
    KNOWN_FIRST_NAMES,
    REPLY_INTRO_PHRASES,
    REPLY_NOT_A_NAME,
    REPLY_VERB_ENDINGS,
    STOP_LEAD_WORDS,
    STOP_QUESTION_PARTICLES,
    STOP_WORDS,
    TIER1_POLICY,
    WAKE_SLIP_AMBIGUOUS,
    WAKE_SLIP_COMMAND_LEADS,
    WAKE_SLIP_NEVER,
    WAKE_SLIP_TARGETS,
    WAKE_SLIP_VARIANTS,
)

_LEFT = r"(?<![\w-])"
_RIGHT = r"(?![\w-])"
_TOKEN = re.compile(r"[\w-]+")

#: Вердикты :func:`stop_verdict` (В19): ясная стоп-команда, спорная (стоп-слово внутри другой команды), нет стопа.
STOP_CLEAR = "clear"
STOP_DISPUTED = "disputed"
STOP_NONE = "none"


@functools.lru_cache(maxsize=64)
def _pattern(words: Tuple[str, ...], tail: str) -> "re.Pattern[str]":
    alternation = "|".join(re.escape(w) for w in words)
    return re.compile(f"{_LEFT}({alternation}){tail}", re.IGNORECASE)


def has_wake_word(text: str, wake_words: Optional[Sequence[str]] = None) -> bool:
    """Есть ли в ``text`` вейк-слово отдельным словом. Пустой список — режим «без вейка»: всё адресовано."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    if not words:
        return True
    return bool(_pattern(words, _RIGHT).search(text or ""))


def strip_wake_word(text: str, wake_words: Optional[Sequence[str]] = None) -> str:
    """Убрать ПЕРВОЕ вейк-слово (в любом месте фразы) вместе с пунктуацией и пробелами после него."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    if not words:
        return (text or "").strip()
    return _pattern(words, _RIGHT + r"[.,!?\s]*").sub("", text or "", count=1).strip()


def _distance(a: str, b: str) -> int:
    """Расстояние Левенштейна (вставка, удаление, замена по 1)."""
    row = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        prev, row[0] = row[0], i
        for j, cb in enumerate(b, 1):
            prev, row[j] = row[j], min(row[j] + 1, row[j - 1] + 1, prev + (ca != cb))
    return row[-1]


def _slip_kind(token: str) -> str:
    """Что за первое слово фразы: ``wake`` (искажённый вейк), ``gated`` (настоящее слово — будит только перед
    командой) или ``""``. Граница (#3975): «роботу» и неизвестные слова на расстоянии ≤ 1 (до 5 букв) / ≤ 2 от
    «робот/робокс/роббокс» — вейк; «работа/рада» — только с командой следом; формы «роботы» — никогда."""
    word = token.lower().replace("ё", "е")
    if word in WAKE_SLIP_NEVER:
        return ""
    if word in WAKE_SLIP_VARIANTS:
        return "wake"
    if word in WAKE_SLIP_AMBIGUOUS:
        return "gated"
    limit = 1 if len(word) <= 5 else 2
    near = len(word) >= 5 and word[0] == "р" and any(_distance(word, t) <= limit for t in WAKE_SLIP_TARGETS)
    return "wake" if near else ""


def slipped_wake_rest(text: str, wake_words: Optional[Sequence[str]] = None) -> Optional[str]:
    """Первое слово фразы — искажённый STT вейк («роботу дали», «работа тишина», «рада слушай дальше»): остаток
    фразы после него, иначе ``None``. Только первый токен; тишину это не снимает (остаток идёт обычным путём).
    Вызывать, когда :func:`has_wake_word` не нашёл настоящего вейка. Вейков нет в таблице — допуска нет."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    if not any(t in words for t in WAKE_SLIP_TARGETS):
        return None
    match = _TOKEN.search(text or "")
    if match is None or (text or "")[:match.start()].strip(" ,.!?;:-–—«»\"'"):
        return None
    kind = _slip_kind(match.group(0))
    rest = (text or "")[match.end():].strip(" ,.!?;:-–—")
    lead = (_TOKEN.search(rest).group(0).lower() if _TOKEN.search(rest) else "")
    if kind == "wake" or (kind == "gated" and lead in WAKE_SLIP_COMMAND_LEADS):
        return rest
    return None


def has_word(text: str, stems: Sequence[str]) -> bool:
    """Есть ли слово, НАЧИНАЮЩЕЕСЯ с одной из основ (``SILENCE_WORDS``: «помолч» → «помолчи», «помолчите»)."""
    if not stems:
        return False
    return bool(_pattern(tuple(stems), r"[\w-]*").search(text or ""))


def has_form(text: str, forms: Sequence[str]) -> bool:
    """Есть ли слово РОВНО одной из форм (не основа: «бас» - не «басня»)."""
    return bool(forms) and bool(_pattern(tuple(forms), _RIGHT).search(text or ""))


def name_tokens(text: str) -> Tuple[str, ...]:
    """Слова фразы в нижнем регистре, ё → е (как в таблицах имён)."""
    return tuple(t for t in (w.lower().replace("ё", "е").strip("-") for w in _TOKEN.findall(text or "")) if t)


def introduced_name(tokens: Sequence[str]) -> Optional[str]:
    """Имя, которым человек назвал себя: «меня зовут X» (X - любое слово от двух букв), «я X» (X - из словаря
    ``KNOWN_FIRST_NAMES``, перед «я» не «не») в любом месте фразы. ``tokens`` - из :func:`name_tokens`.
    Единственная реализация: ею же пользуется ``reply_grammar`` (ответ на «как тебя зовут?»)."""
    for phrase in REPLY_INTRO_PHRASES:
        head = phrase.split()
        for i in range(len(tokens) - len(head)):
            if tuple(tokens[i:i + len(head)]) == tuple(head) and len(tokens[i + len(head)]) >= 2:
                return tokens[i + len(head)]
    for i in range(len(tokens) - 1):
        if tokens[i] == "я" and tokens[i + 1] in KNOWN_FIRST_NAMES and (i == 0 or tokens[i - 1] != "не"):
            return tokens[i + 1]
    return None


def _plausible_name(word: str) -> bool:
    """Незнакомое слово после «привет, я» похоже на имя: не служебное, не глагол, не прилагательное, не состояние."""
    return (len(word) >= 3 and word.replace("-", "").isalpha() and word not in REPLY_NOT_A_NAME
            and word not in INTRO_NOT_A_NAME and not word.endswith(REPLY_VERB_ENDINGS + INTRO_ADJECTIVE_ENDINGS))


def _greeted_stranger(tokens: Sequence[str]) -> bool:
    """«Привет, я Аскар»: «я» сразу после приветствия и за ним правдоподобное незнакомое имя."""
    return any(tokens[i] == "я" and tokens[i - 1] in INTRO_GREETING_WORDS and _plausible_name(tokens[i + 1])
               for i in range(1, len(tokens) - 1))


def introduces_self(text: str) -> bool:
    """Назвал ли человек себя в реплике («здравствуй, я Саша», «привет, я Борис», «меня зовут …», «это я, Валера»).

    Граница (ADR-0148, решает код, не LLM): в STT имена пишутся строчно, заглавной нет, поэтому опираться можно на
    позицию после «я». (1) «меня зовут / зови меня / моё имя» - явный оборот. (2) «я X», где X в словаре имён
    ``KNOWN_FIRST_NAMES``: «я Саша» - да; «я думаю», «я пью чай», «я устал», «я дома» - нет (глагол и состояние в словарь
    не входят). (3) «привет/здравствуй, я X» с незнакомым X, если X не служебное слово, не глагол (окончания
    ``REPLY_VERB_ENDINGS``), не прилагательное и не состояние (``INTRO_NOT_A_NAME``). Редкое имя без приветствия не
    узнаётся - это остаётся на LLM и на ``tools_view.widen_for`` (``register_speaker`` вне среза), честная граница:
    словарь даёт точность, а не полноту."""
    tokens = name_tokens(text)
    return introduced_name(tokens) is not None or _greeted_stranger(tokens)


@dataclass(frozen=True)
class StopCall:
    """Разбор стоп-команды (В19): вердикт, стоп-слово и текст ПОСЛЕ него («хватит, спой другую» → «спой другую»).

    ``word``/``rest``/``words`` заполнены только у ясного вердикта; у спорного ``word`` — первое стоп-слово фразы.
    ``words`` — все стоп-слова ведущей серии («всё, хватит, замолчи» → «хватит», «замолчи»): вид команды решает
    самое сильное из них (тишина — только по слову из ``SILENCE_WORDS``).
    """

    verdict: str
    word: str = ""
    rest: str = ""
    words: Tuple[str, ...] = ()


def stop_verdict(text: str, wake_words: Optional[Sequence[str]] = None) -> str:
    """Стоп-команда ли фраза (В19, Шифу 20.10): :data:`STOP_CLEAR`, :data:`STOP_DISPUTED` или :data:`STOP_NONE`.

    Ясная — первое слово после вейка (ведущие «всё/ну/да…» из ``STOP_LEAD_WORDS`` и сами вейки не в счёт) —
    стоп-слово: «робот, стоп», «всё, хватит, спой другую». Спорная — стоп-слово дальше по фразе: «спой песню про
    стоп», «покажи анимацию стоп»; её решает Chooser (``grammar`` → хост, PR-6), речь до решения не рвётся. Одна
    функция на перебивание речи (stt_node → ``/dialog/interrupt_request``) и на Tier-1 грамматику.

    Команда — то, что ПОСЛЕ вейка: слова перед ним — хвост речи самого робота, который микрофон склеил с фразой
    человека (живой лог 21.10: «старой робот хватит» на сказке «…у корней старой сосны»). Вейк в конце фразы
    («хватит, робот») — команда перед ним.
    """
    return stop_command(text, wake_words).verdict


def stop_command(text: str, wake_words: Optional[Sequence[str]] = None) -> StopCall:
    """То же, что :func:`stop_verdict`, плюс стоп-слово и остаток команды после него (одна реализация разбора)."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    part = _command_part(text, words)
    spans = [(m.group(0).lower(), m.end()) for m in _TOKEN.finditer(part)]
    tokens = [t for t, _ in spans]
    skip = set(STOP_LEAD_WORDS) | {w.lower() for w in words}
    lead = next((i for i, t in enumerate(tokens) if t not in skip), None)
    if lead is not None and _clear_at(tokens, lead):
        return StopCall(STOP_CLEAR, tokens[lead], _rest(part, spans, lead, skip), _stop_run(tokens, lead, skip))
    first = next((t for t in tokens if _is_stop(t)), "")
    return StopCall(STOP_DISPUTED, first, words=(first,)) if first else StopCall(STOP_NONE)


def policy_word(text: str, wake_words: Optional[Sequence[str]] = None) -> Optional[str]:
    """Tier-1 переопределение политики планировщика (§5.6.4): ключ ``TIER1_POLICY`` или ``None``.

    Ясный стоп (:func:`stop_command`) — ``halt`` для форм «стой», иначе ``stop``; дальше «потом/после/когда
    закончишь» (``later``) и «прямо сейчас»/«сейчас» первым словом команды (``now``) — точными формами по границам
    слов: «последний» — не «после»."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    stop = stop_command(text, words)
    if stop.verdict == STOP_CLEAR:
        halt = TIER1_POLICY["halt"].words
        return "halt" if all(part in halt for part in stop.word.split("-")) else "stop"
    part = _command_part(text, words)
    tokens = [t.lower() for t in _TOKEN.findall(part)]
    skip = set(STOP_LEAD_WORDS) | {w.lower() for w in words}
    lead = next((t for t in tokens if t not in skip), "")
    for key in ("later", "now"):
        group = TIER1_POLICY[key]
        anywhere = tuple(w for w in group.words if w not in group.lead_only)
        if (anywhere and _pattern(anywhere, _RIGHT).search(part)) or lead in group.lead_only:
            return key
    return None


def _stop_run(tokens: Sequence[str], lead: int, skip: set) -> Tuple[str, ...]:
    """Стоп-слова ведущей серии: от ведущего до первого слова новой команды (вводные не в счёт)."""
    tail = next((i for i in range(lead + 1, len(tokens)) if not (_is_stop(tokens[i]) or tokens[i] in skip)),
                len(tokens))
    return tuple(t for t in tokens[lead:tail] if _is_stop(t))


def _rest(part: str, spans: Sequence[Tuple[str, int]], lead: int, skip: set) -> str:
    """Текст после ясного стопа без повторных стоп-слов и вводных («стоп, стоп, ну давай …» → «давай …»)."""
    tail = next((i for i in range(lead + 1, len(spans)) if not (_is_stop(spans[i][0]) or spans[i][0] in skip)), None)
    return part[spans[tail - 1][1]:].strip(" ,.!?;:-–—") if tail is not None else ""


def _clear_at(tokens: Sequence[str], lead: int) -> bool:
    """Ведущее слово — стоп-слово, и за ним не вопросительная частица («хватит ли …»)."""
    after = tokens[lead + 1] if lead + 1 < len(tokens) else ""
    return _is_stop(tokens[lead]) and after not in STOP_QUESTION_PARTICLES


def _command_part(text: str, words: Tuple[str, ...]) -> str:
    """Текст после первого вейка; вейк в конце фразы или без вейка — вся фраза без него."""
    match = _pattern(words, _RIGHT).search(text or "") if words else None
    if match is None:
        return text or ""
    after = (text or "")[match.end():]
    return after if _TOKEN.search(after) else (text or "")[:match.start()]


def _is_stop(token: str) -> bool:
    """Стоп-слово целиком; «стоп-стоп» — тоже (все части через дефис — стоп-слова), «стоп-кран» — нет."""
    return all(part in STOP_WORDS for part in token.split("-")) if token else False
