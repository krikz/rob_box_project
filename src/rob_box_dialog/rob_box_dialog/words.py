"""Сверка текста со словами :mod:`.knowledge` по границам слов (ADR-0150 §4.2, §6.3).

Граница слова здесь — не буква, не цифра и не дефис: «работает» не содержит вейка «бот», а «робота-диджея»
(речь о роботе, не к нему) не содержит вейка «робота». Подстрочного матча нет: «захватит» — не «хватит».
"""

from __future__ import annotations

import functools
import itertools
import re
from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, Optional, Sequence, Set, Tuple

from rob_box_dialog.knowledge import (
    ASK_LEAD_SKIP,
    DEFAULT_WAKE_WORDS,
    INTRO_ADJECTIVE_ENDINGS,
    INTRO_GREETING_WORDS,
    INTRO_NOT_A_NAME,
    KNOWN_FIRST_NAMES,
    LISTEN_ONLY_FORMS,
    NAME_CASE_MIN_STEM,
    NAME_CASE_STEMS,
    NAME_CASE_SURE_TAILS,
    NAME_CASE_TAILS,
    QUESTION_LEADS,
    REACH_END_OR_NEXT,
    REACH_NEXT,
    REPLY_INTRO_PHRASES,
    REPLY_NOT_A_NAME,
    REPLY_VERB_ENDINGS,
    REQUEST_LEADS,
    SELF_MEMORY_ASK,
    SILENCE_WORDS,
    STOP_LEAD_WORDS,
    STOP_QUESTION_PARTICLES,
    STOP_SCOPE,
    STOP_WORDS,
    TIER1_POLICY,
    VOICE_STEP,
    VOLUME_LEVEL_WORDS,
    VOLUME_STEMS,
    WAKE_SLIP_AMBIGUOUS,
    WAKE_SLIP_GATES,
    WAKE_SLIP_NEVER,
    WAKE_SLIP_PLURAL,
    WAKE_SLIP_POLITE,
    WAKE_SLIP_TARGETS,
    WAKE_SLIP_VARIANTS,
    SlipGate,
)

_LEFT = r"(?<![\w-])"
_RIGHT = r"(?![\w-])"
_TOKEN = re.compile(r"[\w-]+")
#: Знак между словами, который делит хвост стопа на части (``STOP_SCOPE``): «хватит читать, музыку не выключай».
_CLAUSE_BREAK = re.compile(r"[,.;:!?—–]")

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
    """Что за первое слово фразы: ``wake`` (искажённый вейк), ``gated`` (настоящее слово — будит, только если
    открыты ворота) или ``""``. Граница (#3975, #3996): «роботу» и неизвестные слова на расстоянии ≤ 1 (до 5 букв) /
    ≤ 2 от «робот/робокс/роббокс» — вейк; «работа/рада/роботы» — за воротами; остальные формы «робот» — никогда."""
    word = token.lower().replace("ё", "е")
    if word in WAKE_SLIP_NEVER:
        return ""
    if word in WAKE_SLIP_VARIANTS:
        return "wake"
    if word in WAKE_SLIP_AMBIGUOUS or word in WAKE_SLIP_PLURAL:
        return "gated"
    limit = 1 if len(word) <= 5 else 2
    near = len(word) >= 5 and word[0] == "р" and any(_distance(word, t) <= limit for t in WAKE_SLIP_TARGETS)
    return "wake" if near else ""


def _is_cue(token: str, gate: SlipGate) -> bool:
    return token in gate.forms or token.startswith(gate.stems)


def _gate_cue(tail: Sequence[str], gate: SlipGate) -> bool:
    """Есть ли в ``tail`` (всё, что идёт за ведущим словом) знак ворот: слово целиком из ``forms`` или начинающееся с
    основы из ``stems``. Ворота без знака пропускают всегда. ``gate.reach`` - где искать: в ближайших ``gate.window``
    словах (по умолчанию ``WAKE_SLIP_CUE_WINDOW``) либо только сразу за ведущим, не считая вежливых слов
    (``REACH_NEXT``); ``REACH_END_OR_NEXT`` пропускает и фразу, которая кончилась на ведущем."""
    if not (gate.forms or gate.stems):
        return True
    if gate.reach not in (REACH_NEXT, REACH_END_OR_NEXT):
        return any(_is_cue(t, gate) for t in tail[:gate.window])
    rest = tuple(itertools.dropwhile(lambda t: t in WAKE_SLIP_POLITE, tail))
    if not rest:
        return gate.reach == REACH_END_OR_NEXT
    return _is_cue(rest[0], gate)


def _gate_lead(token: str, gate: SlipGate) -> bool:
    """Ведущее слово ворот: слово целиком из ``leads`` либо основа из ``lead_stems`` с окончанием из ``lead_endings``
    («ускор» + «ь», «быстр» + «ее»; «ускоряются» и «быстро» - не просьба)."""
    return gate.open_lead or token in gate.leads or any(
        token.startswith(stem) and token[len(stem):] in gate.lead_endings for stem in gate.lead_stems)


def _gates_at(tokens: Sequence[str], music_playing: bool, music_only: bool = False) -> bool:
    """Открыта ли хоть одна из ворот ``WAKE_SLIP_GATES`` на фразе ``tokens``. Ворота правки музыки (``under_music``) в
    счёт идут, только пока музыка играет; ``music_only`` - смотрим одни они (за наречием-приставкой, :func:`_gate_prefix`).
    """
    if not tokens:
        return False
    for gate in WAKE_SLIP_GATES:
        if (gate.under_music and not music_playing) or (music_only and not gate.under_music):
            continue
        if not _gate_lead(tokens[0], gate):
            continue
        if _gate_prefix(tokens[1:], gate, music_playing) if gate.prefix else _gate_cue(tokens[1:], gate):
            return True
    return False


def _gate_prefix(tail: Sequence[str], gate: SlipGate, music_playing: bool) -> bool:
    """Ворота наречия-приставки (``gate.prefix``: «ещё», «снова»): дальше ворота правки музыки проверяются заново с
    того, что идёт после приставки и вежливых слов («ещё чуть быстрее» = «быстрее»). Приставка ничего не решает сама:
    «ещё быстрее нас считают» и «ещё громче всех» закрыты, как «быстрее нас считают» и «громче всех»."""
    rest = tuple(itertools.dropwhile(lambda t: t in WAKE_SLIP_POLITE, tail))
    return _gates_at(rest, music_playing, music_only=True)


def _gates_open(rest: str, music_playing: bool = False) -> bool:
    """Открыты ли ворота за искажённым словом: первое слово остатка (ё → е, как в таблицах) - ведущее ворот и знак ворот
    есть там, где велит ворота (``SlipGate.reach``: в ближайших ``window`` словах, сразу за ведущим, либо фраза
    кончилась на нём); ведущим может быть любое слово (``open_lead``) или наречие-приставка (``prefix``), за которой
    ворота правки проверяются заново. Ворота правки музыки и стопа (``under_music``) - только пока музыка играет.
    Единственное правило для «работа/рада/роботы»: таблицы в :data:`rob_box_dialog.knowledge.WAKE_SLIP_GATES`."""
    return _gates_at(name_tokens(rest), music_playing)


def slipped_wake_rest(text: str, wake_words: Optional[Sequence[str]] = None, *,
                      music_playing: bool = False) -> Optional[str]:
    """Первое слово фразы — искажённый STT вейк («роботу дали», «работа тишина», «рада слушай дальше», «роботы еще
    куплет добавь», «работа как меня зовут», под музыкой — «роботы ускорь темп»): остаток фразы после него, иначе
    ``None``. Только первый токен; тишину это не снимает (остаток идёт обычным путём). Вызывать, когда
    :func:`has_wake_word` не нашёл настоящего вейка. Вейков нет в таблице — допуска нет. ``music_playing`` - играет ли
    музыка (``SessionSnapshot.music_playing``): открывает ворота правки музыки (#3975)."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    if not any(t in words for t in WAKE_SLIP_TARGETS):
        return None
    match = _TOKEN.search(text or "")
    if match is None or (text or "")[:match.start()].strip(" ,.!?;:-–—«»\"'"):
        return None
    kind = _slip_kind(match.group(0))
    rest = (text or "")[match.end():].strip(" ,.!?;:-–—")
    if kind == "wake" or (kind == "gated" and _gates_open(rest, music_playing)):
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


def without_forms(text: str, forms: Sequence[str]) -> Tuple[str, bool]:
    """Текст без слов ровно этих форм (границы - как у :func:`has_form`) и были ли они в нём: «запомни мой голос» без
    формы «мой голос» - («запомни  », ``True``)."""
    if not forms:
        return text or "", False
    stripped, count = _pattern(tuple(forms), _RIGHT).subn(" ", text or "")
    return stripped, count > 0


def name_tokens(text: str) -> Tuple[str, ...]:
    """Слова фразы в нижнем регистре, ё → е (как в таблицах имён)."""
    return tuple(t for t in (w.lower().replace("ё", "е").strip("-") for w in _TOKEN.findall(text or "")) if t)


#: Обратное направление: слово с «не» перед ним значит противоположное («не громко» — тише).
_OPPOSITE_LEVEL = {"louder": "quieter", "quieter": "louder"}


def level_word(token: str) -> str:
    """Направление громкости, которое называет слово заказа музыки: ``louder``/``quieter`` или ``""``. Основа из
    ``VOLUME_STEMS`` («потише», «погромче», «убавь») либо слово целиком из ``VOLUME_LEVEL_WORDS`` («негромко», «тихо»);
    ё → е, регистр не важен. Единственное место, где слово называется словом громкости: тема заказа
    (``theme_grounding.heard_theme``) и уровень заказа (:func:`asked_level`) читают его."""
    word = (token or "").lower().replace("ё", "е")
    for intent in _OPPOSITE_LEVEL:
        if word in VOLUME_LEVEL_WORDS[intent] or word.startswith(VOLUME_STEMS[intent]):
            return intent
    return ""


def asked_level(text: str) -> str:
    """Уровень музыки, который просит заказ: ``quieter``/``louder`` или ``""`` (не просил, либо просил то и другое —
    выбирать нечем). «Поставь ровный ритм, только негромко» и «…только не громко» — ``quieter`` (слово с «не» перед ним
    значит обратное); «поставь потише» — ``quieter``. Слова — :func:`level_word`."""
    tokens = name_tokens(text)
    asked = {_OPPOSITE_LEVEL[kind] if i and tokens[i - 1] == "не" else kind
             for i, token in enumerate(tokens) if (kind := level_word(token))}
    return asked.pop() if len(asked) == 1 else ""


def words_in_order(heard: str, said: str) -> int:
    """Сколько слов ``heard`` встречаются в ``said`` в том же порядке — длина наибольшей общей подпоследовательности
    слов :func:`name_tokens` (нижний регистр, ё → е; знаки препинания не слова). «дорогие друзья слушайте и не
    говорите что не слышали» в «Дорогие друзья! Слушайте и не говорите, что не слышали! Наше ПТУ…» — 9: фраза STT
    повторяет речь робота слово в слово (эхо, #4145). Слово — целиком: «говоришь» не «говорите»."""
    words, spoken = name_tokens(heard), name_tokens(said)
    row = [0] * (len(spoken) + 1)
    for word in words:
        diagonal = 0
        for j, other in enumerate(spoken, 1):
            diagonal, row[j] = row[j], diagonal + 1 if word == other else max(row[j], row[j - 1])
    return row[-1]


def nominative_name(word: str) -> str:
    """Слово-имя (нижний регистр, ё → е, как в :func:`name_tokens`) в именительном падеже: «тамарой» → «тамара», «сашей»
    → «саша», «борисом» → «борис», «сергеем» → «сергей», «запиши как тамару» → «тамара», «запиши как бориса» → «борис»,
    «переименуй в сергея» → «сергей», «львом» → «лев» (таблицы ``NAME_CASE_TAILS`` / ``NAME_CASE_STEMS`` /
    ``NAME_CASE_SURE_TAILS``).

    Порядок: имя из словаря ``KNOWN_FIRST_NAMES`` уже именительное — как есть («валерия» — женское имя, не «валерий»);
    иначе форма, из которой слово получается по ``NAME_CASE_TAILS`` и которая есть в словаре (:func:`_dictionary_form`);
    иначе незнакомое имя — только по однозначному творительному «-ой/-ом» (:func:`_sure_form`); иначе слово как есть.
    Единственное место, где падеж имени приводится к именительному: фраза «зови меня Тамарой» и профиль «Тамара» — одно
    имя (ревью PR #4108, #4118)."""
    if word in KNOWN_FIRST_NAMES:
        return word
    return _dictionary_form(word) or _sure_form(word) or word


def _dictionary_form(word: str) -> Optional[str]:
    """Форма из словаря имён, из которой слово получается по ``NAME_CASE_TAILS``, или ``None``. Основа с беглой гласной
    («львом» → «льв») приводится через ``NAME_CASE_STEMS`` («лев») до сверки со словарём."""
    for tail, replacements in NAME_CASE_TAILS:
        stem = word[:-len(tail)]
        if not (word.endswith(tail) and stem):
            continue
        forms = (NAME_CASE_STEMS.get(stem + r, stem + r) for r in replacements)
        found = next((form for form in forms if form in KNOWN_FIRST_NAMES), None)
        if found is not None:
            return found
    return None


def _sure_form(word: str) -> Optional[str]:
    """Незнакомое имя (его нет в словаре и по ``NAME_CASE_TAILS`` оно не получилось) в однозначном творительном
    («аскаром» → «аскар», «никитой» → «никита»), иначе ``None``: слово остаётся как есть. Подходит ЛЮБОЕ слово на
    «-ой»/«-ом» с основой от ``NAME_CASE_MIN_STEM`` букв (``NAME_CASE_SURE_TAILS``) - беглую гласную здесь не
    восстановить: имена с ней (``NAME_CASE_STEMS``) разобраны в :func:`_dictionary_form` раньше."""
    for tail, replacement in NAME_CASE_SURE_TAILS.items():
        if word.endswith(tail) and len(word) - len(tail) >= NAME_CASE_MIN_STEM:
            return word[:-len(tail)] + replacement
    return None


def profile_name(name: Any) -> str:
    """Имя для профиля голоса (``register_speaker(name=…)``): без пробелов по краям. Одно слово - с заглавной и в
    именительном падеже (:func:`nominative_name`): имя, которое модель написала так, как услышала («Тамарой»,
    «Бориса»), не попадает в профиль падежом - профиль «Тамара», а не «Тамарой». ё остаётся: и в именительном
    («Алёна»), и при приведении падежа («Алёну» → «Алёна», а не «Алена»: в профиле одно написание имени). Несколько слов
    («Иван Петров») - как написаны: падеж не трогается и регистр не меняется, только первая буква заглавная. Пусто -
    пусто."""
    cleaned = str(name or "").strip()
    if len(cleaned.split()) != 1:
        return cleaned[:1].upper() + cleaned[1:]
    written = cleaned.lower()
    nominative = nominative_name(written.replace("ё", "е"))
    return _with_written_yo(written, nominative).capitalize()


def _with_written_yo(written: str, nominative: str) -> str:
    """Именительный с ё слова, как оно написано: «алёну» → «алена» → «алёна». ё сохраняется в общей основе слов (буквы
    сравниваются как ё → е); заменённое окончание и основа из ``NAME_CASE_STEMS`` («львом» → «лев») - от именительного."""
    common = 0
    while common < min(len(written), len(nominative)) and written[common].replace("ё", "е") == nominative[common]:
        common += 1
    return written[:common] + nominative[common:]


def name_key(name: Any) -> str:
    """Ключ сравнения имён: пробелы по краям, нижний регистр, ё → е и именительный падеж (:func:`nominative_name`) для
    одного слова. «Тамарой» и «Тамара» — один ключ; пустое имя — пустой ключ."""
    folded = str(name or "").strip().lower().replace("ё", "е")
    return folded if not folded or len(folded.split()) > 1 else nominative_name(folded)


def same_name(left: Any, right: Any) -> bool:
    """Одно ли имя (по ключу :func:`name_key`); пустое имя не равно ничему, даже пустому."""
    a, b = name_key(left), name_key(right)
    return bool(a) and a == b


def introduced_name(tokens: Sequence[str]) -> Optional[str]:
    """Имя, которым человек назвал себя: «меня зовут X» (X - любое слово от двух букв), «я X» (X - из словаря
    ``KNOWN_FIRST_NAMES``, перед «я» не «не») в любом месте фразы; X - в именительном падеже (:func:`nominative_name`:
    «зови меня Тамарой» → «тамара»). ``tokens`` - из :func:`name_tokens`.
    Единственная реализация: ею же пользуется ``reply_grammar`` (ответ на «как тебя зовут?»)."""
    for phrase in REPLY_INTRO_PHRASES:
        head = phrase.split()
        for i in range(len(tokens) - len(head)):
            if tuple(tokens[i:i + len(head)]) == tuple(head) and len(tokens[i + len(head)]) >= 2:
                return nominative_name(tokens[i + len(head)])
    for i in range(len(tokens) - 1):
        if tokens[i] == "я" and tokens[i + 1] in KNOWN_FIRST_NAMES and (i == 0 or tokens[i - 1] != "не"):
            return tokens[i + 1]
    return None


def _plausible_name(word: str) -> bool:
    """Незнакомое слово после «привет, я» похоже на имя: не служебное, не глагол, не прилагательное, не состояние."""
    return (len(word) >= 3 and word.replace("-", "").isalpha() and word not in REPLY_NOT_A_NAME
            and word not in INTRO_NOT_A_NAME and not word.endswith(REPLY_VERB_ENDINGS + INTRO_ADJECTIVE_ENDINGS))


def _greeted_name(tokens: Sequence[str]) -> Optional[str]:
    """«Привет, я Аскар»: «я» сразу после приветствия и за ним правдоподобное незнакомое имя - оно и возвращается."""
    return next((tokens[i + 1] for i in range(1, len(tokens) - 1)
                 if tokens[i] == "я" and tokens[i - 1] in INTRO_GREETING_WORDS and _plausible_name(tokens[i + 1])), None)


def introduces_self(text: str) -> bool:
    """Назвал ли человек себя в реплике («здравствуй, я Саша», «привет, я Борис», «меня зовут …», «это я, Валера»).

    Граница (ADR-0148, решает код, не LLM): в STT имена пишутся строчно, заглавной нет, поэтому опираться можно на
    позицию после «я». (1) «меня зовут / зови меня / моё имя» - явный оборот. (2) «я X», где X в словаре имён
    ``KNOWN_FIRST_NAMES``: «я Саша» - да; «я думаю», «я пью чай», «я устал», «я дома» - нет (глагол и состояние в словарь
    не входят). (3) «привет/здравствуй, я X» с незнакомым X, если X не служебное слово, не глагол (окончания
    ``REPLY_VERB_ENDINGS``), не прилагательное и не состояние (``INTRO_NOT_A_NAME``). Редкое имя без приветствия не
    узнаётся - это остаётся на LLM и на ``tools_view.widen_for`` (``register_speaker`` вне среза), честная граница:
    словарь даёт точность, а не полноту."""
    return self_intro_name(text) is not None


def self_intro_name(text: str) -> Optional[str]:
    """Имя, которым человек назвал себя в реплике, или ``None``: то самое, что находит :func:`introduces_self` («меня
    зовут X», «я X» из словаря имён, «привет, я X»), - строчное, ё → е, как в :func:`name_tokens`. Единственная
    реализация: гейт ``register_speaker`` (#2925) сверяет с ним имя, которое модель пишет в профиль голоса."""
    tokens = name_tokens(text)
    return introduced_name(tokens) or _greeted_name(tokens)


def asks_to_listen(text: str) -> bool:
    """Просит ли фраза робота слушать и не встревать (#4179): формы ``knowledge.LISTEN_ONLY_FORMS`` целиком («ты просто
    слушай и не влезай») или слово тишины ``knowledge.SILENCE_WORDS`` («помолчи»). «Не молчи», «слушай, а как…» - нет."""
    return has_form(text, LISTEN_ONLY_FORMS) or has_word(text, SILENCE_WORDS)


def asks_own_memory(text: str) -> bool:
    """Спрашивает ли человек о памяти о самом себе (таблица ``knowledge.SELF_MEMORY_ASK``, ревью PR #4055): глагол
    памяти к роботу и слово о себе в одной реплике («что ты обо мне помнишь?», «помнишь, что я …») или вопрос целиком
    («как меня зовут?»). Память как условие другой просьбы («учти то, что ты про меня знаешь») - нет. Слова - как у
    :func:`name_tokens` (нижний регистр, ё → е, без знаков): «помнишь, что я» и «помнишь что я» - одно."""
    said = " ".join(name_tokens(text))
    if has_word(said, SELF_MEMORY_ASK["condition"]):
        return False
    return has_form(said, SELF_MEMORY_ASK["whole"]) or (
        has_word(said, SELF_MEMORY_ASK["verbs"]) and has_form(said, SELF_MEMORY_ASK["self"]))


def asks_question(text: str, wake_words: Optional[Sequence[str]] = None) -> bool:
    """Реплика — вопрос или справка, а не просьба сделать (ревью #4153; таблицы ``knowledge.QUESTION_LEADS``): первое
    слово после вейка и вводных (``ASK_LEAD_SKIP``: «а», «ну», «скажи», «подскажи») — вопросительное («как управлять
    роботом», «Робот, скажи сколько секунд займёт…»), или в реплике знак «?» или частица «ли» не после
    просьбы-модальности (``REQUEST_LEADS``: «можешь покекать роботом?» — просьба; «ты себя считаешь человеком или
    роботом?» — вопрос). Слова — как у :func:`name_tokens`."""
    rest = strip_wake_word(text, wake_words)
    tokens = name_tokens(rest)
    lead = next((token for token in tokens if token not in ASK_LEAD_SKIP), "")
    if lead in QUESTION_LEADS:
        return True
    marked = "?" in rest or bool(set(tokens) & set(STOP_QUESTION_PARTICLES))
    return marked and lead not in REQUEST_LEADS


@dataclass(frozen=True)
class StopScope:
    """Охват ясного стопа (#3994, таблица ``knowledge.STOP_SCOPE``): после стоп-слова названа речь робота
    («хватит читать»), названа музыка, которую погасить («хватит музыки»), есть оговорка «музыку оставь»."""

    speech: bool = False
    music: bool = False
    keep_music: bool = False


NO_SCOPE = StopScope()


@dataclass(frozen=True)
class StopCall:
    """Разбор стоп-команды (В19): вердикт, стоп-слово и текст ПОСЛЕ него («хватит, спой другую» → «спой другую»).

    ``word``/``rest``/``words``/``scope`` заполнены только у ясного вердикта; у спорного ``word`` — первое стоп-слово
    фразы. ``words`` — все стоп-слова ведущей серии («всё, хватит, замолчи» → «хватит», «замолчи»): вид команды решает
    самое сильное из них (тишина — только по слову из ``SILENCE_WORDS``). ``scope`` — что названо после стопа
    (:func:`stop_scope`), ``rest`` — только непокрытая им часть (новая просьба).
    """

    verdict: str
    word: str = ""
    rest: str = ""
    words: Tuple[str, ...] = ()
    scope: StopScope = NO_SCOPE


Span = Tuple[str, int, int]  # слово (нижний регистр, ё → е), начало, конец


def stop_scope(rest: str) -> Tuple[StopScope, str]:
    """Хвост ясного стопа → ``(охват, остаток)`` по закрытой грамматике ``knowledge.STOP_SCOPE`` (#3994, решает код).

    Хвост делится на части союзами и знаками препинания; части съедаются по порядку, пока каждая покрыта таблицей
    (объекты, оговорка «оставь», служебные). Первая непокрытая часть и всё после неё — остаток; ни одна не покрыта —
    остаток весь хвост, как был. «читать, а музыку оставь» → речь + оставить музыку, «»; «музыки» → музыка, «»;
    «спой другую» → ничего, «спой другую»; «читать, а музыку сделай потише» → речь, «музыку сделай потише».
    Оговорка в остатке тоже оставляет музыку: «спой другую, а музыку оставь» → оставить музыку, «спой другую, а
    музыку оставь» (просьба целиком — LLM)."""
    text = rest or ""
    speech = music = keep = False
    parts = clauses(text, STOP_SCOPE.joins)
    for n, clause in enumerate(parts):
        parsed = _clause_scope(tuple(token for token, _, _ in clause))
        if parsed is None:
            tail = text[clause[0][1]:] if n else text
            keep = keep or _kept_later(parts[n + 1:])
            return StopScope(speech, music, keep), tail.strip(" ,.!?;:-–—")
        named, kept = parsed
        speech = speech or "speech" in named
        keep = keep or kept
        music = music or ("music" in named and not kept)  # с оговоркой названная музыка — что оставить
    return StopScope(speech, music, keep), ""


def clauses(text: str, joins: Sequence[str]) -> Tuple[Tuple[Span, ...], ...]:
    """Части фразы (одна реализация на хвост стопа и шаг голоса, #4074): граница — союз из ``joins`` (сам он ни в
    одну часть не входит) или знак препинания между словами. Слово части — ``(слово в нижнем регистре, ё → е,
    начало, конец)``: по началу первого слова части берётся остаток фразы."""
    spans = [(m.group(0).lower().replace("ё", "е"), m.start(), m.end()) for m in _TOKEN.finditer(text or "")]
    return _clauses(text or "", spans, joins)


def _clauses(text: str, spans: Sequence[Span], joins: Sequence[str]) -> Tuple[Tuple[Span, ...], ...]:
    out, current, prev_end = [], [], 0
    for span in spans:
        token, start, end = span
        join = token in joins
        if current and (join or _CLAUSE_BREAK.search(text[prev_end:start])):
            out.append(tuple(current))
            current = []
        prev_end = end
        if not join:
            current.append(span)
    return tuple(out + [tuple(current)] if current else out)


def _kept_later(clauses: Sequence[Tuple[Span, ...]]) -> bool:
    """Есть ли среди частей остатка покрытая часть с оговоркой («…, а музыку оставь» после новой просьбы)."""
    parsed = (_clause_scope(tuple(token for token, _, _ in clause)) for clause in clauses)
    return any(part is not None and part[1] for part in parsed)


def _clause_scope(tokens: Sequence[str]) -> Optional[Tuple[FrozenSet[str], bool]]:
    """Часть хвоста по таблице: ``(названные объекты, есть ли оговорка «оставь»)``; чужое слово или дополнение без
    глагола своего объекта («рэпа» без «читать») — ``None``."""
    found: Dict[str, Set[str]] = {"stem": set(), "verb": set(), "complement": set()}
    keep, i = False, 0
    while i < len(tokens):
        phrase = next((p for p in STOP_SCOPE.keep if tuple(tokens[i:i + len(p)]) == p), None)
        if phrase is not None:
            keep, i = True, i + len(phrase)
            continue
        what, role = _stop_object(tokens[i])
        if not what and tokens[i] not in STOP_SCOPE.filler:
            return None
        found[role or "stem"].update((what,) if what else ())
        i += 1
    if not found["complement"] <= found["verb"]:
        return None
    return frozenset(found["stem"] | found["verb"]), keep


def _stop_object(token: str) -> Tuple[str, str]:
    """``(объект стопа — ключ STOP_SCOPE.objects, роль слова: stem/verb/complement)`` или ``("", "")``: слово с основы
    объекта; глагол — кроме повелительной формы («говори», «играй» — новая просьба); дополнение («рэп»)."""
    imperative = token.endswith(STOP_SCOPE.imperative_endings)
    for what, obj in STOP_SCOPE.objects.items():
        if token.startswith(obj.stems):
            return what, "stem"
        if token.startswith(obj.verbs) and not imperative:
            return what, "verb"
        if token.startswith(obj.complements):
            return what, "complement"
    return "", ""


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
    """То же, что :func:`stop_verdict`, плюс стоп-слово, его охват (:func:`stop_scope`) и остаток команды после них
    (одна реализация разбора)."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    part = _command_part(text, words)
    spans = [(m.group(0).lower(), m.end()) for m in _TOKEN.finditer(part)]
    tokens = [t for t, _ in spans]
    skip = set(STOP_LEAD_WORDS) | {w.lower() for w in words}
    lead = next((i for i, t in enumerate(tokens) if t not in skip), None)
    if lead is not None and _clear_at(tokens, lead):
        scope, rest = stop_scope(_rest(part, spans, lead, skip))
        return StopCall(STOP_CLEAR, tokens[lead], rest, _stop_run(tokens, lead, skip), scope)
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
    """Ведущее слово — стоп-слово, и за ним не вопросительная частица («хватит ли …») и не манера речи робота
    («хватит тянуть»: просьба о темпе, а не стоп — #4074, :func:`speech_manner`; покрытую фразу решает шаг голоса
    ``grammar``, остальное — спорный стоп). «Стой, тороплюсь», «стоп, тяни» — ясный стоп: ведущее «стой/стоп» —
    не слово манеры."""
    after = tokens[lead + 1] if lead + 1 < len(tokens) else ""
    return _is_stop(tokens[lead]) and after not in STOP_QUESTION_PARTICLES and speech_manner(tokens, lead) is None


def speech_manner(tokens: Sequence[str], lead: int) -> Optional[Tuple[Tuple[str, str], int]]:
    """Манера речи робота с ведущим словом (#4074, ``VOICE_STEP``): «хватит тянуть», «не тараторь», «хватит так
    тараторить» → ``((тул, действие), индекс слова манеры)`` — шаг, который её исправляет; иначе ``None``. Слова —
    в нижнем регистре; манера — точной формой (инфинитив или повелительная), а не основой: «хватит тянет» — не она.
    Одна реализация на грамматику шага, ясность стопа и срез тулов."""
    if lead >= len(tokens) or tokens[lead] not in VOICE_STEP.manner_leads:
        return None
    at = lead + 1
    if at < len(tokens) - 1 and tokens[at] in VOICE_STEP.manner_gap:
        at += 1
    step = VOICE_STEP.manners.get(tokens[at]) if at < len(tokens) else None
    return (step, at) if step is not None else None


def has_speech_manner(text: str) -> bool:
    """Есть ли во фразе манера речи робота с ведущим словом (:func:`speech_manner`): «хватит тянуть резину» — тоже
    (срез тулов открывает голос, решает модель)."""
    tokens = [t.lower().replace("ё", "е") for t in _TOKEN.findall(text or "")]
    return any(speech_manner(tokens, i) is not None for i in range(len(tokens)))


def wake_command(text: str, wake_words: Optional[Sequence[str]] = None) -> str:
    """Команда фразы с вейком — то, что ПОСЛЕ него (вейк в конце фразы — перед ним, вейка нет — вся фраза), без
    крайних знаков: слова перед вейком — хвост речи самого робота, который микрофон склеил с фразой человека (n811
    27.10: «четыре недели робот хватит читать а музыку оставь» под рэп). То же правило, что у :func:`stop_command`,
    — запасной разбор Tier-1 реплики с микрофона, когда фраза целиком не Tier-1 или спорна
    (``address.Addressed.command``, ``turn.tier1_of``)."""
    words = DEFAULT_WAKE_WORDS if wake_words is None else tuple(wake_words)
    return _command_part(text, words).strip(" ,.!?;:-–—")


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
