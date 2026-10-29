"""Тема DJ-сета из СЛОВ ЧЕЛОВЕКА этого хода, а не из истории диалога (ADR-0148, живой лог 06.10 18:04–18:05).

Человек повторил фразу «Ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки, замути сэт на
30 минут», а LLM вызвала ``dj_set(theme='Увертюра 1812, мегасет для геймеров: Марио, Аладдин, Тетрис …')`` — тему из
СТАРОЙ реплики истории: классика пропала. Решает код: текст реплики хода едет скрытым аргументом ``heard_text``
(``llm_adapter.TURN_CONTEXT_ARGS``; присланное LLM вырезается). Тема LLM остаётся, если делит со словами реплики
хоть одну значимую основу (:func:`search.content_stems`: перефраз, сокращение, жанр); иначе тема — слова реплики без
служебных («ты диджей X», «замути сэт», длина сета). Нет реплики или в ней нет значимых слов («да», «давай») —
тема LLM как есть.

:func:`heard_theme` — ЕДИНСТВЕННОЕ место, где тема выделяется из свободных слов реплики: путь LLM и путь команды
роутера (``rob_box_voice.core.media_router`` шлёт ``dj_set`` без темы, «у нас сегодня …» грамматика не режет) дают
одну тему (07.10: команда «… и у нас сегодня …» везла тему «… классической музыки замути сэт»). Слова просьбы —
таблица грамматики ``media_command_grammar.SET_REQUEST_WORDS``, имя диджея — её ``dj_persona_span``, стиль с цифрой
(«8битный», #3476) — ``search.blank_style``.

Слоты темы (#3581, ADR-0157 §2.1; живой сет 12.10 «ты диджей анакен скайвокер и у нас сегодня веселье на звезде
смерте» играл Celebration/Happy): :func:`set_theme` — алиас вселенной (``knowledge.UNIVERSES``) в теме или имени
персоны делает темой вселенную, слова настроения (``knowledge.THEME_MOOD_STEMS``) рядом со словами темы — не тема.
:func:`continued` — «продолжай сэт» без новой темы: тема последнего сета, а не слово «продолжай».
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

from rob_box_dialog.words import level_phrases, level_word
from rob_box_music import knowledge as kn
from rob_box_music.theme import match_row, match_style
from rob_box_music.universes import is_mood, universe_of, universe_spans, words_of

from .brief import split_brief
from .search import _PART_RE, blank_style, content_stems, genre_of

_AND = frozenset({"и", "and"})
_NOT = "не"


def _theme_word(words: List[str], i: int, request: frozenset) -> bool:
    """Слово ``i`` реплики называет тему: не просьба, не стиль, значимое; «музыки» — при слове жанра."""
    word = words[i].replace("ё", "е")
    if genre_of(word):
        return True
    if word in kn.GENRE_FILLER:
        return i > 0 and bool(genre_of(words[i - 1]))  # «классической музыки» — жанр словами человека
    if word in request or match_style([word]) is not None or level_word(word):  # громкость — уровень заказа, не тема
        return False
    return word.isdigit() or bool(content_stems(word))  # цифра темы — слово («mambo nr 5»), длину уже вырезали


def _kept(words: List[str], request: frozenset) -> List[bool]:
    """Какие слова реплики остаются в теме: слова темы (:func:`_theme_word`); «и» - только между словами темы, «не» -
    только перед словом темы (слово громкости уходит с «не», слово темы остаётся с ним). Фраза громкости целиком -
    уровень заказа, не тема: «не слишком громко» уходит со степенью (``words.level_phrases``, #4159 п.2), иначе в теме
    остались бы «не слишком»."""
    level = {i for first, last, _kind in level_phrases(words) for i in range(first, last + 1)}
    keep = [w not in _AND and i not in level and _theme_word(words, i, request) for i, w in enumerate(words)]
    for i, word in enumerate(words):
        if word in _AND:
            keep[i] = any(keep[:i]) and any(keep[i + 1:])
        elif word == _NOT:
            keep[i] = i + 1 < len(words) and keep[i + 1]
    return keep


def heard_theme(text: str) -> str:
    """Тема из реплики человека: слова без просьбы, «ты диджей X», стиля, громкости и длины сета; «и» между словами
    темы и запятые остаются (границы частей темы-перечисления, ``search.theme_parts``). «Не» связано со словом после
    него: слово громкости уходит из темы вместе с «не» («только не громко» - громкость, не тема «...громко»), слово темы
    остаётся с ним («не космос» не станет «космос»). Значимых слов нет - ``""``. Подробный промт (роль, BPM, ноты, таблица
    MIDI; #3845) - тема только из его просьбы (``brief.split_brief``): спецификация темой не становится."""
    from rob_box_voice.core.media_command_grammar import SET_REQUEST_WORDS, dj_persona_span
    from rob_box_voice.core.set_length_words import split_set_length, tokens  # длину сета разбирает одна грамматика

    _tracks, rest = split_set_length(split_brief(text).head)  # подробный промт (#3845): тема - из просьбы, не из спецификации
    persona = dj_persona_span(rest)
    if persona:
        rest = rest[:persona[0]] + " " * (persona[1] - persona[0]) + rest[persona[1]:]
    rest = blank_style(rest)
    toks = tokens(rest)
    keep = _kept([t[0] for t in toks], SET_REQUEST_WORDS)
    out = ""
    last_end = 0
    for (word, start, end), kept in zip(toks, keep):
        if not kept:
            continue
        if out:
            out += (", " if any(c in ",;:" for c in rest[last_end:start]) else " ")
        out += word
        last_end = end
    return out if content_stems(out) else ""


def order_theme(text: Optional[str], heard_text: Optional[str]) -> str:
    """Тема заказа генерации (#3775): слова темы просьбы «сочини … про / на тему X» реплики хода (грамматика
    ``GENERATE``, затем :func:`heard_theme` — без «ты», «сэт», предлогов); в реплике её нет, а ``text`` — не реплика
    (тема ``generate_music(theme)``) — его значимые слова. Ничего — ``""``: заказ без темы."""
    from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command

    command = parse_media_command(heard_text or "")
    said = command.name if command.intent is MediaIntent.GENERATE else ""
    if not said and text and text != heard_text:
        said = text
    return heard_theme(said) or (said if content_stems(said) else "")


def _names_new_theme(heard_text: Optional[str]) -> bool:
    """В реплике есть слова новой темы. «Тема та же» (``asks_same_theme``) — явная ссылка на прежнюю: остальные слова
    («только техно») — стиль. Глагол смены стиля («переходим на рок», ``asks_to_keep_theme``) новой темы не называет,
    пока в реплике нет значимых слов кроме жанра (#3593)."""
    from rob_box_voice.core.media_command_grammar import SET_SAME_WORDS, asks_same_theme, asks_to_keep_theme

    heard = heard_theme(heard_text or "")
    if asks_same_theme(heard_text):
        return False
    if not asks_to_keep_theme(heard_text):
        return bool(heard)
    return any(content_stems(w) and not genre_of(w) and w not in SET_SAME_WORDS for w, _s, _e in words_of(heard))


def _known_word(word: str) -> bool:
    """Слово темы с опорой в знании: латиница/цифра (название, STT пишет кириллицей), жанр, вселенная, настроение,
    стиль-алиас, строка таблицы тем (``knowledge.THEMES``)."""
    return (word.isascii() or bool(genre_of(word) or universe_of(word) or is_mood(word))
            or match_style([word]) is not None or match_row(word) is not None)


def names_theme(heard: str, heard_text: Optional[str]) -> bool:
    """Слова реплики НАЗЫВАЮТ тему (марафон-6 n605: «поставь сайт из трёх треков» - «сайт» это огрызок STT от «сет», и
    guard подменил им тему LLM «рок»; сет заказал платную генерацию «по теме сайт»). Называют, если значимых слов
    два и больше (описание, а не огрызок), или слово одно, но опорное (:func:`_known_word`), или человек ввёл его сам
    («про X», «на тему X»: ``THEME_LEADS``), или в реплике названо слово сета («сет X»: ``SET_WORDS``; без него «сайт»
    - слово сета, распознанное зря). Голое незнакомое слово на месте слова сета - не тема: у него нет опоры, кроме
    звучания. Одно знание - те же таблицы, что у поиска и слотов темы."""
    words = _content_words(heard)
    if len(words) != 1:
        return len(words) > 1
    return theme_support(heard, heard_text)


def _content_words(heard: str) -> List[str]:
    return [w for w, _s, _e in words_of(heard) if w not in _AND and w != _NOT and content_stems(w)]


def theme_support(heard: str, heard_text: Optional[str]) -> bool:
    """У слов темы ``heard`` есть опора, кроме звучания: хоть одно слово опорное (:func:`_known_word`), человек ввёл
    тему сам («про X», «на тему X»: ``THEME_LEADS`` перед первым словом темы) или назвал слово сета (``SET_WORDS``).
    Без опоры — огрызок распознавания, даже из нескольких слов («уводи сторону тётя гриша подруг добреет», n605):
    чойсер темы (``engine.theme_choice``) штрафует такой вариант (``THEME_CHOICE_WEIGHTS['bare']``)."""
    from rob_box_voice.core.media_command_grammar import SET_WORDS, THEME_LEADS

    words = _content_words(heard)
    if not words:
        return False
    said = [w for w, _s, _e in words_of(heard_text or "")]
    led = words[0] in said and any(w in THEME_LEADS for w in said[:said.index(words[0])])
    return any(_known_word(w) for w in words) or led or bool(SET_WORDS.intersection(said))


def grounded_theme(theme: Optional[str], heard_text: Optional[str]) -> Tuple[str, bool]:
    """``(тема, заменена ли)``: тема LLM, если она делит значимую основу с репликой хода, иначе тема реплики.
    Смена стиля без новой темы («переходим на рок, тема та же») тему LLM не заменяет (#3593), как и огрызок
    распознавания вместо слова темы (:func:`names_theme`, n605 М6)."""
    theme = theme or ""
    heard = heard_theme(heard_text or "")
    if not heard or content_stems(theme) & content_stems(heard):
        return theme, False
    if theme and (not _names_new_theme(heard_text) or not names_theme(heard, heard_text)):
        return theme, False
    return heard, True


def guessed_theme(theme: str, llm_theme: Optional[str], heard_text: Optional[str]) -> bool:
    """Тема сета - догадка кода: не тема LLM и не названа словами человека (:func:`names_theme`: голое слово реплики
    без опоры). Платной генерации по такой теме нет (``DjSetTool._start(free=True)``: кэш или синтез) - деньги не
    уходят на огрызок распознавания (n605 М6: две задачи Suno по 12 кредитов по теме «сайт»)."""
    heard = heard_theme(heard_text or "")
    if not theme or theme == (llm_theme or "") or not heard or not content_stems(theme) & content_stems(heard):
        return False
    return not names_theme(heard, heard_text)


@dataclass(frozen=True)
class SetTheme:
    """Слоты темы сета (ADR-0157 §2.1): ``theme`` — что искать, ``mood`` — слова настроения, выпавшие из темы,
    ``universe`` — ключ ``knowledge.UNIVERSES``, ставший темой (из темы или из имени персоны)."""

    theme: str
    mood: str = ""
    universe: str = ""


def _without_mood(text: str) -> Tuple[str, List[str]]:
    """Текст без слов настроения (``universes.is_mood``; на их месте пробелы) и сами эти слова."""
    out = text
    mood = []
    for word, start, end in words_of(text):
        if is_mood(word):
            out = out[:start] + " " * (end - start) + out[end:]
            mood.append(word)
    return out, mood


def _universe_parts(text: str) -> Tuple[str, List[str], List[str]]:
    """Части темы (``search._PART_RE``): часть с алиасом вселенной — фраза вселенной (повтор — один раз), иначе часть
    без слов настроения. ``(тема, ключи вселенных, слова настроения)``; вселенных нет — ключей нет."""
    keys: List[str] = []
    mood: List[str] = []
    out: List[str] = []
    for part in (p for p in _PART_RE.split(text) if p and p.strip()):
        rest, dropped = _without_mood(part)
        mood += dropped
        found = [key for _i, _j, key in universe_spans([w for w, _s, _e in words_of(part)])]
        new = [key for key in dict.fromkeys(found) if key not in keys]
        keys += new
        if found:
            out += [kn.UNIVERSES[key].theme for key in new]
        elif content_stems(rest):
            out.append(" ".join(rest.split()))
    return ", ".join(out), keys, mood


def set_theme(theme: Optional[str], persona: Optional[str] = None) -> SetTheme:
    """Тема сета после выделения слотов (#3581): алиас вселенной в теме — тема-вселенная («веселье звезде смерте» →
    «звёздные войны»); слова настроения рядом со словами темы — в ``mood``; в теме нет ничего, кроме настроения, а
    имя персоны из вселенной («диджей Анакин») — тема её вселенной. Ничего из этого — тема как была, слово в слово."""
    theme = theme or ""
    text, keys, mood = _universe_parts(theme)
    if keys:
        return SetTheme(text, " ".join(mood), keys[0])
    rest, mood = _without_mood(theme)
    persona_key = universe_of(persona)
    if not content_stems(rest):  # тема пуста или одно настроение: тема — вселенная персоны, иначе как была
        if persona_key:
            return SetTheme(kn.UNIVERSES[persona_key].theme, " ".join(mood), persona_key)
        return SetTheme(theme)
    return SetTheme(" ".join(rest.split()), " ".join(mood)) if mood else SetTheme(theme)


def continued(heard_text: Optional[str],
              last: Optional[Tuple[str, Optional[str]]]) -> Optional[Tuple[str, Optional[str]]]:
    """``(тема, персона)`` последнего сета, если реплика просит продолжить сет («продолжай сэт», «ещё сет») или сменить
    стиль при той же теме («переходим на рок, тема та же», #3593) и не называет новой темы; иначе ``None``. Слова —
    ``media_command_grammar``: ``SET_CONTINUE_WORDS`` (#3581), ``SET_SWITCH_WORDS``, ``SET_SAME_WORDS``."""
    from rob_box_voice.core.media_command_grammar import asks_to_continue_set

    if last is None or not asks_to_continue_set(heard_text) or _names_new_theme(heard_text):
        return None
    return last


def _grounded_logged(theme: Optional[str], heard_text: Optional[str], log: Any) -> str:
    """Тема сета, опирающаяся на реплику хода (:func:`grounded_theme`); подмену код пишет в лог честной строкой."""
    grounded, replaced = grounded_theme(theme, heard_text)
    if not replaced:
        return theme or ""
    if not theme:  # команда роутера или LLM без темы: тему выделил код из слов реплики, подмены не было
        log.info(f"🎛️ [dj_set] тема из слов реплики: {grounded!r}")
        return grounded
    log.warning(f"🎛️ [dj_set] тема LLM не из текущей реплики → из реплики: LLM={theme!r} → {grounded!r}")
    return grounded


def set_slots(theme: Optional[str], persona: Optional[str], heard_text: Optional[str],
              last: Optional[Tuple[str, Optional[str]]], log: Any) -> Tuple[str, Optional[str]]:
    """``(тема, персона)`` нового сета (``dj_set(start)``): «продолжай сэт» без новой темы — тема и персона последнего
    сета ``last`` (:func:`continued`); иначе тема по реплике хода (:func:`grounded_theme`). Затем слоты темы
    (:func:`set_theme`): вселенная из темы или персоны, настроение — не тема. Каждую замену код пишет в ``log``."""
    resumed = continued(heard_text, last)
    if resumed is not None:
        log.info(f"🎛️ [dj_set] продолжить сет без новой темы → тема последнего сета: {resumed[0]!r} (LLM={theme!r})")
        theme, persona = resumed[0], persona or resumed[1]
    else:
        theme = _grounded_logged(theme, heard_text, log)
    slots = set_theme(theme, persona)
    if slots.theme != theme:
        log.info(f"🎛️ [dj_set] слоты темы: {theme!r} → тема={slots.theme!r} вселенная={slots.universe or '—'} "
                 f"настроение={slots.mood or '—'} персона={persona!r}")
    return slots.theme, persona


__all__ = ["SetTheme", "continued", "grounded_theme", "guessed_theme", "heard_theme", "names_theme", "order_theme",
           "set_slots", "set_theme", "theme_support"]
