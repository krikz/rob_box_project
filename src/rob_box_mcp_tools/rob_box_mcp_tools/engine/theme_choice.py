"""Тема DJ-сета — выбором чойсера из известных тем и слов реплики (решение Шифу 29.10; #3845 #4217; ADR-0148).

Шифу (Telegram 29.10 05:30 UTC): «мусорной темы не должно быть, можно из запроса пользователя сгенрить промт для
чойсера в чойсере пусть будут все слова из запроса плюс все известные темы что есть, и он по идее не ошибётся».
Марафоны давали тему-мусор («сайт», «объявляй каждый», «уводи сторону тетя гриша подруг добреет»): guard
``theme_grounding.grounded_theme`` подменял тему LLM словами реплики, а сет по такой теме не находил ничего в кэше
(266+ треков) и заказывал платную генерацию.

Решает код: вопрос чойсера (``rob_box_music.choice.ChoiceQuestion``, закрытый перечень id) строит
:func:`theme_question`:

* ``none`` — без темы, сет по стилю (стиль решает ``set_style``/``slot_genres`` словами человека, как раньше);
* ``new:<i>`` — тема из слов реплики (``heard_theme`` без названия движка), если они называют тему
  (``names_theme``, #4226), и её части-перечисления; текст строит код, модель его не пишет;
* ``llm`` — тема LLM, только если она делит основу со словами реплики (перефраз), или в реплике слов темы нет
  совсем («давай» — тема из контекста диалога, как раньше);
* ``known:<тема>`` — все известные темы (``rob_box_music.known_themes``: вселенные, таблица тем, темы кэша).

Ответ модели — один id из перечня (ответ вне перечня отбрасывает ``DeterministicChooser``); текст выбранного варианта
проверяет :func:`valid_theme` (известная тема или только слова реплики/LLM с содержанием). Мусорная тема структурно
невозможна: свободного текста от модели нет. Однозначный вопрос (разрыв очков ≥ ``CHOOSER_CLEAR_GAP``) модель не видит,
кроме открытого (:func:`open_question`: у реплики свои слова темы, а известную тему в них код не узнал — перефраз и
перевод сверяет модель); кубик кода последней ступени тему не бросает — лучший по очкам кода.
Одна строка лога на выбор: ``[theme] реплика=… LLM=… → <id> «тема» (кто) варианты=N известных=K (кэш C) лучшие=[…]``.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from rob_box_music import knowledge as kn
from rob_box_music.choice import DICE, Choice, ChoiceQuestion, Chooser, DeterministicChooser, Option, score
from rob_box_music.known_themes import KnownTheme, covers_all, known_themes, names_known
from rob_box_music.source import blank_engine, set_genres
from rob_box_music.theme import match_style_text

from .search import _PART_RE, content_stems
from .theme_grounding import (continued, grounded_theme, guessed_theme, heard_theme, names_theme, set_slots, set_theme,
                              theme_support)

_LOG = logging.getLogger(__name__)
NONE_ID = "none"
LLM_ID = "llm"
NEW_PREFIX = "new:"
KNOWN_PREFIX = "known:"
#: Кто «выбрал» «без темы», когда выбор чойсера не прошёл :func:`valid_theme`.
CHECK = "check"
#: Потолок вариантов из слов реплики: вся тема и части перечня («космос и динозавры» → «космос», «динозавры»).
NEW_MAX = 4
#: Что сказать модели (решение Шифу 29.10): варианты и очки строит код, модель выбирает один id.
THEME_INSTRUCTIONS = (
    "Выбери тему диджей-сета по заказу человека. Варианты: «без темы» (сет по стилю), темы из слов самого человека "
    "и известные темы (вселенные, таблица тем, темы треков, сочинённых раньше, — под них есть музыка). Если человек "
    "говорит об известной теме (перефраз, перевод, персонаж, место), выбери её. Слова, которые тему не называют "
    "(огрызки распознавания, обращения, имена слушателей, просьбы), — не тема: тогда «без темы». У вариантов — очки "
    "кода; отклоняйся от лучшего, только если явно видишь, что другой лучше подходит к заказу. Выбери один id.")


@dataclass(frozen=True)
class ThemePick:
    """Выбор темы: ``theme`` — тема сета («» — без темы), ``id`` — вариант, ``provider`` — кто выбрал."""

    theme: str
    id: str
    provider: str


def request_words(heard_text: Optional[str]) -> str:
    """Слова темы реплики: без просьбы, стиля, длины и названия движка («на синтезаторе» — путь, не тема)."""
    return heard_theme(blank_engine(heard_text or ""))


def _stems(text: str) -> frozenset:
    return frozenset(content_stems(text or ""))


def _new_options(heard: str, heard_text: str) -> Dict[str, Dict[str, float]]:
    """Варианты из слов реплики: вся тема и части перечня; слова без опоры (``theme_support``) — ``bare``."""
    if not heard or not names_theme(heard, heard_text):
        return {}
    bare = 0.0 if theme_support(heard, heard_text) else 1.0
    out = {heard: {"asked": 1.0, "whole": 1.0, "bare": bare}}
    parts = [" ".join(p.split()) for p in _PART_RE.split(heard) if p and content_stems(p)]
    for part in parts[:NEW_MAX - 1] if len(parts) > 1 else ():
        out.setdefault(part, {"asked": 1.0, "bare": bare})
    return out


def _llm_option(texts: Dict[str, Dict[str, float]], heard: str, heard_text: str, llm_theme: str) -> Optional[str]:
    """Тема LLM вариантом: её слова темы делят основу со словами реплики (перефраз — слова человека) или в реплике слов
    темы нет совсем (тема из контекста диалога). Совпала со словами реплики — тот же вариант, подтверждённый LLM.
    LLM, повторившая огрызок реплики, опоры ему не даёт: слова без опоры — ``bare`` и у неё. Текст варианта — тема LLM
    как есть («Очень странные дела»: «очень» — не степень громкости, а название), если в ней есть слова темы."""
    words = heard_theme(llm_theme)
    if not words or (heard and not _stems(words) & _stems(heard)):
        return None
    llm = " ".join(llm_theme.split())
    if llm in texts:
        texts[llm]["llm"] = 1.0
        return None
    texts[llm] = {"asked": float(bool(heard)), "whole": float(bool(heard) and _stems(heard) <= _stems(llm)),
                  "llm": 1.0, "bare": float(bool(heard) and not theme_support(llm, heard_text))}
    return llm


def _none_option(heard: str, heard_text: str, llm_theme: str) -> Option:
    """«Без темы»: просили стиль (``asked``), слов темы нет ни в реплике, ни у LLM (``whole``), LLM назвала только стиль
    (``llm``: «рок» — стиль, а не тема: по теме «рок» кэш сета пуст)."""
    style = bool(match_style_text(heard_text) or set_genres(heard_text)[0])
    llm_words = heard_theme(llm_theme)
    features = {"asked": float(style), "whole": float(not heard and not llm_words),
                "llm": float(bool(llm_theme) and not llm_words)}
    return Option(NONE_ID, features, "без темы — сет по стилю")


def _known_option(theme: KnownTheme, heard: str, llm_theme: str) -> Option:
    asked = bool(heard) and names_known(theme, heard)
    features = {"asked": float(asked), "whole": float(asked and covers_all(theme, heard)),
                "llm": float(bool(llm_theme) and names_known(theme, llm_theme)), "cached": float(theme.tracks > 0)}
    note = f", в кэше {theme.tracks}" if theme.tracks else ""
    return Option(f"{KNOWN_PREFIX}{theme.name}", features, f"известная тема «{theme.name}» ({theme.source}{note})")


def theme_question(heard_text: Optional[str], llm_theme: Optional[str],
                   known: Sequence[KnownTheme]) -> Tuple[ChoiceQuestion, Dict[str, str]]:
    """Вопрос чойсера темы и текст темы по id варианта. Порядок вариантов решает ничью: «без темы», слова реплики,
    тема LLM, известные темы."""
    said, llm_theme = heard_text or "", llm_theme or ""
    heard = request_words(said)
    texts = _new_options(heard, said)
    llm = _llm_option(texts, heard, said, llm_theme)
    options = [_none_option(heard, said, llm_theme)]
    themes = {NONE_ID: ""}
    for i, (text, features) in enumerate(t for t in texts.items() if t[0] != llm):
        options.append(Option(f"{NEW_PREFIX}{i}", features, f"тема из слов человека «{text}»"))
        themes[options[-1].id] = text
    if llm is not None:
        options.append(Option(LLM_ID, texts[llm], f"тема LLM из слов человека «{llm}»"))
        themes[LLM_ID] = llm
    for theme in known:
        option = _known_option(theme, heard, llm_theme)
        if option.id not in themes:
            options.append(option)
            themes[option.id] = theme.name
    question = ChoiceQuestion(f"тема сета: «{said or llm_theme}»", tuple(options),
                              state={"heard": said, "llm": llm_theme}, weights=kn.THEME_CHOICE_WEIGHTS,
                              instructions=THEME_INSTRUCTIONS)
    return question, themes


def valid_theme(text: str, option_id: str, known: Sequence[KnownTheme], heard_text: Optional[str],
                llm_theme: Optional[str]) -> bool:
    """Валидатор выбора: «без темы» — пусто; известная тема — из списка; иначе — значимые слова только из реплики и
    темы LLM (код строил вариант из них; модель текста не пишет)."""
    if option_id == NONE_ID:
        return text == ""
    if option_id.startswith(KNOWN_PREFIX):
        return text in {t.name for t in known}
    stems = _stems(text)
    return bool(stems) and stems <= _stems(f"{heard_text or ''} {llm_theme or ''}")


def _best(question: ChoiceQuestion, top: int = 3) -> List[str]:
    ranked = sorted(question.options, key=lambda o: -score(o.features, question.weights))
    return [f"{o.id}:{score(o.features, question.weights):g}" for o in ranked[:top]]


def open_question(question: ChoiceQuestion) -> bool:
    """Код по очкам не видит ответа: у реплики свои слова темы, а ни одну известную тему код в них не узнал —
    перефраз или перевод известной темы («очень странные дела» — Stranger Things) сверяет только модель."""
    new = any(o.id.startswith(NEW_PREFIX) or o.id == LLM_ID for o in question.options)
    return new and not any(o.id.startswith(KNOWN_PREFIX) and o.features.get("asked") for o in question.options)


def _ask(chooser: Chooser, question: ChoiceQuestion) -> Choice:
    """Выбор чойсера; открытый вопрос (:func:`open_question`) — модели и при ясном разрыве. Кубик кода последней
    ступени цепочки (``choice.dice``) тему не бросает: модели не ответили — лучший по очкам кода."""
    if isinstance(chooser, DeterministicChooser):
        choice = chooser.choose(question, ask_model=open_question(question))
    else:
        choice = chooser.choose(question)
    return DeterministicChooser().choose(question) if choice.provider == DICE else choice


def choose_theme(llm_theme: Optional[str], heard_text: Optional[str], known: Sequence[KnownTheme], chooser: Chooser,
                 log: Any = _LOG) -> ThemePick:
    """Тема сета выбором чойсера (:func:`theme_question`); выбор не прошёл :func:`valid_theme` — «без темы»."""
    question, themes = theme_question(heard_text, llm_theme, known)
    choice = _ask(chooser, question)
    text = themes.get(choice.id, "")
    if not valid_theme(text, choice.id, known, heard_text, llm_theme):
        log.warning(f"🎯 [theme] выбор {choice.id} «{text}» не прошёл проверку — без темы")
        choice, text = Choice(NONE_ID, 0.0, CHECK), ""
    if heard_theme(llm_theme or "") and LLM_ID not in themes and text != " ".join((llm_theme or "").split()):
        log.warning(f"🎛️ [dj_set] тема LLM не из текущей реплики → из реплики: LLM={llm_theme!r} → {text!r}")
    cached = sum(t.tracks > 0 for t in known)
    log.info(f"🎯 [theme] реплика={heard_text!r} LLM={llm_theme!r} → {choice.id} «{text}» ({choice.provider}) "
             f"варианты={len(question.options)} известных={len(known)} (кэш {cached}) лучшие={_best(question)}")
    return ThemePick(text, choice.id, choice.provider)


def chosen_slots(theme: Optional[str], persona: Optional[str], heard_text: Optional[str],
                 last: Optional[Tuple[str, Optional[str]]], known: Sequence[KnownTheme], chooser: Chooser,
                 log: Any = _LOG) -> Tuple[str, Optional[str]]:
    """``(тема, персона)`` нового сета: «продолжай сэт» или вызов без реплики — как раньше (``set_slots``); иначе тема —
    выбор чойсера (:func:`choose_theme`), затем слоты темы (``set_theme``: вселенная, настроение — не тема)."""
    if heard_text is None or continued(heard_text, last) is not None:
        return set_slots(theme, persona, heard_text, last, log)
    picked = choose_theme(theme, heard_text, known, chooser, log).theme
    slots = set_theme(picked, persona)
    if slots.theme != picked:
        log.info(f"🎛️ [dj_set] слоты темы: {picked!r} → тема={slots.theme!r} вселенная={slots.universe or '—'} "
                 f"настроение={slots.mood or '—'} персона={persona!r}")
    return slots.theme, persona


def dj_set_slots(dj: Any, theme: Optional[str], persona: Optional[str],
                 heard_text: Optional[str]) -> Tuple[str, Optional[str], bool]:
    """``(тема, персона, без платной генерации)`` для ``DjSetTool``: известные темы — из кэша его сета из генерации
    (``GenSetMaker.cached``) и знания, чойсер — тот же, что у пути музыки (``music_chooser``). Слова реплики — догадка
    кода из голого слова (``guessed_theme``, n605 М6: «сайт» вместо «сет»): платный заказ не уходит, как в #4226."""
    log = dj.node.get_logger() if dj.node is not None else _LOG
    maker = dj.gen_sets
    known = known_themes(maker.cached() if maker is not None else ())
    chooser = getattr(maker, "chooser", None) or DeterministicChooser()
    slots, persona = chosen_slots(theme, persona, heard_text, dj._last_dj, known, chooser, log)  # noqa: SLF001
    free = guessed_theme(grounded_theme(theme, heard_text)[0], theme, heard_text)
    if free:
        log.warning(f"🎛️ [dj_set] слова реплики {request_words(heard_text)!r} - догадка кода, не названы: без платной "
                    f"генерации")
    return slots, persona, free


__all__ = ["CHECK", "KNOWN_PREFIX", "LLM_ID", "NEW_PREFIX", "NONE_ID", "THEME_INSTRUCTIONS", "ThemePick",
           "choose_theme", "chosen_slots", "dj_set_slots", "open_question", "request_words", "theme_question",
           "valid_theme"]
