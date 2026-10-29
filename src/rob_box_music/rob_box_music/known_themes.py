"""Известные темы сета — одно место (решение Шифу 29.10, #3845 #4217): из них и слов реплики чойсер выбирает тему.

Шифу (Telegram 29.10 05:30 UTC): «мусорной темы не должно быть, можно из запроса пользователя сгенрить промт для
чойсера в чойсере пусть будут все слова из запроса плюс все известные темы что есть, и он по идее не ошибётся».

Известная тема — то, у чего есть опора в коде, а не только звучание слова:

* знание — :data:`knowledge.KNOWN_THEMES` (строки ``THEMES``/``GEN_THEME_TAGS``) и вселенные ``UNIVERSES``;
* кэш библиотеки — темы, под которые треки уже заказаны: метка темы в ``prompt`` трека после тегов стиля
  (``GenSet._job``: ``"<теги стиля>, <теги темы>"``); теги строки таблицы — её тема знания.

Чистый пакет (без сети): кэш приходит готовым списком ``CacheTrack``. Называет ли текст известную тему — те же
правила, что у кэша сета (``source.theme_fit``, ``theme_tags``), строки таблицы тем (``theme.match_row``) и вселенных
(``universes.universe_of``): выбранная известная тема берёт из кэша ровно те треки, что считает «по теме» сет.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
from typing import Dict, Sequence, Tuple

from . import knowledge as kn
from .source import CacheTrack, theme_fit, theme_stems, theme_tags
from .theme import match_row
from .universes import universe_of

KNOWLEDGE = "knowledge"
UNIVERSE = "universe"
CACHE = "cache"


@dataclass(frozen=True)
class KnownTheme:
    """Известная тема: ``name`` — тема сета словами (её получит сет), ``tags`` — её метка в кэше и заказе
    (``source.theme_tags``), ``source`` — откуда известна, ``tracks`` — сколько инструменталов кэша с этой меткой."""

    name: str
    tags: str
    source: str
    tracks: int = 0


def cache_theme(track: CacheTrack) -> str:
    """Метка темы трека кэша: ``prompt`` инструментала после тегов его стиля (``"<теги стиля>, <теги темы>"``);
    заказ без темы, песня, кавер, чужой формат ``prompt`` — «»."""
    style = kn.GEN_STYLES.get(track.style_key)
    if track.kind != "instrumental" or style is None or not track.prompt.startswith(style.tags):
        return ""
    return track.prompt[len(style.tags):].strip(" ,")


def known_themes(cache: Sequence[CacheTrack] = ()) -> Tuple[KnownTheme, ...]:
    """Известные темы: вселенные (точнее строк таблицы), темы знания, затем темы кэша (больше треков — раньше, не
    больше ``KNOWN_THEMES_CACHE_MAX``). Метка кэша, равная тегам темы знания, — эта тема знания (счётчик треков)."""
    out: Dict[str, KnownTheme] = {}
    for universe in kn.UNIVERSES.values():
        out.setdefault(universe.theme, KnownTheme(universe.theme, theme_tags(universe.theme), UNIVERSE))
    for name in kn.KNOWN_THEMES:
        out.setdefault(name, KnownTheme(name, theme_tags(name), KNOWLEDGE))
    counts = Counter(tag for tag in map(cache_theme, cache) if tag)
    for tags, count in counts.most_common(kn.KNOWN_THEMES_CACHE_MAX):
        same = [name for name, theme in out.items() if theme.tags == tags]
        for name in same:
            out[name] = replace(out[name], tracks=out[name].tracks + count)
        if not same:
            out[tags] = KnownTheme(tags, tags, CACHE, count)
    return tuple(out.values())


def names_known(theme: KnownTheme, said: str) -> bool:
    """Текст ``said`` называет известную тему: вселенная — её алиас; тема знания — та же строка таблицы тем или те же
    теги заказа; тема кэша — треки с её меткой кэш сета счёл бы «по теме» ``said`` (``theme_fit ≥ THEME_FIT_MIN``)."""
    if not theme_stems(said) and universe_of(said) is None:
        return False  # «рок», «да»: слов темы нет — темы не называют (короткое слово не ищется подстрокой)
    if theme.source == UNIVERSE:
        return universe_of(said) is not None and universe_of(said) == universe_of(theme.name)
    if theme.source == KNOWLEDGE:
        row = match_row(theme.name)
        return theme_tags(said) == theme.tags or (row is not None and match_row(said) == row)
    return theme_fit(theme_tags(said), CacheTrack("", theme.name, prompt=theme.tags)) >= kn.THEME_FIT_MIN


def covers_all(theme: KnownTheme, said: str) -> bool:
    """Тема покрывает все слова ``said``: для кэша и заказа это одна тема (те же теги) или каждая основа слов ``said``
    есть в названии и метке темы."""
    stems = theme_stems(said)
    return bool(stems) and (theme_tags(said) == theme.tags or stems <= theme_stems(f"{theme.name} {theme.tags}"))


__all__ = ["CACHE", "KNOWLEDGE", "KnownTheme", "UNIVERSE", "cache_theme", "covers_all", "known_themes", "names_known"]
