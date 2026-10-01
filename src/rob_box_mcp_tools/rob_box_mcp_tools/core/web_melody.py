"""web_melody.py — RTTTL из веб-сниппетов как источник мелодии для темы (issue #3228, umbrella #3223).

Зачем
=====
Тема DJ-сета («Очень странные дела») бывает вне RTTTL-архива: тогда
:func:`core.club_fragments.pick_club_hook` брал любой качественный фрагмент,
не связанный с темой. Здесь тема ищется в вебе (``search_web``, DuckDuckGo):
запрос ``«<тема> rtttl»``, из сниппетов вырезаются RTTTL-строки, проходят
валидацию (:func:`core.rtttl.parse_rtttl` + число нот) и кэшируются в
архив (:func:`core.rtttl_catalog.add_melody`, ``source="web"``,
``tags=["web", <тема>]``) — дальше выбор идёт обычным путём (б): следующий
трек с той же темой находит мелодию в архиве без сети.

Безопасность
============
Сниппеты — НЕДОВЕРЕННЫЕ данные третьей стороны. Из них берутся только
подстроки по строгому шаблону RTTTL; остальной текст не читается, не
интерпретируется и в лог/промпт не попадает. Текст RTTTL проходит
:func:`parse_rtttl` — то, что не разобралось, отбрасывается. Заголовок
(имя мелодии) в архив не пишется: ``name``/``title`` строит код из темы.

Сниппет режется ``search_web`` до 280 символов (``...``): последний токен
обрезанного сниппета отбрасывается (мог быть оборван посреди ноты).

Модуль чистый: поиск приходит вызываемым ``search(query) -> [snippet]``.
"""

from __future__ import annotations

import re
from typing import Any, Callable, List, Mapping, Optional, Sequence, Tuple

from .rtttl import parse_rtttl
from .rtttl_catalog import add_melody, melodies_by_tag, purge_web_melodies

__all__ = [
    "MAX_WEB_RTTTL_CHARS", "MIN_WEB_NOTES", "VERIFIED_TAG", "WEB_CACHE_TTL_S",
    "as_search_callable", "attach_web_search",
    "cached_web_melodies", "extract_rtttl_candidates", "fetch_web_melody", "is_relevant", "pick_theme",
    "purge_stale_web_melodies", "search_results_to_snippets", "theme_tag", "web_query",
]

#: Меньше звучащих нот — не мелодия (заголовок, обрывок).
MIN_WEB_NOTES = 12
#: Длиннее — не рингтон, а мусор (защита от подстановки огромного текста).
MAX_WEB_RTTTL_CHARS = 2000
#: Сколько мелодий-кандидатов кэшируем за один поиск.
MAX_CACHED = 2
#: Веб-кэш не вечный (issue #3243): старше — удаляется и ищется заново (30 суток).
WEB_CACHE_TTL_S = 30 * 24 * 3600
#: Метка «релевантность проверена» (:func:`is_relevant`); записи без неё (до #3243) недоверенные.
VERIFIED_TAG = "web-verified"
#: Тема длиннее не уходит в поисковик (запрос идёт наружу).
MAX_THEME_CHARS = 80

_TOKEN = r"(?:\d{1,2})?[a-gpA-GP][#_]?(?:\d\.?|\.\d?)?"
_RTTTL_RE = re.compile(
    r"(?P<head>[^\s:,;|]{0,40}):\s*(?P<defaults>(?:[dobDOB]\s*=\s*\d{1,3}\s*,?\s*){1,3}):\s*"
    r"(?P<body>" + _TOKEN + r"(?:\s*,\s*" + _TOKEN + r")+)"
)

Snippet = Mapping[str, Any]


def web_query(theme: str) -> str:
    """Запрос в поисковик: тема + ``rtttl`` (пробелы/кавычки/скобки в теме нормализуются)."""
    clean = re.sub(r"[\s\"'<>]+", " ", str(theme or "")).strip()[:MAX_THEME_CHARS]
    return f"{clean} rtttl" if clean else ""


def extract_rtttl_candidates(text: str, truncated: bool = False) -> List[str]:
    """RTTTL-строки из текста сниппета (валидные), с нейтральным именем ``web``.

    ``truncated`` — сниппет обрезан (``...``): если RTTTL идёт до самого
    конца текста, его последний токен отбрасывается.
    """
    text = text or ""
    tail = len(text.rstrip(". \n\t"))
    out: List[str] = []
    for m in _RTTTL_RE.finditer(text):
        defaults = re.sub(r"\s+", "", m.group("defaults")).rstrip(",")
        tokens = [t.strip() for t in m.group("body").split(",")]
        if truncated and m.end() >= tail and len(tokens) > 1:
            tokens = tokens[:-1]
        rtttl = f"web:{defaults}:{','.join(tokens)}"
        if len(rtttl) <= MAX_WEB_RTTTL_CHARS and _valid(rtttl) and rtttl not in out:
            out.append(rtttl)
    return out


def _stems(text: str) -> List[str]:
    """Значимые слова темы (>= 4 букв), урезанные до основы (``космический`` ~ ``космос``)."""
    words = [w for w in re.findall(r"\w+", str(text or "").lower()) if len(w) >= 4]
    return [w[:4] if len(w) >= 5 else w for w in words]


def is_relevant(theme: str, *evidence: str) -> bool:
    """Подтверждают ли заголовок/сниппет/имя RTTTL тему (issue #3243).

    Выдача по ``«<тема> rtttl»`` полна страниц про сам формат (демо библиотек,
    генераторы): мелодия принимается, только если хотя бы половина значимых слов
    темы встретилась в тексте-доказательстве. Тема без значимых слов проверке
    не поддаётся — отказ (честнее, чем принять вслепую).
    """
    stems = _stems(theme)
    hay = " ".join(str(e or "") for e in evidence).lower()
    return bool(stems) and sum(1 for s in stems if s in hay) * 2 >= len(stems)


def _valid(rtttl: str) -> bool:
    try:
        _name, bpm, notes = parse_rtttl(rtttl)
    except (ValueError, KeyError, TypeError):
        return False
    sounding = [m for m, _d in notes if m is not None]
    return bpm > 0 and len(sounding) >= MIN_WEB_NOTES and len(set(sounding)) >= 3


def search_results_to_snippets(result: Any) -> List[Snippet]:
    """``MCPToolResult`` от ``search_web`` → список сниппетов (``[]`` при неудаче)."""
    data = getattr(result, "data", None)
    if not getattr(result, "success", False) or not isinstance(data, Mapping):
        return []
    rows = data.get("results")
    return [r for r in rows if isinstance(r, Mapping)] if isinstance(rows, list) else []


def theme_tag(theme: str) -> str:
    """Тег/имя кэшированной веб-мелодии: тема в нижнем регистре, без кавычек и лишних пробелов."""
    return re.sub(r"\s+", " ", re.sub(r"[\"'<>]+", " ", str(theme or ""))).strip().lower()[:MAX_THEME_CHARS]


def cached_web_melodies(library: Any, theme: str) -> List[Any]:
    """Веб-мелодии, ранее закэшированные по этой теме (``source="web"``, тег темы)."""
    tag = theme_tag(theme)
    if not tag:
        return []
    return [r for r in melodies_by_tag(library, tag) if r.get("source") == "web" and VERIFIED_TAG in r.get("tags", [])]


def purge_stale_web_melodies(library: Any, theme: str, info: Optional[Callable[[str], None]] = None) -> int:
    """Убрать из архива протухшие и непроверенные веб-мелодии темы (issue #3243); вернуть сколько."""
    removed = purge_web_melodies(library, theme_tag(theme), WEB_CACHE_TTL_S, VERIFIED_TAG)
    if removed and info:
        info(f"[#3243] веб-кэш темы {theme!r}: удалено {removed} протухших/непроверенных мелодий")
    return removed


def fetch_web_melody(
    library: Any, theme: str, search: Callable[[str], Sequence[Snippet]],
    warn: Optional[Callable[[str], None]] = None, info: Optional[Callable[[str], None]] = None,
) -> int:
    """Найти в вебе RTTTL темы и записать в архив; вернуть, сколько мелодий добавлено.

    Любой сбой (нет сети, пустая выдача, нет валидных нот) — ``0`` и причина в
    ``warn``: вызывающий уходит в обычный выбор фрагмента, не молча.
    """
    query = web_query(theme)
    if not query:
        return 0
    try:
        snippets = list(search(query))
    except Exception as exc:  # noqa: BLE001 — сеть не должна ронять музыку
        if warn:
            warn(f"[#3228] веб-поиск мелодии по теме {theme!r} не удался ({type(exc).__name__}: {exc})")
        return 0
    found: List[Tuple[str, str]] = []
    skipped = 0
    for snip in snippets:
        body = str(snip.get("body") or "")
        if not is_relevant(theme, snip.get("title"), body):
            skipped += 1
            continue
        for rtttl in extract_rtttl_candidates(body, truncated=body.rstrip().endswith("...")):
            found.append((rtttl, str(snip.get("url") or "")))
    if not found:
        if warn:
            warn(f"[#3228] в веб-выдаче по теме {theme!r} нет валидного RTTTL ({len(snippets)} сниппетов, "
                 f"{skipped} отсеяно как не про тему [#3243])")
        return 0
    name = theme_tag(theme)
    added = 0
    for rtttl, url in found[:MAX_CACHED]:
        if add_melody(library, rtttl, name=name, title=name, source="web", tags=["web", VERIFIED_TAG, name]):
            added += 1
            if info:
                info(f"[#3228] мелодия по теме {theme!r} из веба закэширована ({url[:80]})")
    return added


def as_search_callable(tool: Any) -> Callable[[str], List[Snippet]]:
    """Обернуть ``SearchWebTool`` в ``search(query) -> [snippet]``."""
    return lambda query: search_results_to_snippets(tool.execute(query=query, max_results=8))


def attach_web_search(compose_tool: Any, search_tool: Any) -> None:
    """Дать ``compose_music`` веб-поиск темы (``search_web``); нет тула — веб не используется."""
    if search_tool is not None:
        compose_tool.web_search = as_search_callable(search_tool)


def club_label_theme(
    kwargs: Mapping[str, Any], hook_title: Optional[str], default: Optional[str],
) -> Optional[str]:
    """Issue #3244: «на тему «…»» в имени трека — заказанное, а не случайный хук.

    Хук из пула — деталь реализации: в сете про пиратов трек звался «на тему
    «Pacman»». Хук даёт имя, только если его заказали явно (``name``/``rtttl``);
    иначе — тема вызова (``theme`` или тема тула по умолчанию), без неё — None.
    """
    if hook_title and (kwargs.get("name") or kwargs.get("rtttl")):
        return hook_title
    theme = pick_theme(kwargs, default)
    return (str(theme).strip() or None) if theme else None


def pick_theme(kwargs: Mapping[str, Any], default: Optional[str]) -> Optional[str]:
    """Тема club-вызова: параметр ``theme`` важнее темы по умолчанию тула."""
    return kwargs.get("theme") or default
