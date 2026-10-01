"""club_fragments.py — фрагменты RTTTL-мелодий как хук club-трека (issue #3225, umbrella #3223).

Зачем
=====
Без ``name=``/``rtttl=`` lead club — пентатонное блуждание
(:func:`core.club_arranger.riff_indices`): живой сет 30.09 звучал «одним и
тем же». Теперь такой вызов берёт РЕАЛЬНЫЙ фрагмент мелодии из
:class:`core.rtttl_library.RtttlLibrary` (~10 тыс. записей) и отдаёт его в
существующий путь хука (:func:`core.club_hook.extract_hook` →
:meth:`ClubHook.arrange`: транспонирование в тональность сета, аккорды,
регистр). Пентатоника остаётся только фолбеком (архив недоступен/пуст) —
с WARNING, не молча.

Что считается «музыкальным» фрагментом
=====================================
Окно — :data:`FRAGMENT_BARS` такта (блок аккордов club, ``CHORD_BARS``) на
границе такта: смещение кратно ``STEPS_PER_BAR`` шагов 16-х от первой
звучащей ноты мелодии (не обязательно 0). Окно годится, если (:func:`is_musical`):

* нот не меньше :data:`MIN_NOTES`;
* разных высот не меньше :data:`MIN_DISTINCT_PITCHES`;
* диапазон не уже :data:`MIN_RANGE` полутонов (не «одна нота» и не трель);
* звучащих шагов не меньше :data:`MIN_FILL` окна (не одни паузы).

Мелодия-кандидат — из ``качество >= MIN_QUALITY`` (:func:`melody_quality`);
список ``rowid`` таких записей строится один раз на библиотеку и
кэшируется (в памяти только числа, не ноты).

Отпечаток хука (:func:`hook_fingerprint`)
=========================================
Строка интервалов между соседними нотами в полутонах (со знаком, не более
:data:`FP_MAX_NOTES` нот): ``"+2,+2,-4,0,+7"``. Не зависит от тональности,
поэтому транспонированные копии (и одинаковые мотивы из разных мелодий)
дают один отпечаток; ритм не входит.

Выбор с учётом истории
======================
1. Мелодия — :func:`core.music_diversity.weighted_pick` по недавним
   ``melody_name``; 2. смещение — по недавним ``fragment_offset`` этой же
   мелодии, а окна с недавним отпечатком отбрасываются, если остаются другие.
   Детерминировано при заданных ``seed`` и истории (rng сидируется от
   ``seed`` и id последней записи — каждый вызов видит новую выборку).
"""

from __future__ import annotations

import logging
import random
import weakref
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from .club_arranger import CHORD_BARS, STEPS_PER_BAR
from .club_hook import ClubHook, HookNote, extract_hook
from .music_diversity import weighted_pick
from .rtttl_catalog import iter_melodies, melody_by_rowid
from .rtttl_library import human_track_title, melody_quality
from .web_melody import cached_web_melodies, fetch_web_melody, purge_stale_web_melodies

__all__ = [
    "FRAGMENT_BARS",
    "Fragment",
    "FragmentUnavailable",
    "fragment_pool",
    "club_hook_sentence",
    "fragment_windows",
    "hook_fingerprint",
    "is_musical",
    "pick_club_hook",
    "pick_fragment",
]

_LOG = logging.getLogger(__name__)

FRAGMENT_BARS = CHORD_BARS
MIN_QUALITY = 8.0
MIN_NOTES = 6
MIN_DISTINCT_PITCHES = 3
MIN_RANGE = 3
MIN_FILL = 0.35
FP_MAX_NOTES = 16
#: Сколько мелодий из пула пробуем за раунд и сколько раундов.
SAMPLE_MELODIES = 6
MAX_ROUNDS = 5
#: Сколько тактов от начала мелодии просматриваем в поисках окон.
MAX_OFFSET_BARS = 24
#: Сколько последних записей истории учитывает отпечаток.
RECENT_FINGERPRINTS = 10

_POOLS: "weakref.WeakKeyDictionary[Any, List[int]]" = weakref.WeakKeyDictionary()


class FragmentUnavailable(RuntimeError):
    """Фрагмент взять неоткуда (архив пуст/недоступен) — вызывающий уходит в фолбек."""


@dataclass(frozen=True)
class Fragment:
    """Выбранный фрагмент: хук для :meth:`ClubHook.arrange` и метаданные для истории."""

    hook: ClubHook
    melody: str
    title: str
    offset: int
    fingerprint: str
    source: str  # "theme" — найден по теме сета, "pool" — любой качественный

    @property
    def label(self) -> str:
        return f"{self.melody}@{self.offset}"


# ---------------------------------------------------------------------------
# Чистые функции
# ---------------------------------------------------------------------------


def hook_fingerprint(notes: Sequence[HookNote]) -> str:
    """Контур интервалов хука (см. модульный докстринг); пустой хук — ``""``."""
    pitches = [m for _s, _l, m in notes][:FP_MAX_NOTES]
    return ",".join(f"{b - a:+d}" for a, b in zip(pitches, pitches[1:]))


def is_musical(notes: Sequence[HookNote], bars: int = FRAGMENT_BARS) -> bool:
    """Годится ли окно как хук (критерии — в модульном докстринге)."""
    if len(notes) < MIN_NOTES:
        return False
    pitches = [m for _s, _l, m in notes]
    if len(set(pitches)) < MIN_DISTINCT_PITCHES or max(pitches) - min(pitches) < MIN_RANGE:
        return False
    sounding = sum(ln for _s, ln, _m in notes)
    return sounding >= MIN_FILL * bars * STEPS_PER_BAR


def fragment_windows(rtttl: str, bpm: float, melody_id: str = "", title: str = "") -> List[ClubHook]:
    """Все музыкальные окна мелодии на границах тактов (смещение 0, 16, 32, …)."""
    windows: List[ClubHook] = []
    for bar in range(MAX_OFFSET_BARS):
        hook = extract_hook(
            rtttl, bpm, melody_id=melody_id, title=title, offset=bar * STEPS_PER_BAR, bars=FRAGMENT_BARS,
        )
        if not hook.notes:
            break
        if is_musical(hook.notes):
            windows.append(hook)
    return windows


def fragment_pool(library: Any) -> List[int]:
    """``rowid`` мелодий качества >= :data:`MIN_QUALITY`; кэш на библиотеку.

    Raises:
        FragmentUnavailable: библиотека не отдала ни одной подходящей записи.
    """
    cached = _POOLS.get(library)
    if cached is None:
        cached = [
            rec["rowid"] for rec in iter_melodies(library)
            if rec.get("rtttl") and melody_quality(rec["rtttl"]) >= MIN_QUALITY
        ]
        if cached:
            try:
                _POOLS[library] = cached
            except TypeError:  # библиотека без weakref — без кэша, но работает
                pass
    if not cached:
        raise FragmentUnavailable("в RTTTL-библиотеке нет мелодий подходящего качества")
    return cached


# ---------------------------------------------------------------------------
# Выбор
# ---------------------------------------------------------------------------


def _usable(rec: Mapping[str, Any], bpm: float) -> Optional[Tuple[str, Mapping[str, Any], List[ClubHook]]]:
    """``(name, запись, окна)`` или ``None`` — не разбирается / нет музыкальных окон."""
    name = str(rec.get("name") or "")
    try:
        windows = fragment_windows(rec["rtttl"], bpm, melody_id=name, title=str(rec.get("title") or name))
    except (ValueError, KeyError, TypeError):
        return None
    return (name, rec, windows) if windows else None


def _human_title(library: Any, rec: Mapping[str, Any]) -> str:
    """Имя для голоса (как у хука по ``name``); сбой — архивный ``title``."""
    try:
        return human_track_title(library, dict(rec)) or str(rec.get("title") or rec.get("name") or "")
    except Exception:  # noqa: BLE001 — красивое имя не обязательно
        return str(rec.get("title") or rec.get("name") or "")


def _theme_candidates(library: Any, theme: str, bpm: float) -> List[Tuple[str, Mapping[str, Any], List[ClubHook]]]:
    try:
        # Issue #3228: мелодия темы, ранее найденная в вебе, важнее нечёткого поиска (он латинский).
        found = cached_web_melodies(library, theme) or library.search(theme, limit=20, include_rtttl=True)
    except Exception as exc:  # noqa: BLE001 — тема не должна ронять музыку
        _LOG.warning("club-фрагмент: поиск по теме %r не удался (%s) — берём любой качественный", theme, exc)
        return []
    return [u for u in (_usable(rec, bpm) for rec in found) if u]


def _pool_candidates(library: Any, bpm: float, rng: random.Random) -> List[Tuple[str, Mapping[str, Any], List[ClubHook]]]:
    pool = fragment_pool(library)
    for _round in range(MAX_ROUNDS):
        sample = rng.sample(pool, min(SAMPLE_MELODIES, len(pool)))
        found = []
        for rowid in sample:
            rec = melody_by_rowid(library, rowid)
            usable = _usable(rec, bpm) if rec else None
            if usable:
                found.append(usable)
        if found:
            return found
    return []


def _history_rng(seed: int, recent: Sequence[Mapping[str, Any]]) -> random.Random:
    last_id = recent[0].get("id", 0) if recent else 0
    return random.Random(f"club-fragment:{seed}:{last_id}")


def pick_fragment(
    library: Any, bpm: float, seed: int = 0, recent: Sequence[Mapping[str, Any]] = (), theme: Optional[str] = None,
) -> Fragment:
    """Выбрать фрагмент (см. модульный докстринг). ``recent`` — свежие первыми.

    Raises:
        FragmentUnavailable: нет ни одной подходящей мелодии.
    """
    rng = _history_rng(seed, recent)
    source = "theme"
    candidates = _theme_candidates(library, theme, bpm) if theme and theme.strip() else []
    if not candidates:
        source = "pool"
        candidates = _pool_candidates(library, bpm, rng)
    if not candidates:
        raise FragmentUnavailable("не нашлось мелодии с музыкальным фрагментом")
    by_name = {name: (rec, windows) for name, rec, windows in candidates}
    melody = weighted_pick(list(by_name), [r.get("melody_name") for r in recent], rng)
    rec, windows = by_name[melody]
    hook = _pick_window(windows, melody, recent, rng)
    return Fragment(
        hook=hook, melody=melody, title=_human_title(library, rec), offset=hook.offset,
        fingerprint=hook_fingerprint(hook.notes), source=source,
    )


def _not_just_played(windows: List[ClubHook], melody: str, recent: Sequence[Mapping[str, Any]]) -> List[ClubHook]:
    """Окна без отпечатка/смещения предыдущего трека (если это не оставляет пустоту)."""
    last = recent[0] if recent else {}
    last_fp = last.get("hook_fingerprint")
    last_off = last.get("fragment_offset") if last.get("melody_name") == melody else None
    other_fp = [w for w in windows if not (last_fp and hook_fingerprint(w.notes) == last_fp)]
    return [w for w in other_fp if w.offset != last_off] or other_fp or windows


def _pick_window(
    windows: List[ClubHook], melody: str, recent: Sequence[Mapping[str, Any]], rng: random.Random,
) -> ClubHook:
    """Окно мелодии со штрафом за недавний отпечаток и смещение (issue #3245).

    1. Отпечаток/смещение ПРЕДЫДУЩЕГО трека не берём подряд, пока есть другое окно
       (раньше, когда у короткой темы все отпечатки уже были в истории, жёсткий
       фильтр сворачивался в «любое окно» и тот же фрагмент играл два трека подряд).
    2. Среди оставшихся — :func:`weighted_pick` по давности отпечатка (а не
       бинарное «видели/не видели»), при равенстве — по давности смещения.
    """
    allowed = _not_just_played(windows, melody, recent)
    fps = [r.get("hook_fingerprint") for r in recent[:RECENT_FINGERPRINTS]]
    by_fp: Dict[str, List[ClubHook]] = {}
    for w in allowed:
        by_fp.setdefault(hook_fingerprint(w.notes), []).append(w)
    group = by_fp[weighted_pick(list(by_fp), fps, rng)]
    by_offset: Dict[int, ClubHook] = {w.offset: w for w in group}
    offsets = [r.get("fragment_offset") for r in recent if r.get("melody_name") == melody]
    return by_offset[weighted_pick(list(by_offset), offsets, rng)]


# ---------------------------------------------------------------------------
# Вход для ComposeMusicTool
# ---------------------------------------------------------------------------


def _describe(library: Any, frag: Fragment) -> Dict[str, Any]:
    """``hook_info`` в том же формате, что у :meth:`ComposeMusicTool._hook_info`."""
    hook = frag.hook
    return {
        "id": frag.melody, "title": frag.title, "source": "fragment", "bars": hook.bars,
        "key": hook.key_name, "time_scale": hook.time_scale, "offset": frag.offset,
        "fingerprint": frag.fingerprint, "pick": frag.source,
        "label": (
            f"фрагмент {hook.bars} такта с {frag.offset // STEPS_PER_BAR + 1}-го такта мелодии, тема в {hook.key_name}"
            + (", мажорная тема звучит в параллельном мажоре тональности трека (от III ступени)"
               if hook.key_mode == "major" else ", перенесена на тонику трека")
        ),
    }


#: Темы, по которым веб-поиск уже пробовали (библиотека, тема): без повтора на каждый трек сета.
_WEB_TRIED: set = set()


def _ensure_theme_melody(
    library: Any, theme: str, bpm: float, web_search: Callable[[str], Sequence[Mapping[str, Any]]],
    warn: Callable[[str], None], info: Callable[[str], None],
) -> None:
    """Темы нет в архиве → один раз найти её RTTTL в вебе и закэшировать (issue #3228)."""
    key = (id(library), theme.strip().lower())
    if key not in _WEB_TRIED:
        purge_stale_web_melodies(library, theme, info)  # TTL и недоверенные записи до #3243
    if key in _WEB_TRIED or _theme_candidates(library, theme, bpm):
        return
    _WEB_TRIED.add(key)
    if fetch_web_melody(library, theme, web_search, warn, info):
        info(f"[#3228] тема {theme!r} не найдена в архиве — мелодия из веба добавлена, выбираем из неё")


def pick_club_hook(
    library: Any, bpm: float, seed: int, recent: Sequence[Mapping[str, Any]], theme: Optional[str] = None,
    warn: Optional[Callable[[str], None]] = None, info: Optional[Callable[[str], None]] = None,
    web_search: Optional[Callable[[str], Sequence[Mapping[str, Any]]]] = None,
) -> Tuple[Optional[ClubHook], Optional[Dict[str, Any]]]:
    """Хук-фрагмент для club-вызова без ``name``/``rtttl``.

    ``(None, None)`` — фолбек на пентатонику (библиотеки нет, архив пуст или
    любая ошибка): причина всегда уходит в ``warn`` (по умолчанию WARNING
    логгера модуля), не молча.
    """
    recent = recent or ()
    warn = warn or _LOG.warning
    info = info or _LOG.info
    if library is None:
        warn("club-хук: RTTTL-библиотеки нет — lead = пентатонное блуждание (pentatonic-fallback)")
        return None, None
    try:
        if theme and theme.strip() and web_search is not None:
            _ensure_theme_melody(library, theme, bpm, web_search, warn, info)
        frag = pick_fragment(library, bpm, seed, recent, theme)
        info(f"[#3225] club хук: fragment {frag.label} ({frag.source})")
        return frag.hook, _describe(library, frag)
    except Exception as exc:  # noqa: BLE001 — любой сбой = фолбек, но громкий
        warn(f"club-хук: фрагмент из RTTTL-библиотеки не взят ({type(exc).__name__}: {exc}) "
             "— lead = пентатонное блуждание (pentatonic-fallback)")
        return None, None


def club_hook_sentence(hook: Mapping[str, Any]) -> str:
    """Фраза ответа тула про играющий хук (для LLM/диджея); ``hook`` — ``hook_info``."""
    fragment = hook.get("source") == "fragment"
    return (
        f" Lead играет {'фрагмент мелодии' if fragment else 'хук темы'} «{hook['title']}» (id={hook['id']}): "
        f"{hook['label']}. Это клубная переработка мотива, а не вся песня"
        + (" — можно объявить, что это за мелодия." if fragment else ".")
    )
