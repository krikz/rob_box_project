"""dj_set_walk.py — движение тональности и темпа по DJ-сету (issue #3226).

Живой лог 30.09: тоника сета ходила F → C → F → A# → F …, темп всегда
124. Слушатель слышит цикл из двух-трёх нот и один и тот же BPM весь вечер.
Здесь — чистые детерминированные функции (без ROS/времени/ГСЧ процесса):

* :func:`related_root` — тоника трека: явный обход КРУГА КВИНТ (+7
  полутонов на трек; за 12 треков все 12 тоник, цикла из 3 нот нет). Соседи
  по кругу квинт делят 6 из 7 нот — смена мягкая, как у диджеев (Camelot +1);
* :func:`track_key` — (тоника, лад) трека: тоника из круга квинт, а лад —
  из «родственных»: минор, ДОРИЙСКИЙ/ФРИГИЙСКИЙ на той же тонике
  (одноимённые лады) и ПАРАЛЛЕЛЬНЫЙ мажор (тоника + 3 полутона, те же ноты).
  Треки #1 и #2 — всегда минор: #1 звучит в тонике сета (превью), #2 — её
  квинта;
* :func:`track_bpm` — темп дрейфует в пределах ±4 BPM от базового
  (120–128 при 124), шаг между соседними треками ≤ 4, без возврата к
  значению двухтрековой давности (нет «пилы» 124-126-124).

Всё выводится из ``seed_key`` (эпоха старта сета) и номера трека: один и тот
же сет воспроизводим, разные сеты ходят по-разному.

Issue #3249 — ХАРАКТЕР СЕТА. Ночной прогон 30.09: 37 club-треков в 8 темах,
все 120–128 BPM и 70 % минора — тема («детский праздник», «дождливый город»)
до темпа и лада не доходила. Словаря «тема → BPM» здесь нет и не будет:
характер темы выводит LLM по правилам скилла ``dj.txt`` и отдаёт в
``set_dj_mode(base_bpm=..., scale=...)``; :func:`apply_set_character` честно
принимает их, а дрейф и родственные лады (разнообразие ADR-0146/#3226)
работают уже вокруг этого характера:

* ``base_bpm`` — центр дрейфа (не фиксация, в отличие от ``bpm`` юзера);
* ``scale`` — ПРЕДПОЧТЁННЫЙ лад: треки #1–#2 в нём, дальше он берёт
  :data:`SET_SCALE_SHARE` треков, остальное — прежние родственные лады.

Без характера (старые вызовы, тема не задана) — побайтно как раньше.
"""

from __future__ import annotations

import random
import re
from typing import Any, Callable, List, Optional, Tuple

__all__ = [
    "BPM_DRIFT_MAX_STEP",
    "BPM_DRIFT_SPAN",
    "CLUB_ROOTS",
    "FIFTH",
    "PARALLEL_MAJOR_SHIFT",
    "SET_BASE_BPM_RANGE",
    "SET_SCALES",
    "SET_SCALE_SHARE",
    "apply_bpm_request",
    "apply_set_character",
    "bpm_is_request",
    "bpm_walk",
    "club_key",
    "club_theme_arg",
    "related_root",
    "state_bpm",
    "track_bpm",
    "track_key",
]

#: Тоники в написании ``compose_music`` (``arranger.VALID_ROOTS``).
CLUB_ROOTS = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")

#: Шаг обхода по кругу квинт: чистая квинта вверх (полутонов).
FIFTH = 7
#: Тоника параллельного мажора относительно минорной (те же 7 нот).
PARALLEL_MAJOR_SHIFT = 3

#: Максимальный скачок темпа между соседними треками (BPM).
BPM_DRIFT_MAX_STEP = 4
#: Насколько темп может уйти от базового вверх/вниз (BPM): 124 → 120..128.
BPM_DRIFT_SPAN = 4

#: Лады трека и их веса. Первые два трека сета — только minor.
_MODE_WEIGHTS: Tuple[Tuple[str, float], ...] = (
    ("minor", 0.55), ("dorian", 0.15), ("phrygian", 0.10), ("major", 0.20),
)
_PLAIN_TRACKS = 2
_BPM_STEPS = (-4, -2, 2, 4)

#: Issue #3249 — лады, которые сет может предпочесть (``SUPPORTED_SCALES`` club).
SET_SCALES = tuple(mode for mode, _ in _MODE_WEIGHTS)
#: Доля треков после #2 в предпочтённом ладу; остаток делят прочие лады
#: в прежней пропорции — сет в мажоре всё ещё иногда уходит в минор/дорийский.
SET_SCALE_SHARE = 0.7
#: Допустимый базовый темп характера сета: дрейф ±4 остаётся внутри
#: ``arranger.BPM_RANGE``, а club-грув не превращается в даб/драм-н-бейс.
SET_BASE_BPM_RANGE = (90, 150)


def related_root(set_root: str, track_no: int) -> str:
    """Минорная тоника трека ``track_no`` (с 1): круг квинт от тоники сета.

    Трек #1 — тоника сета, каждый следующий — на чистую квинту выше.
    Неизвестная тоника — как есть.
    """
    if set_root not in CLUB_ROOTS:
        return set_root
    shift = FIFTH * (max(1, track_no) - 1)
    return CLUB_ROOTS[(CLUB_ROOTS.index(set_root) + shift) % len(CLUB_ROOTS)]


def _mode_weights(prefer: str) -> Tuple[Tuple[str, float], ...]:
    """Веса ладов: прежние или с долей :data:`SET_SCALE_SHARE` у ``prefer``."""
    if prefer not in SET_SCALES:
        return _MODE_WEIGHTS
    rest = sum(w for mode, w in _MODE_WEIGHTS if mode != prefer)
    return tuple(
        (mode, SET_SCALE_SHARE if mode == prefer else (1.0 - SET_SCALE_SHARE) * w / rest)
        for mode, w in _MODE_WEIGHTS
    )


def _pick_mode(seed_key: int, track_no: int, prefer: str = "") -> str:
    if track_no <= _PLAIN_TRACKS:
        return prefer if prefer in SET_SCALES else "minor"
    weights = _mode_weights(prefer)
    point = random.Random(f"dj-mode:{seed_key}:{track_no}").random() * sum(w for _, w in weights)
    acc = 0.0
    for mode, weight in weights:
        acc += weight
        if point < acc:
            return mode
    return "minor"


def track_key(set_root: str, track_no: int, seed_key: int = 0, prefer: str = "") -> Tuple[str, str]:
    """``(root, scale)`` club-трека ``track_no`` (с 1) сета с тоникой ``set_root``.

    ``scale`` из ``club_arranger.SUPPORTED_SCALES``. Параллельный мажор
    получает тонику на 3 полутона выше минорной (A minor → C major): звукоряд
    тот же, смена тональности незаметна. Неизвестная тоника — ``(как есть, "minor")``.
    ``prefer`` — лад характера сета (#3249), ``""`` — прежние веса.
    """
    minor_root = related_root(set_root, track_no)
    if minor_root not in CLUB_ROOTS:
        return minor_root, "minor"
    mode = _pick_mode(seed_key, track_no, prefer)
    if mode != "major":
        return minor_root, mode
    return CLUB_ROOTS[(CLUB_ROOTS.index(minor_root) + PARALLEL_MAJOR_SHIFT) % len(CLUB_ROOTS)], "major"


def bpm_walk(base: int, count: int, seed_key: int = 0) -> List[int]:
    """Темпы первых ``count`` треков: #1 = ``base``, дальше дрейф ±2/±4 BPM.

    Остаётся в ``base ± BPM_DRIFT_SPAN``; значение двухтрековой давности
    не повторяется (пока есть другой ход).
    """
    lo, hi = base - BPM_DRIFT_SPAN, base + BPM_DRIFT_SPAN
    walk = [base]
    for n in range(2, count + 1):
        options = [walk[-1] + s for s in _BPM_STEPS if lo <= walk[-1] + s <= hi]
        fresh = [b for b in options if len(walk) < 2 or b != walk[-2]]
        pool = fresh or options
        walk.append(pool[random.Random(f"dj-bpm:{seed_key}:{n}").randrange(len(pool))])
    return walk[:count]


def track_bpm(base: int, track_no: int, seed_key: int = 0) -> int:
    """Темп трека ``track_no`` (с 1) — :func:`bpm_walk`."""
    return bpm_walk(base, max(1, track_no), seed_key)[-1]


# ---------------------------------------------------------------------------
# Связка с состоянием сета (``DJState``: set_bpm, bpm_locked, started_at,
# tracks_started) — утиная типизация, чтобы модуль не импортировал dj_mode.
# ---------------------------------------------------------------------------


def _seed_key(state: Any) -> int:
    return int(state.started_at) if state.started_at else 0


def state_bpm(state: Any, track_no: int) -> int:
    """Темп трека ``track_no``: дрейф от ``set_bpm`` или темп, названный юзером."""
    if state.bpm_locked:
        return int(state.set_bpm)
    return track_bpm(int(state.set_bpm), track_no, _seed_key(state))


def club_key(state: Any, set_root: str, track_no: int, *, hooked: bool = False) -> Tuple[str, str]:
    """``(root, scale)`` вызова club-трека.

    ``hooked`` — трек играет хук RTTTL-темы (``name=``): хук переносится в
    minor-тонику сета (``core/club_hook``), поэтому мажор/лады тут не
    применяются — минорная тоника круга квинт и ``minor``.
    """
    if hooked:
        return related_root(set_root, track_no), "minor"
    return track_key(set_root, track_no, _seed_key(state), str(getattr(state, "set_scale", "") or ""))


def club_theme_arg(state: Any, hooked: bool = False) -> str:
    """``theme="…", `` для вызова club-трека (issue #3228) или ``""``.

    Тема идёт в ``compose_music``, только когда трек без ``name=`` (``hooked`` —
    ``name=`` уже есть): тул берёт фрагмент мелодии на эту тему из архива, а
    если её там нет — из веба. Тема — речь юзера (STT): кавычки, слэши и
    переводы строк вычищаются, длина ограничена, чтобы вызов оставался
    одной строкой без чужих аргументов.
    """
    raw = str(getattr(state, "theme", "") or "")
    clean = " ".join(re.sub(r'[\\"<>{}()\[\]`]+', " ", raw).split())[:80]
    return "" if hooked or not clean else f'theme="{clean}", '


def bpm_is_request(state: Any, bpm: Optional[int]) -> bool:
    """Явная просьба юзера о темпе (``set_dj_mode(bpm=...)``), а не эхо.

    Модель повторяет ``set_dj_mode`` на каждом переходе и может вписать
    ``bpm`` из готового вызова (темп предстоящего/прошлого трека или базовый):
    это не просьба, темп остаётся дрейфующим. Уже зафиксированный темп
    любое новое значение только перезаписывает.
    """
    if bpm is None:
        return False
    if state.bpm_locked:
        return True
    upcoming = state.tracks_started + 1
    return bpm not in {state.set_bpm, state_bpm(state, upcoming), state_bpm(state, max(1, upcoming - 1))}


def apply_bpm_request(state: Any, bpm: Optional[int]) -> bool:
    """Применить явную просьбу о темпе: зафиксировать сет на ``bpm``.

    Returns:
        ``True`` — темп сета изменился (стоит записать в лог).
    """
    if not bpm_is_request(state, bpm):
        return False
    state.bpm_locked = True
    changed = bpm != state.set_bpm
    state.set_bpm = bpm
    return changed


def _base_bpm(value: Any) -> Optional[int]:
    """``base_bpm`` от модели → int в :data:`SET_BASE_BPM_RANGE` или ``None``."""
    if value is None or isinstance(value, bool):
        return None
    try:
        number = int(float(value))
    except (TypeError, ValueError):
        return None
    if number <= 0:
        return None
    return max(SET_BASE_BPM_RANGE[0], min(SET_BASE_BPM_RANGE[1], number))


def apply_set_character(
    state: Any, data: dict, *, fresh: bool, new_theme: bool, log: Optional[Callable[[str], None]] = None
) -> str:
    """Issue #3249 — характер сета из ``set_dj_mode(base_bpm=..., scale=...)``.

    Принимается только на генуинном старте (``fresh``) или с новой темой
    (``new_theme``): модель повторяет аргументы на каждом переходе, и эхо не
    должно двигать центр дрейфа. Итог пишется в ``log`` (контроллер DJ не
    растёт ветками — ADR-0145). ``base_bpm`` не трогает темп, который назвал юзер
    (``bpm_locked``). Неизвестный лад не применяется, но попадает в лог
    («отклонён»), а не теряется молча.

    Returns:
        Хвост строки лога (``"scale=major, base_bpm=132"``) или ``""``.
    """
    if not (fresh or new_theme):
        return ""
    parts: List[str] = []
    scale = str(data.get("scale") or "").strip().lower()
    if scale in SET_SCALES:
        state.set_scale = scale
        parts.append(f"scale={scale}")
    elif scale:
        parts.append(f"scale={scale!r} отклонён (не из {', '.join(SET_SCALES)})")
    base = _base_bpm(data.get("base_bpm"))
    if base is not None and not state.bpm_locked:
        state.set_bpm = base
        parts.append(f"base_bpm={base}")
    line = ", ".join(parts)
    if line and log is not None:
        log(f"🎧 DJ характер сета: {line}")
    return line
