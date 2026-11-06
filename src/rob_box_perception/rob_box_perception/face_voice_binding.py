"""Доказательство «этот голос — это лицо в кадре» (issue #4357, ADR-0123 §6).

Правило ADR-0123 §6 было «голос опознан + в ПОСЛЕДНЕМ кадре ровно одно лицо → имя голоса этому лицу». На роботе
04.11.2026 оно отдало имя «Дэнчик» и ``speaker_id`` его голоса напечатанному фото на кресле (запись ``566331d3``):
Дэнчик говорил рядом, вне кадра, а единственным лицом в кадре было фото. Одно лицо в кадре — не доказательство, что
говорит оно.

Здесь решение принимает код по наблюдениям кадра, а не по одному снимку. Привязка — только когда сошлись все три
признака:

1. **Одно лицо всё окно фразы.** Во ВСЕХ кадрах окна ``window_sec`` перед результатом голоса — не больше одного лица,
   и все узнанные — одна запись; таких кадров не меньше ``min_frames``. Два человека по очереди — не «одно лицо».
2. **Лицо говорит.** Рот этого лица менялся за окно: отношение ширины рта к расстоянию между глазами (5 точек
   RetinaFace, ``det['landmarks']``) гуляет не меньше чем на ``min_mouth_spread``. Отношение не зависит от масштаба,
   сдвига и (в первом приближении) поворота головы — фото, которое качается вместе с роботом, его не меняет, а речь
   меняет. Точек нет — доказательства нет, привязки нет.
3. **Повторилось.** Пункты 1–2 выполнились для той же пары «голос — лицо» в ``confirmations`` разных фразах,
   разнесённых не меньше чем на окно (окна не перекрываются — наблюдения независимы). Та же проверка с другим
   лицом сбрасывает счёт.

Пороги (``DEFAULT_*`` ниже) выбраны рассуждением, на роботе НЕ откалиброваны: измеренный разброс рта пишется в
причину отказа (:attr:`BindingVerdict.detail`), по логу ``vision-face`` его можно сверить с живыми людьми и с фото.

Чистый модуль: ни ROS, ни хранилища; время передаёт вызывающий. Что делать с вердиктом (запись имени, запрет
перезаписи чужого имени) решает ``FaceRecognizer.note_voice_identification``.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

#: Сколько кадров окна должно содержать то самое лицо: тик ноды ~0.5 с, окно 6 с — до 12 кадров.
DEFAULT_MIN_FRAMES = 4

#: Минимальный разброс отношения «ширина рта / расстояние между глазами» за окно, доля медианы. Догадка: шум
#: точек RetinaFace на лице ~150 px — пиксель-два на угол рта (единицы процентов), речь меняет ширину рта на
#: десятки процентов. Не откалибровано на роботе.
DEFAULT_MIN_MOUTH_SPREAD = 0.10

#: Сколько независимых фраз должно подтвердить пару «голос — лицо», прежде чем имя ляжет в галерею.
DEFAULT_CONFIRMATIONS = 3

#: Причины вердикта (они же ключи счётчиков ``FaceRecognizer.stats``).
STALE = 'stale'
NO_FACE = 'no_face'
MULTI_FACE = 'multi_face'
NOT_SPEAKING = 'not_speaking'
PENDING = 'pending'
BIND = 'bind'
#: Отказы после вердикта ``BIND`` — по хранилищу (:func:`store_veto`).
CONFLICT = 'conflict'
NAMED_OTHER = 'named_other'

#: Причина → ключ счётчика в ``FaceRecognizer.stats()`` (issue #2748: сводка говорит, ПОЧЕМУ слияний 0).
STAT_KEYS: Dict[str, str] = {
    STALE: 'voice_merge_skip_stale',
    NO_FACE: 'voice_merge_skip_no_face',
    MULTI_FACE: 'voice_merge_skip_multi_face',
    CONFLICT: 'voice_merge_skip_conflict',
    NOT_SPEAKING: 'voice_merge_skip_not_speaking',
    PENDING: 'voice_merge_pending',
    NAMED_OTHER: 'voice_merge_skip_named_other',
}

#: Индексы 5 точек лица в плоском списке ``landmarks`` (x, y попарно): глаза, нос, углы рта.
_LEFT_EYE, _RIGHT_EYE, _MOUTH_LEFT, _MOUTH_RIGHT = 0, 1, 3, 4


@dataclass(frozen=True)
class BindingVerdict:
    """Исход проверки: ``kind`` — одна из причин модуля; ``person_id`` — лицо окна (если оно одно); ``detail`` —
    человекочитаемые числа для лога (разброс рта, счёт подтверждений)."""

    kind: str
    person_id: Optional[str] = None
    detail: str = ''


@dataclass(frozen=True)
class _Frame:
    ts: float
    person_ids: Tuple[Optional[str], ...]
    mouth_ratios: Tuple[Optional[float], ...]


def mouth_ratio(landmarks: Any, frame_w: int, frame_h: int) -> Optional[float]:
    """Ширина рта / расстояние между глазами по 5 точкам лица (normalized [0,1] в координатах кадра).

    ``None`` — точек нет, их не 10 чисел или глаза слиплись (вырожденная детекция).
    """
    if landmarks is None:
        return None
    try:
        pts = [float(v) for v in landmarks]
    except (TypeError, ValueError):
        return None
    if len(pts) != 10:
        return None

    def point(i: int) -> Tuple[float, float]:
        return pts[2 * i] * frame_w, pts[2 * i + 1] * frame_h

    eyes = math.dist(point(_LEFT_EYE), point(_RIGHT_EYE))
    if eyes < 1e-6:
        return None
    return math.dist(point(_MOUTH_LEFT), point(_MOUTH_RIGHT)) / eyes


def mouth_spread(ratios: Sequence[float]) -> Optional[float]:
    """Разброс отношения рта за окно, доля медианы. Пять и больше отсчётов — без крайних (одиночный выброс детектора
    не делает фото говорящим). ``None`` — отсчётов меньше двух."""
    vals = sorted(ratios)
    if len(vals) < 2:
        return None
    if len(vals) >= 5:
        vals = vals[1:-1]
    median = vals[len(vals) // 2]
    if median <= 0:
        return None
    return (vals[-1] - vals[0]) / median


class VoiceFaceBinding:
    """Кадры последних секунд и счёт подтверждений пары «голос — лицо» (issue #4357)."""

    def __init__(
        self,
        *,
        window_sec: float,
        min_frames: int = DEFAULT_MIN_FRAMES,
        min_mouth_spread: float = DEFAULT_MIN_MOUTH_SPREAD,
        confirmations: int = DEFAULT_CONFIRMATIONS,
    ) -> None:
        self._window = float(window_sec)
        self._min_frames = max(1, int(min_frames))
        self._min_spread = float(min_mouth_spread)
        self._confirmations = max(1, int(confirmations))
        self._frames: Deque[_Frame] = deque()
        #: speaker_id → (person_id, сколько фраз подтвердили, время последнего подтверждения).
        self._candidates: Dict[str, Tuple[str, int, float]] = {}
        #: Сколько раз голос НЕ привязан, по причинам (:data:`STAT_KEYS`).
        self.skips: Dict[str, int] = {kind: 0 for kind in STAT_KEYS}

    def count(self, kind: str) -> None:
        """Отказ ``kind`` в счётчик (вердикты :meth:`judge` считает сам; отказы хранилища — вызывающий)."""
        self.skips[kind] += 1

    def stats(self) -> Dict[str, int]:
        """Счётчики отказов под ключами ``FaceRecognizer.stats()``."""
        return {STAT_KEYS[kind]: n for kind, n in self.skips.items()}

    def note_frame(
        self,
        ts: float,
        person_ids: Sequence[Optional[str]],
        mouth_ratios: Sequence[Optional[float]] = (),
    ) -> None:
        """Кадр: по одному ``person_id`` (``None`` — не узнан) и отношению рта на детекцию. Пустой кадр тоже кадр:
        «в окне никого» отличается от «камера молчит»."""
        ratios = tuple(mouth_ratios) + (None,) * (len(person_ids) - len(mouth_ratios))
        self._frames.append(_Frame(float(ts), tuple(person_ids), ratios[:len(person_ids)]))
        while self._frames and ts - self._frames[0].ts > self._window:
            self._frames.popleft()

    def judge(self, speaker_id: str, ts: float) -> BindingVerdict:
        """Вердикт по голосу ``speaker_id``, опознанному в момент ``ts``; подтверждения и отказы считает здесь же."""
        verdict = self._verdict(speaker_id, ts)
        if verdict.kind != BIND:
            self.count(verdict.kind)
        return verdict

    def _verdict(self, speaker_id: str, ts: float) -> BindingVerdict:
        frames = [f for f in self._frames if ts - self._window <= f.ts <= ts]
        if not frames:
            return BindingVerdict(STALE)
        sole = self._sole_person(frames)
        if not isinstance(sole, str):
            return sole
        spread = self._speaking_spread(frames, sole)
        if spread is None or spread < self._min_spread:
            shown = 'нет точек рта' if spread is None else f'{spread:.3f}'
            return BindingVerdict(NOT_SPEAKING, sole, f'рот={shown} < {self._min_spread:.2f}')
        return self._confirm(speaker_id, sole, ts, spread)

    def _sole_person(self, frames: List[_Frame]) -> Any:
        """Единственная запись окна (``str``) или вердикт отказа: нет лица / не одно лицо."""
        if any(len(f.person_ids) > 1 for f in frames):
            return BindingVerdict(MULTI_FACE)
        known = {pid for f in frames for pid in f.person_ids if pid is not None}
        if len(known) > 1:
            return BindingVerdict(MULTI_FACE, detail='разные лица в окне')
        if not known:
            return BindingVerdict(NO_FACE)
        person_id = known.pop()
        seen = sum(1 for f in frames if person_id in f.person_ids)
        if seen < self._min_frames:
            return BindingVerdict(NOT_SPEAKING, person_id, f'кадров с лицом {seen} < {self._min_frames}')
        return person_id

    @staticmethod
    def _speaking_spread(frames: List[_Frame], person_id: str) -> Optional[float]:
        ratios = [
            r for f in frames for pid, r in zip(f.person_ids, f.mouth_ratios)
            if pid == person_id and r is not None
        ]
        return mouth_spread(ratios)

    def _confirm(self, speaker_id: str, person_id: str, ts: float, spread: float) -> BindingVerdict:
        prev = self._candidates.get(speaker_id)
        if prev is None or prev[0] != person_id:
            count, last = 1, ts
        elif ts - prev[2] >= self._window:
            count, last = prev[1] + 1, ts
        else:
            count, last = prev[1], prev[2]  # то же окно — не новое доказательство
        self._candidates[speaker_id] = (person_id, count, last)
        detail = f'рот={spread:.3f} подтверждений {count}/{self._confirmations}'
        if count < self._confirmations:
            return BindingVerdict(PENDING, person_id, detail)
        self._candidates.pop(speaker_id, None)
        return BindingVerdict(BIND, person_id, detail)


def skip_note(verdict: BindingVerdict, speaker_id: str, name: str) -> Optional[str]:
    """Строка лога для отказа с числами (рот, подтверждения) — по ней калибруются пороги; прочим — ``None``."""
    if verdict.kind not in (NOT_SPEAKING, PENDING) or not verdict.person_id:
        return None
    what = 'имя не привязано — лицо не говорило' if verdict.kind == NOT_SPEAKING else 'ждём ещё фраз'
    return (f'[слияние] голос {speaker_id[:8]} «{name}» и лицо {verdict.person_id[:8]}: {what} '
            f'({verdict.detail}).')


def store_veto(store: Any, person_id: str, speaker_id: str, name: str) -> Optional[Tuple[str, str]]:
    """Почему доказанную пару нельзя записать в галерею: ``(причина, строка лога)`` или ``None`` — можно.

    Голос уже привязан к другому лицу — разбор дублей голосовых профилей живёт отдельно (ADR-0123 §9.1). У лица уже
    другое имя или другой голос — не перетираем (issue #4357: имя «Литвин», поставленное Шифу руками, голос Дэнчика
    перетёр бы при следующей фразе). Записи нет — тоже нельзя.
    """
    existing = store.find_by_speaker(speaker_id)
    if existing is not None and existing != person_id:
        return CONFLICT, (f'Голос {speaker_id[:8]} уже привязан к лицу {existing[:8]}, а в кадре {person_id[:8]} — '
                          f'слияние пропущено (дубли профилей, ADR-0123 §9.1).')
    record = next((p for p in store.people() if p.get('person_id') == person_id), None)
    if record is None:
        return NAMED_OTHER, f'Лица {person_id[:8]} нет в галерее — имя «{name}» не привязано.'
    face_name, face_speaker = record.get('name'), record.get('speaker_id')
    other_name = bool(face_name) and str(face_name).strip().lower() != name.strip().lower()
    if other_name or (face_speaker and face_speaker != speaker_id):
        return NAMED_OTHER, (f'Лицо {person_id[:8]} уже «{face_name or "?"}» (голос {str(face_speaker or "-")[:8]}) — '
                             f'имя «{name}» голосом не перетираю (issue #4357).')
    return None


__all__ = [
    'BIND', 'BindingVerdict', 'CONFLICT', 'MULTI_FACE', 'NAMED_OTHER', 'NOT_SPEAKING', 'NO_FACE', 'PENDING',
    'STALE', 'STAT_KEYS', 'VoiceFaceBinding', 'mouth_ratio', 'mouth_spread', 'skip_note', 'store_veto',
]
