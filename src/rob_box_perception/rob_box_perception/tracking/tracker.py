"""Шаг камерного трекера и жизненный цикл трека (ADR-0130 §2.1, §7 этап 2).

Жизненный цикл (поле ``state``, НЕ ``name_state`` — тот про имя, §2.6)::

    рождение ──► tentative ──(confirm_hits попаданий)──► confirmed
                    │                                     │  ▲
       нет обновлений > tentative_timeout_sec   нет > lost_after_sec
                    ▼                                     ▼  │ повторный захват
                 удалён                                  lost ┘ (тот же track_id)
                                                          │
                                         нет > delete_after_sec ──► удалён

Шаг :meth:`Tracker.step` — один пакет детекций одного момента (один кадр
одного датчика):

1. предсказать все треки на момент пакета;
2. сопоставить активные (tentative/confirmed) треки и детекции —
   венгерский алгоритм, ворота ``gate_m``, класс трека должен совпасть;
3. оставшиеся детекции — с потерянными треками (ворота
   ``reacquire_gate_m``): человек вышел из кадра и вернулся рядом;
4. из остатка ``birth_classes`` рождают tentative-треки;
5. по времени без обновления — переходы жизненного цикла;
6. сближение треков ближе ``resolution_m`` отмечается (ADR-0130 §6:
   потребитель имени сбрасывает ``confirmed → tentative`` у обоих).

Пакеты обязаны идти по неубыванию момента: пакет старше последнего
отбрасывается и считается в ``stats['late_batches_dropped']``. Переупорядочить
пакеты двух контейнеров (лицо приходит позже тела) — дело узла.

Детерминизм: треки обходятся по порядку рождения, детекции — в порядке
пакета, id — счётчик. Ни часов, ни случайности внутри.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from rob_box_perception.tracking.assignment import assign
from rob_box_perception.tracking.motion import ConstantVelocity2D
from rob_box_perception.tracking.params import TrackerParams

TENTATIVE = 'tentative'
CONFIRMED = 'confirmed'
LOST = 'lost'
TRACK_STATES = (TENTATIVE, CONFIRMED, LOST)

EVENT_BORN = 'born'
EVENT_CONFIRMED = 'confirmed'
EVENT_LOST = 'lost'
EVENT_REACQUIRED = 'reacquired'
EVENT_DELETED = 'deleted'
EVENT_CLOSE_ENCOUNTER = 'close_encounter'


@dataclass(frozen=True)
class Detection:
    """Одна детекция пакета в фрейме трекера (плоскость пола)."""

    class_name: str
    x: float
    y: float
    confidence: float = 1.0
    #: Разброс глубины из Observation.position_stddev_m; < 0 — нет оценки.
    position_std_m: float = -1.0
    #: Биометрическая улика без имени (Observation.candidate_id), '' — нет.
    candidate_id: str = ''
    source: str = ''


@dataclass(frozen=True)
class TrackSnapshot:
    """Неизменяемый снимок трека для потребителей (без имени, §5.1 п.1)."""

    track_id: str
    class_name: str
    state: str
    x: float
    y: float
    vx: float
    vy: float
    var_x: float
    var_y: float
    first_seen: float
    last_seen: float
    hits: int
    candidate_id: str
    sources: Tuple[str, ...]
    close_encounter_at: Optional[float]


@dataclass(frozen=True)
class TrackEvent:
    """Событие жизненного цикла: ``kind`` из ``EVENT_*``."""

    kind: str
    track_id: str
    stamp: float
    other_track_id: str = ''


@dataclass(frozen=True)
class StepResult:
    """Итог шага: живые треки и события этого шага."""

    tracks: Tuple[TrackSnapshot, ...]
    events: Tuple[TrackEvent, ...]


@dataclass
class _Track:
    """Изменяемое внутреннее состояние трека."""

    track_id: str
    class_name: str
    filt: ConstantVelocity2D
    first_seen: float
    last_seen: float
    state: str = TENTATIVE
    hits: int = 1
    candidate_id: str = ''
    sources: Tuple[str, ...] = ()
    close_encounter_at: Optional[float] = None
    close_to: Tuple[str, ...] = field(default_factory=tuple)

    def snapshot(self) -> TrackSnapshot:
        """Неизменяемая копия для потребителя."""
        x, y = self.filt.position
        vx, vy = self.filt.velocity
        var_x, var_y = self.filt.position_variance
        return TrackSnapshot(
            self.track_id, self.class_name, self.state, x, y, vx, vy,
            var_x, var_y, self.first_seen, self.last_seen, self.hits,
            self.candidate_id, self.sources, self.close_encounter_at)


class Tracker:
    """Камерный трекер: устойчивые ``track_id`` между кадрами."""

    def __init__(self, params: Optional[TrackerParams] = None) -> None:
        """Пустой трекер с параметрами ``params`` (по умолчанию — дефолты)."""
        self.params = params or TrackerParams()
        self._tracks: List[_Track] = []
        self._next_id = 1
        self._last_stamp: Optional[float] = None
        self.stats: Dict[str, int] = {
            'late_batches_dropped': 0,
            'unknown_class_dropped': 0,
            'births_refused_cap': 0,
        }

    # -- публичное ---------------------------------------------------------

    def tracks(self) -> Tuple[TrackSnapshot, ...]:
        """Снимки всех живых треков в порядке рождения."""
        return tuple(t.snapshot() for t in self._tracks)

    def step(self, stamp: float,
             detections: Sequence[Detection]) -> StepResult:
        """Учесть пакет детекций момента ``stamp`` (секунды)."""
        if self._last_stamp is not None and stamp < self._last_stamp:
            self.stats['late_batches_dropped'] += 1
            return StepResult(self.tracks(), ())
        self._predict_all(stamp)
        self._last_stamp = stamp
        events: List[TrackEvent] = []
        dets = self._known_classes(detections)
        active = [t for t in self._tracks if t.state != LOST]
        lost = [t for t in self._tracks if t.state == LOST]
        updated: List[_Track] = []
        dets = self._match(active, dets, self.params.gate_m, stamp,
                           updated, events)
        dets = self._match(lost, dets, self.params.reacquire_gate_m, stamp,
                           updated, events)
        self._age(stamp, updated, events)
        self._birth(dets, stamp, events)
        self._mark_close(stamp, events)
        return StepResult(self.tracks(), tuple(events))

    # -- шаги --------------------------------------------------------------

    def _known_classes(self, detections: Sequence[Detection]
                       ) -> List[Detection]:
        """Отбросить детекции классов вне ``class_map`` (со счётчиком)."""
        known = [d for d in detections if d.class_name in self.params.class_map]
        self.stats['unknown_class_dropped'] += len(detections) - len(known)
        return known

    def _predict_all(self, stamp: float) -> None:
        """Предсказать все треки на момент ``stamp``."""
        if self._last_stamp is None:
            return
        dt = stamp - self._last_stamp
        for track in self._tracks:
            track.filt.predict(dt, self.params.accel_noise)

    def _cost(self, track: _Track, det: Detection,
              gate: float) -> Optional[float]:
        """Стоимость пары, м; ``None`` — пара запрещена."""
        if self.params.class_map[det.class_name] != track.class_name:
            return None
        x, y = track.filt.position
        dist = math.hypot(det.x - x, det.y - y)
        if dist > gate:
            return None
        if track.candidate_id and det.candidate_id:
            if track.candidate_id == det.candidate_id:
                return max(0.0, dist - self.params.candidate_match_bonus_m)
            return dist + self.params.candidate_mismatch_penalty_m
        return dist

    def _match(self, tracks: List[_Track], dets: List[Detection],
               gate: float, stamp: float, updated: List[_Track],
               events: List[TrackEvent]) -> List[Detection]:
        """Сопоставить и обновить; вернуть детекции, оставшиеся без пары."""
        costs = [[self._cost(t, d, gate) for d in dets] for t in tracks]
        used = set()
        for ti, di in assign(costs):
            self._apply(tracks[ti], dets[di], stamp, events)
            updated.append(tracks[ti])
            used.add(di)
        return [d for i, d in enumerate(dets) if i not in used]

    def _meas_var(self, det: Detection) -> float:
        """Дисперсия измерения: шум модели плюс разброс глубины."""
        var = self.params.meas_std_m ** 2
        if det.position_std_m > 0.0:
            var += det.position_std_m ** 2
        return var

    def _apply(self, track: _Track, det: Detection, stamp: float,
               events: List[TrackEvent]) -> None:
        """Обновить трек детекцией и перевести по жизненному циклу."""
        track.filt.update(det.x, det.y, self._meas_var(det))
        track.last_seen = stamp
        track.hits += 1
        if det.candidate_id:
            track.candidate_id = det.candidate_id
        if det.source and det.source not in track.sources:
            track.sources = tuple(sorted(track.sources + (det.source,)))
        if track.state == LOST:
            track.state = CONFIRMED
            events.append(TrackEvent(EVENT_REACQUIRED, track.track_id, stamp))
        elif (track.state == TENTATIVE
              and track.hits >= self.params.confirm_hits):
            track.state = CONFIRMED
            events.append(TrackEvent(EVENT_CONFIRMED, track.track_id, stamp))

    def _age(self, stamp: float, updated: List[_Track],
             events: List[TrackEvent]) -> None:
        """Состарить треки без обновления на этом шаге."""
        keep: List[_Track] = []
        for track in self._tracks:
            if any(track is u for u in updated) or not self._expire(
                    track, stamp, events):
                keep.append(track)
        self._tracks = keep

    def _expire(self, track: _Track, stamp: float,
                events: List[TrackEvent]) -> bool:
        """Переход по времени без обновления; True — трек удалить."""
        p = self.params
        age = stamp - track.last_seen
        if track.state == TENTATIVE and age > p.tentative_timeout_sec:
            events.append(TrackEvent(EVENT_DELETED, track.track_id, stamp))
            return True
        if track.state == CONFIRMED and age > p.lost_after_sec:
            track.state = LOST
            track.filt.stop(p.init_vel_std ** 2)
            events.append(TrackEvent(EVENT_LOST, track.track_id, stamp))
        if track.state == LOST and age > p.delete_after_sec:
            events.append(TrackEvent(EVENT_DELETED, track.track_id, stamp))
            return True
        return False

    def _birth(self, dets: List[Detection], stamp: float,
               events: List[TrackEvent]) -> None:
        """Родить tentative-треки из оставшихся детекций ``birth_classes``."""
        p = self.params
        for det in dets:
            if det.class_name not in p.birth_classes:
                continue
            if len(self._tracks) >= p.max_tracks:
                self.stats['births_refused_cap'] += 1
                continue
            track = _Track(
                track_id=f'{p.id_prefix}{self._next_id}',
                class_name=p.class_map[det.class_name],
                filt=ConstantVelocity2D(det.x, det.y, self._meas_var(det),
                                        p.init_vel_std ** 2),
                first_seen=stamp, last_seen=stamp,
                candidate_id=det.candidate_id,
                sources=(det.source,) if det.source else ())
            self._next_id += 1
            self._tracks.append(track)
            events.append(TrackEvent(EVENT_BORN, track.track_id, stamp))
            if p.confirm_hits <= 1:
                track.state = CONFIRMED
                events.append(
                    TrackEvent(EVENT_CONFIRMED, track.track_id, stamp))

    def _mark_close(self, stamp: float, events: List[TrackEvent]) -> None:
        """Отметить пары треков ближе ``resolution_m`` (ADR-0130 §6)."""
        active = [t for t in self._tracks if t.state != LOST]
        close: Dict[str, List[str]] = {t.track_id: [] for t in active}
        for i, a in enumerate(active):
            for b in active[i + 1:]:
                if a.class_name == b.class_name and self._dist(a, b) < \
                        self.params.resolution_m:
                    close[a.track_id].append(b.track_id)
                    close[b.track_id].append(a.track_id)
        for track in active:
            now = tuple(close[track.track_id])
            for other in now:
                if other not in track.close_to and track.track_id < other:
                    events.append(TrackEvent(EVENT_CLOSE_ENCOUNTER,
                                             track.track_id, stamp, other))
            if now:
                track.close_encounter_at = stamp
            track.close_to = now

    @staticmethod
    def _dist(a: _Track, b: _Track) -> float:
        """Расстояние между оценками положения двух треков, м."""
        ax, ay = a.filt.position
        bx, by = b.filt.position
        return math.hypot(ax - bx, ay - by)
