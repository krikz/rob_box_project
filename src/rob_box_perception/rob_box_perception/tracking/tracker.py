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
2. две фазы: сперва детекции тела (не ``attach_classes``), затем лица —
   так лицо того же пакета находит трек, только что заведённый телом.
   В каждой фазе:

   a. сопоставить активные (tentative/confirmed) треки и детекции —
      венгерский алгоритм, ворота ``gate_m``, класс трека должен совпасть;
   b. оставшиеся детекции — с потерянными треками (ворота
      ``reacquire_gate_m``): человек вышел из кадра и вернулся рядом;
   c. лицо из остатка прикрепляется к треку с телом того же момента
      рядом (``attach_m``, ``attach_window_sec``) — без дубля (ADR-0161 §2);
   d. из остатка ``birth_classes`` рождают tentative-треки; трек,
      рождённый лицом, — без тела (``has_body=False``), пока тело не придёт;
3. по времени без обновления — переходы жизненного цикла;
4. сближение треков ближе ``resolution_m`` отмечается (ADR-0130 §6:
   потребитель имени сбрасывает ``confirmed → tentative`` у обоих).

Смена фрейма трекера (``base_link`` ↔ ``map``, ADR-0161 §1) — явный
:meth:`Tracker.reset`: все треки удаляются с событием ``deleted``, новые
рождаются в новом фрейме с новыми id. Пересчёта треков между фреймами нет.

Пакеты обязаны идти по неубыванию момента: пакет старше последнего
отбрасывается и считается в ``stats['late_batches_dropped']``. Переупорядочить
пакеты двух контейнеров (лицо приходит позже тела) — дело узла.

Детерминизм: треки обходятся по порядку рождения, детекции — в порядке
пакета, id — счётчик. Ни часов, ни случайности внутри.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from rob_box_perception.tracking.assignment import assign
from rob_box_perception.tracking.motion import ConstantVelocity2D
from rob_box_perception.tracking.params import TrackerParams
from rob_box_perception.tracking.proximity import (
    attach_target,
    close_neighbours,
    match_cost,
)

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
    #: Было ли у трека тело (не только лицо). ``False`` — трек рождён лицом
    #: и примет одежды у него нет (ADR-0161 §2).
    has_body: bool = False


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
    #: Момент последнего обновления детекцией тела; ``None`` — тела не было.
    last_body: Optional[float] = None
    #: Момент последнего лица (сопоставлено или прикреплено).
    last_attached: Optional[float] = None

    def snapshot(self) -> TrackSnapshot:
        """Неизменяемая копия для потребителя."""
        x, y = self.filt.position
        vx, vy = self.filt.velocity
        var_x, var_y = self.filt.position_variance
        return TrackSnapshot(
            self.track_id, self.class_name, self.state, x, y, vx, vy,
            var_x, var_y, self.first_seen, self.last_seen, self.hits,
            self.candidate_id, self.sources, self.close_encounter_at,
            self.last_body is not None)


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
            'faces_attached': 0,
            'frame_resets': 0,
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
        attach = self.params.attach_classes
        updated: List[_Track] = []
        for phase in ([d for d in dets if d.class_name not in attach],
                      [d for d in dets if d.class_name in attach]):
            updated += self._phase(phase, stamp, updated, events)
        self._age(stamp, updated, events)
        self._mark_close(stamp, events)
        return StepResult(self.tracks(), tuple(events))

    def reset(self, stamp: float) -> Tuple[TrackEvent, ...]:
        """Удалить все треки (смена фрейма, ADR-0161 §1); вернуть события.

        Счётчик id не сбрасывается: новые треки нового фрейма получают
        новые ``track_id``, старые id не переиспользуются.
        """
        events = tuple(TrackEvent(EVENT_DELETED, t.track_id, stamp)
                       for t in self._tracks)
        self._tracks = []
        self._last_stamp = stamp
        self.stats['frame_resets'] += 1
        return events

    # -- шаги --------------------------------------------------------------

    def _phase(self, dets: List[Detection], stamp: float,
               updated: List[_Track],
               events: List[TrackEvent]) -> List[_Track]:
        """Сопоставить, прикрепить, родить; вернуть рождённые треки."""
        active = [t for t in self._tracks if t.state != LOST]
        lost = [t for t in self._tracks if t.state == LOST]
        dets = self._match(active, dets, self.params.gate_m, stamp,
                           updated, events)
        dets = self._match(lost, dets, self.params.reacquire_gate_m, stamp,
                           updated, events)
        dets = [d for d in dets if not self._attach(d, stamp)]
        return self._birth(dets, stamp, events)

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

    def _match(self, tracks: List[_Track], dets: List[Detection],
               gate: float, stamp: float, updated: List[_Track],
               events: List[TrackEvent]) -> List[Detection]:
        """Сопоставить и обновить; вернуть детекции, оставшиеся без пары."""
        costs = [[match_cost(t, d, gate, self.params) for d in dets]
                 for t in tracks]
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
        self._note_source(track, det, stamp)
        if track.state == LOST:
            track.state = CONFIRMED
            events.append(TrackEvent(EVENT_REACQUIRED, track.track_id, stamp))
        elif (track.state == TENTATIVE
              and track.hits >= self.params.confirm_hits):
            track.state = CONFIRMED
            events.append(TrackEvent(EVENT_CONFIRMED, track.track_id, stamp))

    def _note_source(self, track: _Track, det: Detection,
                     stamp: float) -> None:
        """Запомнить источник и вид детекции (тело или лицо)."""
        if det.source and det.source not in track.sources:
            track.sources = tuple(sorted(track.sources + (det.source,)))
        if det.class_name in self.params.attach_classes:
            track.last_attached = stamp
        else:
            track.last_body = stamp

    def _attach(self, det: Detection, stamp: float) -> bool:
        """Прикрепить лицо к треку тела того же момента; True — прикреплено.

        Какой трек годится — :func:`.proximity.attach_target`.
        Фильтр не обновляется: точка лица за воротами — не
        измерение положения тела, а улика «это лицо этого тела».
        """
        if det.class_name not in self.params.attach_classes:
            return False
        active = [t for t in self._tracks if t.state != LOST]
        track = attach_target(active, det, stamp, self.params)
        if track is None:
            return False
        if det.candidate_id:
            track.candidate_id = det.candidate_id
        self._note_source(track, det, stamp)
        self.stats['faces_attached'] += 1
        return True

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
               events: List[TrackEvent]) -> List[_Track]:
        """Родить tentative-треки из оставшихся детекций ``birth_classes``."""
        p = self.params
        born: List[_Track] = []
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
                sources=())
            self._note_source(track, det, stamp)
            self._next_id += 1
            self._tracks.append(track)
            born.append(track)
            events.append(TrackEvent(EVENT_BORN, track.track_id, stamp))
            if p.confirm_hits <= 1:
                track.state = CONFIRMED
                events.append(
                    TrackEvent(EVENT_CONFIRMED, track.track_id, stamp))
        return born

    def _mark_close(self, stamp: float, events: List[TrackEvent]) -> None:
        """Отметить пары треков ближе ``resolution_m`` (ADR-0130 §6)."""
        active = [t for t in self._tracks if t.state != LOST]
        close = close_neighbours(active, self.params.resolution_m)
        for track in active:
            now = close[track.track_id]
            for other in now:
                if other not in track.close_to and track.track_id < other:
                    events.append(TrackEvent(EVENT_CLOSE_ENCOUNTER,
                                             track.track_id, stamp, other))
            if now:
                track.close_encounter_at = stamp
            track.close_to = now
