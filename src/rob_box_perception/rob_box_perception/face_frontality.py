#!/usr/bin/env python3
"""face_frontality.py — можно ли по Встрече ЗАВЕСТИ новую запись лица (ADR-0130 §10.4 п.2).

Дешёвый фикс 1.1 по решению владельца 04.11.2026: «не заводить новую
запись по нефронтальной встрече или по кадру, где нет лица». Узнавание
уже существующих записей этот модуль не трогает — он отсекает ТОЛЬКО
рождение новой записи: нефронтальную Встречу незнакомца не пускает в
``FaceStore.record_encounter`` (там, ``face_store.py`` ветка «Незнакомец»,
и рождается запись), а известного человека пускает как раньше.

Откуда фронтальность. Других признаков позы в данных нет: RetinaFace
(``vision_face_loader._post_process_faces``) отдаёт на детекцию 5 точек
``landmarks`` — плоский список из 10 float, normalized [0, 1] в
координатах кадра, порядок left_eye, right_eye, nose, mouth_left,
mouth_right (тот же контракт читает ``face_embedding.align_face``).
Углов yaw/pitch детектор не даёт, поэтому считаем два безразмерных
отношения по этим точкам, предварительно сняв наклон головы (roll):

* ``yaw_ratio`` — смещение носа от середины глаз вдоль линии глаз,
  в долях межглазного расстояния. Анфас ≈ 0 (шаблон ArcFace: 0.003),
  поворот головы растит модуль, профиль — около 0.5 и больше;
* ``pitch_ratio`` — где нос между линией глаз (0) и линией рта (1).
  Анфас ≈ 0.5 (шаблон ArcFace: 0.495), кивок вниз/вверх сдвигает.

«Кадр, где нет лица» — детекция без landmarks или с точками, которые
не складываются в лицо (глаза слиплись, рот не ниже глаз, нос вне
лица). Такой кадр не фронтальный: подтвердить лицо нечем.

Пороги ниже — стартовые, по геометрии, НЕ откалиброваны на роботе.
Грубая оценка: ``|yaw_ratio| = 0.25`` соответствует повороту головы
около 20°. Все пороги — только здесь; остальные модули импортируют.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Optional, Sequence

# ============================================================================
# Пороги (единственное место)
# ============================================================================

#: Предел |yaw_ratio| для фронтального кадра (≈ 20° поворота, оценка).
MAX_YAW_RATIO = 0.25

#: Допустимая полоса pitch_ratio для фронтального кадра (анфас ≈ 0.5).
MIN_PITCH_RATIO = 0.30
MAX_PITCH_RATIO = 0.70

#: Доля фронтальных кадров среди лучших кадров Встречи (тех, что несут
#: эмбеддинг), при которой по Встрече можно завести новую запись.
#: Эмбеддинг Встречи — среднее этих кадров, и семя новой галереи не
#: должно наполовину состоять из профиля.
MIN_FRONTAL_SHARE = 0.5

#: Межглазное расстояние меньше этого (в пикселях кадра) — точки
#: слиплись, лица по ним не видно.
MIN_EYE_DISTANCE_PX = 2.0


# ============================================================================
# Поза одного кадра
# ============================================================================

@dataclass(frozen=True)
class FacePose:
    """Безразмерные признаки позы по 5 landmarks."""

    yaw_ratio: float
    pitch_ratio: float

    @property
    def frontal(self) -> bool:
        return (
            abs(self.yaw_ratio) <= MAX_YAW_RATIO
            and MIN_PITCH_RATIO <= self.pitch_ratio <= MAX_PITCH_RATIO
        )


def estimate_pose(
    landmarks: Optional[Sequence[float]],
    frame_w: int,
    frame_h: int,
) -> Optional[FacePose]:
    """Поза по 5 landmarks, либо ``None``, если лица по точкам не видно.

    ``frame_w``/``frame_h`` нужны, чтобы перевести normalized точки в
    пиксели: иначе неквадратный кадр исказит отношения. Если размер кадра
    неизвестен (0), считаем в normalized координатах.
    """
    if landmarks is None:
        return None
    try:
        pts = [float(v) for v in landmarks]
    except (TypeError, ValueError):
        return None
    if len(pts) != 10 or not all(math.isfinite(v) for v in pts):
        return None

    sx = float(frame_w) if frame_w > 0 else 1.0
    sy = float(frame_h) if frame_h > 0 else 1.0
    xy = [(pts[2 * k] * sx, pts[2 * k + 1] * sy) for k in range(5)]
    (lex, ley), (rex, rey), (nx, ny), (mlx, mly), (mrx, mry) = xy

    ex, ey = rex - lex, rey - ley
    eye_dist = math.hypot(ex, ey)
    min_dist = MIN_EYE_DISTANCE_PX if frame_w > 0 and frame_h > 0 else 1e-6
    if eye_dist < min_dist:
        return None
    # Ось глаз (u) и перпендикуляр «вниз по лицу» (v) — снимаем roll.
    ux, uy = ex / eye_dist, ey / eye_dist
    vx, vy = -uy, ux

    mx, my = (lex + rex) / 2.0, (ley + rey) / 2.0
    mouth_x, mouth_y = (mlx + mrx) / 2.0, (mly + mry) / 2.0

    nose_u = (nx - mx) * ux + (ny - my) * uy
    nose_v = (nx - mx) * vx + (ny - my) * vy
    mouth_v = (mouth_x - mx) * vx + (mouth_y - my) * vy
    if mouth_v <= 0.0:
        # Рот не ниже глаз — это не лицо (или лицо вверх ногами).
        return None

    return FacePose(yaw_ratio=nose_u / eye_dist, pitch_ratio=nose_v / mouth_v)


def is_frontal(
    landmarks: Optional[Sequence[float]],
    frame_w: int,
    frame_h: int,
) -> bool:
    """Фронтален ли кадр. Нет точек / не лицо — не фронтален."""
    pose = estimate_pose(landmarks, frame_w, frame_h)
    return pose is not None and pose.frontal


# ============================================================================
# Решение по Встрече
# ============================================================================

@dataclass(frozen=True)
class BirthDecision:
    """Можно ли по Встрече завести новую запись лица, и почему."""

    allowed: bool
    frontal_frames: int
    considered_frames: int
    reason: str


def new_record_decision(best_frames: Iterable[Any]) -> BirthDecision:
    """Решить по лучшим кадрам Встречи, можно ли родить новую запись.

    ``best_frames`` — ``FaceEncounter.best``: объекты с полями
    ``embedding``, ``landmarks``, ``frame_w``, ``frame_h``. Учитываются
    только кадры с эмбеддингом — именно они составили эмбеддинг Встречи.
    """
    considered = 0
    frontal = 0
    for obs in best_frames:
        if getattr(obs, 'embedding', None) is None:
            continue
        considered += 1
        if is_frontal(
            getattr(obs, 'landmarks', None),
            int(getattr(obs, 'frame_w', 0) or 0),
            int(getattr(obs, 'frame_h', 0) or 0),
        ):
            frontal += 1
    if considered == 0:
        return BirthDecision(False, 0, 0, 'нет кадров с эмбеддингом')
    if frontal == 0:
        return BirthDecision(
            False, 0, considered,
            'нет фронтального кадра (профиль, нет landmarks или не лицо)',
        )
    if frontal / considered < MIN_FRONTAL_SHARE:
        return BirthDecision(
            False, frontal, considered,
            f'фронтальных кадров {frontal}/{considered} < {MIN_FRONTAL_SHARE:.0%}',
        )
    return BirthDecision(True, frontal, considered, 'фронтальная встреча')


@dataclass(frozen=True)
class EncounterGate:
    """Писать ли Встречу в ``FaceStore`` — и что сказать в лог, если нет."""

    record: bool
    #: 1, если отсечено именно рождение новой записи (для счётчика ноды).
    suppressed_birth: int = 0
    level: str = 'info'
    reason: str = ''


def encounter_gate(
    encounter: Any,
    identify: Callable[[Any], Optional[Any]],
    min_embed_px: float,
) -> EncounterGate:
    """Решить, идёт ли Встреча в ``FaceStore.record_encounter``.

    * Нет эмбеддинга — писать нечего (как и раньше).
    * Встреча фронтальная — пишем: узнаём или заводим новую запись.
    * Нефронтальная / без лица — пишем ТОЛЬКО если человек уже известен
      (``identify`` — тот же ``FaceStore.identify``: тот же порог и тот же
      разбор спора, что в ``record_encounter``, но без записи). Незнакомца
      не пишем: новая запись по такой Встрече не рождается.
    """
    if encounter.embedding is None:
        # Трек дожил до Встречи, но ни один кадр не дал эмбеддинга. Причину строим из
        # известного (issue #4544): размер лица встречи; про отброшенные кропы — в сводке.
        if encounter.max_face_px < min_embed_px:
            cause = f'лицо мельче {min_embed_px:.0f}px (max {encounter.max_face_px:.0f}px)'
        else:
            cause = (
                f'лицо {encounter.max_face_px:.0f}px крупнее порога {min_embed_px:.0f}px, '
                f'но нет ни одного годного кропа: причины — кропов_отброшено в сводке [лицо]'
            )
        return EncounterGate(
            record=False, level='warn',
            reason=f'Встреча track={encounter.track_id} без эмбеддинга — запись пропущена ({cause}).',
        )
    birth = new_record_decision(encounter.best)
    if birth.allowed or identify(encounter.embedding) is not None:
        return EncounterGate(record=True)
    return EncounterGate(
        record=False, suppressed_birth=1, level='info',
        reason=(
            f'Встреча track={encounter.track_id}: незнакомец, новая запись '
            f'не заведена — {birth.reason} (фронтальных '
            f'{birth.frontal_frames}/{birth.considered_frames}, '
            f'face={encounter.max_face_px:.0f}px).'
        ),
    )
