"""Камерный трекер восприятия v2 — чистое ядро без ROS (ADR-0130 §7 этап 2).

Слой «Трек» ADR-0130 §2.1: «где физически объект и куда движется».
Ядро сопоставляет наблюдения между кадрами, ведёт устойчивые ``track_id``
и жизненный цикл ``tentative → confirmed → lost``. Имён не знает
(ADR-0130 §5.1 п.1): ``candidate_id`` наблюдения хранится как улика, но
имя и ``name_state`` — дело реестра Знакомых (этапы 3–4).

Модули:

* :mod:`.params` — все параметры трекера в одном месте;
* :mod:`.motion` — фильтр Калмана с постоянной скоростью на плоскости;
* :mod:`.assignment` — оптимальное сопоставление (венгерский алгоритм);
* :mod:`.tracker` — жизненный цикл и шаг трекера;
* :mod:`.proximity` — лицо к телу того же момента, сближение треков;
* :mod:`.frame_policy` — фрейм трекера: ``map`` или ``base_link`` (ADR-0161);
* :mod:`.observation_adapter` — ``Observation`` → :class:`Detection`.

ROS-нода (тонкий слой над ядром) — следующий PR, см.
``docs/plans/2026-11-04-adr-0130-stage2-tracker-plan.md``; решения
этапа — ADR-0161.
"""

from rob_box_perception.tracking.frame_policy import FramePolicy
from rob_box_perception.tracking.params import TrackerParams
from rob_box_perception.tracking.tracker import (
    Detection,
    StepResult,
    Tracker,
    TrackEvent,
    TrackSnapshot,
)

__all__ = [
    'Detection',
    'FramePolicy',
    'StepResult',
    'TrackEvent',
    'TrackSnapshot',
    'Tracker',
    'TrackerParams',
]
