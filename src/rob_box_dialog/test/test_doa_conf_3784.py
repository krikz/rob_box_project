"""#3784: фраза несёт doa_deg + doa_conf; ниже порога угол не применяется и это видно.

Красное до фикса: ``DoaTrack.during`` возвращает только угол или ``None`` и глотает R,
``Heard`` поля уверенности не имеет, потребитель не отличает «угла не было» от «угол недостоверен».
"""

import math

import pytest

from rob_box_dialog.backlog import Backlog, Heard, same_direction
from rob_box_dialog.doa import DoaTrack
from rob_box_dialog.knowledge import DOA_CONFIDENCE_MIN
from rob_box_dialog.speaker_adapter import same_interlocutor


def _spread(half: float, n: int = 8) -> list:
    """n отсчётов в ±half вокруг 90°: чем шире, тем меньше R."""
    if n == 1:
        return [90.0]
    return [90.0 - half + 2 * half * i / (n - 1) for i in range(n)]


def _track(samples) -> DoaTrack:
    track = DoaTrack()
    track.on_vad(True, 0.0)
    for i, deg in enumerate(samples):
        track.on_direction(deg, 1.0 + i * 0.1)
    return track


def test_phrase_carries_confidence_and_names_the_unreliable_case():
    """Согласованные отсчёты: угол и R наружу. Разброс ~180°: угол есть, но недостоверен — не тишина."""
    agreed = _track(_spread(5.0)).during(0.0, 3.0)
    assert agreed.doa_deg == pytest.approx(90.0, abs=0.1)
    assert agreed.doa_conf == pytest.approx(1.0, abs=0.02)
    assert agreed.doa_conf >= DOA_CONFIDENCE_MIN and agreed.reliable

    unreliable = None
    for half in (40.0, 60.0, 80.0, 100.0):
        reading = _track(_spread(half)).during(0.0, 3.0)
        if not reading.reliable and reading.doa_deg is not None:
            unreliable = reading
            break
    assert unreliable is not None, "нужен разброс ниже порога, но с посчитанным углом"
    assert unreliable.doa_conf < DOA_CONFIDENCE_MIN and unreliable.usable_deg is None

    empty = _track([]).during(0.0, 3.0)
    assert empty.doa_deg is None and empty.doa_conf == 0.0 and empty.reliable is False


def test_consumer_applies_angle_only_above_the_knowledge_threshold():
    """Один порог — ``knowledge.DOA_CONFIDENCE_MIN``. Ниже него потребитель не считает направления одним."""
    wide = None
    for half in (20.0, 40.0, 60.0, 80.0, 100.0):
        reading = _track(_spread(half)).during(0.0, 3.0)
        if reading.doa_conf < DOA_CONFIDENCE_MIN:
            wide = reading
            break
    assert wide is not None and wide.doa_deg is not None, "нужен разброс, который порог не проходит"
    assert wide.reliable is False

    unreliable = Heard(text="робот", t=10.0, doa_deg=wide.usable_deg, doa_conf=wide.doa_conf, faces=frozenset({"f1"}))
    earlier = Heard(text="покажи", t=5.0, doa_deg=90.0, doa_conf=0.95, faces=frozenset({"f1"}))
    assert unreliable.doa_deg is None and unreliable.doa_conf == pytest.approx(wide.doa_conf)
    assert same_direction(unreliable.doa_deg, earlier.doa_deg) is False
    assert "doa" not in same_interlocutor(earlier, unreliable)

    label = Backlog.label(unreliable, earlier)
    assert label["doa_conf"] == round(wide.doa_conf, 2)
    assert label["doa_unreliable"] is True, "угол недостоверен — это видно в метке, не молчание"
    assert label["same_direction"] is False and label["doa_deg"] is None


def test_consumer_applies_a_confident_angle():
    reading = _track(_spread(5.0)).during(0.0, 3.0)
    assert reading.reliable and reading.usable_deg == pytest.approx(90.0, abs=0.1)
    caller = Heard(text="робот", t=10.0, doa_deg=reading.usable_deg, doa_conf=reading.doa_conf, faces=frozenset({"f1"}))
    earlier = Heard(text="покажи", t=5.0, doa_deg=85.0, doa_conf=0.9, faces=frozenset({"f1"}))
    assert same_direction(caller.doa_deg, earlier.doa_deg) is True
    assert "doa" in same_interlocutor(earlier, caller)
    label = Backlog.label(earlier, caller)
    assert label["doa_unreliable"] is False and label["same_direction"] is True
    assert label["doa_conf"] == pytest.approx(0.9)
    assert label["doa_deg"] == 85


def test_threshold_lives_only_in_knowledge():
    """Порог не зашит вторым числом: чтение ниже него недостоверно ровно по ``DOA_CONFIDENCE_MIN``."""
    reading = _track([0.0, 180.0]).during(0.0, 3.0)
    assert reading.doa_conf < DOA_CONFIDENCE_MIN
    assert reading.reliable is (reading.doa_conf >= DOA_CONFIDENCE_MIN)
    assert not math.isnan(reading.doa_conf)
