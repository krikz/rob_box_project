"""issue #2747 — тёзка: имя совпало, голос не дотянул до порога слияния.

Зеркало ADR-0127. Там разбирался случай «голос похож, имя другое» — тогда
сливать нельзя, потому что в мастерской звучат похоже разные люди (Саша и
Борис дали resemblyzer cos=0.846). Здесь ровно наоборот: человек назвался
именем, которое в базе уже есть, но биометрия до ``REGISTER_MATCH_THRESHOLD``
не дотянула.

Молча нельзя ни то, ни другое:

* слить по имени — второй Саша унаследует факты первого; эпитеты
  (внутренние клички робота) заведены именно потому, что тёзки ожидаемы;
* завести новый профиль молча — то, что делалось раньше. Наблюдалось
  живьём 22.09.2026: человека перестали узнавать (медиана косинуса 0.502
  при пороге 0.72), он представился заново, и пара профилей «Дэнчик»,
  слитая вручную двумя часами ранее, восстановилась за пятнадцать минут.

Поэтому профиль создаётся (данные целы — инвариант ADR-0127), но на
исход вешается повод переспросить, и решает человек.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

# Грузим модуль по пути, минуя пакет `rob_box_voice`, чей `__init__` тянет
# pyaudio — на dev-машине и в CI его нет (тот же приём, что в соседних
# тестах этого каталога).
_PKG_ROOT = Path(__file__).resolve().parents[3] / "rob_box_voice"
_SPEAKER_EMB = _PKG_ROOT / "utils" / "speaker_embeddings.py"

_spec = importlib.util.spec_from_file_location(
    "rob_box_voice.utils.speaker_embeddings", _SPEAKER_EMB
)
_se = importlib.util.module_from_spec(_spec)
sys.modules["rob_box_voice.utils.speaker_embeddings"] = _se
_spec.loader.exec_module(_se)

SpeakerDatabase = _se.SpeakerDatabase
REGISTER_MATCH_THRESHOLD = _se.REGISTER_MATCH_THRESHOLD

DIM = 256


def _unit(seed: int) -> np.ndarray:
    rng = np.random.RandomState(seed)
    v = rng.normal(size=DIM).astype(np.float32)
    return v / np.linalg.norm(v)


def _near(base: np.ndarray, cos: float) -> np.ndarray:
    """Вектор с ЗАДАННЫМ косинусом к ``base`` — чтобы пороги проверялись
    точным числом, а не «примерно похожим» случайным вектором."""
    ortho = _unit(999)
    ortho = ortho - base * float(np.dot(ortho, base))
    ortho /= np.linalg.norm(ortho)
    v = base * cos + ortho * float(np.sqrt(max(0.0, 1.0 - cos * cos)))
    return (v / np.linalg.norm(v)).astype(np.float32)


@pytest.fixture()
def db(tmp_path) -> SpeakerDatabase:
    return SpeakerDatabase(db_path=str(tmp_path / "speakers.db"))


def test_same_name_low_voice_creates_profile_and_flags_twin(db) -> None:
    """Главный случай: профиль заведён, но с поводом переспросить."""
    base = _unit(1)
    first = db.register("Дэнчик", base)

    # Голос заметно ниже порога слияния — биометрия says «незнакомец».
    outcome = db.register_or_merge("Дэнчик", _near(base, 0.50))

    assert outcome.speaker_id != first, "профиль отдельный — данные целы"
    assert outcome.reused is False
    assert outcome.name_twin is True, "повод переспросить обязан быть"
    assert outcome.twin_speaker_id == first
    assert outcome.twin_name == "Дэнчик"
    assert outcome.twin_score == pytest.approx(0.50, abs=0.02), (
        "в поводе стоит ЧИСЛО — оператор по логу должен видеть, "
        "насколько близко было решение"
    )


def test_same_name_high_voice_reuses_profile_without_question(db) -> None:
    """Голос дотянул — это обычное слияние, переспрашивать не о чем."""
    base = _unit(2)
    first = db.register("Дэнчик", base)

    outcome = db.register_or_merge("Дэнчик", _near(base, 0.95))

    assert outcome.speaker_id == first
    assert outcome.reused is True
    assert outcome.name_twin is False


def test_new_name_does_not_flag_twin(db) -> None:
    """Незнакомое имя — обычная регистрация, без вопросов."""
    db.register("Дэнчик", _unit(3))

    outcome = db.register_or_merge("Борис", _unit(4))

    assert outcome.reused is False
    assert outcome.name_twin is False
    assert outcome.twin_speaker_id is None


def test_twin_match_is_case_insensitive(db) -> None:
    """«дэнчик» и «Дэнчик» — один и тот же тёзка.

    Сравнение имён идёт в Python, а не в SQL: ``LOWER()`` в SQLite умеет
    только ASCII и кириллицу не свёл бы, то есть повод переспросить
    молча не сработал бы ровно на русских именах.
    """
    base = _unit(5)
    first = db.register("Дэнчик", base)

    outcome = db.register_or_merge("дэнчик", _near(base, 0.40))

    assert outcome.name_twin is True
    assert outcome.twin_speaker_id == first


def test_voice_conflict_takes_precedence_over_twin(db) -> None:
    """Если голос совпал с ДРУГИМ именем, это случай ADR-0127.

    Он строже: там уже известно, что голос чей-то, и путать личности
    опаснее. Проверяем, что ветка тёзки его не перехватывает.
    """
    base = _unit(6)
    boris = db.register("Борис", base)

    outcome = db.register_or_merge("Саша", _near(base, 0.95))

    assert outcome.name_conflict is True
    assert outcome.conflict_speaker_id == boris
    assert outcome.name_twin is False, "это конфликт имени, а не тёзка"


def test_twin_picks_most_recent_namesake(db) -> None:
    """Если тёзок уже несколько — переспрашиваем про последнего.

    Логично: с ним разговаривали недавно, и именно его профиль человек
    скорее всего и имеет в виду.
    """
    db.register("Дэнчик", _unit(7))
    recent = db.register("Дэнчик", _unit(8))

    outcome = db.register_or_merge("Дэнчик", _unit(9))

    assert outcome.name_twin is True
    assert outcome.twin_speaker_id == recent
