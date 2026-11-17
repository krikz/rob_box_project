"""Гард: tests/unit гоняется в CI каталогом целиком, а не поштучно (ADR-0057).

Если кто-то заменит каталожный шаг `pytest tests/unit` на список файлов, новые каталоги (как tests/unit/sim)
выпадут из CI молча. Гард ловит это, а также потерю Pillow в офлайн-джобе (без неё test_sim_coverage_anim не идёт).
"""

import re
from pathlib import Path

WORKFLOW = Path(__file__).resolve().parents[2] / ".github" / "workflows" / "G-Lint Code.yml"


def _text() -> str:
    return WORKFLOW.read_text(encoding="utf-8")


def test_whole_tests_unit_dir_is_collected():
    assert re.search(r"python -m pytest tests/unit \\\r?\n", _text()), "нет шага `pytest tests/unit` (каталог целиком)"


def test_offline_job_installs_pillow_for_sim_anim():
    offline = _text().split("Самая долгая job")[1]
    assert re.search(r"pip install [^\n]*\bpillow\b", offline.split("tests/unit offline tests")[0])
