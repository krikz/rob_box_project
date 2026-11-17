"""Русские названия звуков (``knowledge.SOUND_ALIASES_RU``, #3976) сверены с ``sound_pack/sound_catalog.json``.

Лежит здесь, а не в пакете ``rob_box_dialog/test``: в сборке пакета (G-Run Tests, ``test_ws``) корня репозитория и
каталога звуков нет, а здесь (Offline Unit Tests, шаг «Dialog metrics scripts self-tests») есть.
"""

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_dialog"))

from rob_box_dialog import knowledge as kn  # noqa: E402

CATALOG = REPO / "sound_pack" / "sound_catalog.json"


def test_sound_aliases_match_the_sound_catalog():
    """Чужого триггера в таблице нет; у каждого триггера каталога есть название или он назван в ``SOUND_UNNAMED``
    (новый звук в каталоге без строки в таблице - красный тест)."""
    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    triggers = {info["trigger"] for name, info in catalog["sounds"].items() if name.endswith(".mp3")}
    assert triggers, "каталог звуков пуст - тест ничего не проверяет"
    named = {t for ts, _ in kn.SOUND_ALIASES_RU for t in ts}
    unnamed = set(kn.SOUND_UNNAMED)
    assert named <= triggers, sorted(named - triggers)
    assert unnamed <= triggers, sorted(unnamed - triggers)
    assert not named & unnamed
    assert triggers - named - unnamed == set(), sorted(triggers - named - unnamed)


def test_sound_imitations_are_named_sounds_of_the_sound_catalog():
    """Звукоподражание (``knowledge.SOUND_IMITATIONS``, #4031) - только к звукам каталога, у которых есть разговорное
    название (человек просит их словами: «включи сирену»); звука реакции робота (``SOUND_UNNAMED``) в таблице нет."""
    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    triggers = {info["trigger"] for name, info in catalog["sounds"].items() if name.endswith(".mp3")}
    named = {t for ts, _ in kn.SOUND_ALIASES_RU for t in ts}
    imitated = set(kn.SOUND_IMITATIONS)
    assert imitated <= triggers, sorted(imitated - triggers)
    assert imitated <= named, sorted(imitated - named)
    assert not imitated & set(kn.SOUND_UNNAMED)
