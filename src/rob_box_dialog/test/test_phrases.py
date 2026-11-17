"""Каталог фраз: покрытие всех (класс, статус), особые ключи, 1–3 варианта, слоты (ADR-0150 §5.4)."""

import string

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases


def _slots(template):
    return {name for _, name, _, _ in string.Formatter().parse(template) if name}


def test_every_class_status_has_a_phrase():
    keys = [key for cls in kn.ACTION_CLASSES.values() for key in cls.phrases.values()]
    missing = [key for key in keys if key not in phrases.catalog()]
    assert missing == []


def test_tool_and_special_keys_exist():
    tool_keys = [k for keys in kn.TOOL_PHRASES.values() for entry in keys.values()
                 for k in (entry.all_keys() if isinstance(entry, kn.ByData) else (entry,))]
    ask_keys = [rule.ask for rule in kn.COMPOSITION.values() if rule.ask]
    assert [k for k in (*tool_keys, *ask_keys, *kn.PHRASE_KEYS) if k not in phrases.catalog()] == []


def test_variants_and_slots_are_well_formed():
    for key, variants in phrases.catalog().items():
        assert 1 <= len(variants) <= phrases.MAX_VARIANTS, key
        assert len({frozenset(_slots(v)) for v in variants}) == 1, f"{key}: у вариантов разные слоты"
        for variant in variants:
            assert all(name.isidentifier() for name in _slots(variant)), key


def test_text_fills_slots_and_picks_variant_by_modulo():
    assert phrases.text("memory.saved", fact="Саша любит зелёный чай") == "Запомнил: Саша любит зелёный чай."
    assert phrases.text("understand.timeout", 7) == phrases.text("understand.timeout", 0)
    with pytest.raises(KeyError):
        phrases.text("memory.saved")
    with pytest.raises(KeyError):
        phrases.text("no.such.key")


def test_reason_falls_back_to_the_class_key():
    assert phrases.first_key("motion.rejected.no_motors", "motion.rejected") == "motion.rejected.no_motors"
    assert phrases.first_key("motion.rejected.cosmic_rays", "motion.rejected") == "motion.rejected"
    assert phrases.first_key(None, "nope") is None


def test_one_text_one_place():
    """Одинаковый текст у двух ключей — якорь YAML, а значит один объект, а не копия строки."""
    catalog = phrases.catalog()
    assert catalog["media.done"] == catalog["music.request_ok"]
    assert catalog["media.timeout"] == catalog["music.not_started"]
