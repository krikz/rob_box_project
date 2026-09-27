"""test_arrangement_presets.py — ArrangementPresetStore (ADR-0132 PR-7).

ROS-free (:mod:`core.arrangement_presets` не тянет rclpy) — без стаба
модулей, в отличие от ``test_tools/test_music.py``.
"""

from __future__ import annotations

import json

import pytest

from rob_box_mcp_tools.core.arrangement_presets import (
    PRESET_KNOB_FIELDS,
    ArrangementPresetStore,
)


def _store(tmp_path, shipped=None):
    shipped_path = tmp_path / "shipped.json"
    shipped_path.write_text(json.dumps(shipped or {}), encoding="utf-8")
    return ArrangementPresetStore(
        shipped_path=shipped_path, learned_root=str(tmp_path / "learned")
    )


class TestGet:
    def test_unknown_key_is_none(self, tmp_path):
        store = _store(tmp_path)
        assert store.get("nope") is None

    def test_empty_key_is_none(self, tmp_path):
        store = _store(tmp_path)
        assert store.get("") is None

    def test_shipped_preset_is_returned(self, tmp_path):
        store = _store(
            tmp_path,
            {"starwars_4": {"title": "Imperial March", "knobs": {"drum_style": "march"}}},
        )
        preset = store.get("starwars_4")
        assert preset["title"] == "Imperial March"
        assert preset["knobs"] == {"drum_style": "march"}

    def test_unknown_knob_fields_are_dropped(self, tmp_path):
        """Пресет не может пронести произвольный параметр (только ручки)."""
        store = _store(
            tmp_path,
            {
                "x": {
                    "title": "X",
                    "knobs": {"drum_style": "march", "lead_notes": "0,2,4", "hack": 1},
                }
            },
        )
        preset = store.get("x")
        assert preset["knobs"] == {"drum_style": "march"}
        assert "lead_notes" not in preset["knobs"]
        assert "hack" not in preset["knobs"]


class TestSaveAndLearnedOverridesShipped:
    def test_save_then_get_round_trips(self, tmp_path):
        store = _store(tmp_path)
        saved = store.save(
            "fifth",
            title="Beethoven's Fifth",
            knobs={"bass_style": "root", "lead_synth": "brass"},
            note="звучит собранно",
            approved_by_user_quote="класс, сохрани",
        )
        assert saved["melody_key"] == "fifth"
        assert "created_at" in saved

        preset = store.get("fifth")
        assert preset["title"] == "Beethoven's Fifth"
        assert preset["knobs"]["bass_style"] == "root"
        assert preset["note"] == "звучит собранно"
        assert preset["approved_by_user_quote"] == "класс, сохрани"

    def test_learned_overrides_shipped_for_the_same_key(self, tmp_path):
        store = _store(
            tmp_path,
            {"fifth": {"title": "shipped", "knobs": {"bass_style": "off"}}},
        )
        store.save("fifth", title="learned", knobs={"bass_style": "root"})
        preset = store.get("fifth")
        assert preset["title"] == "learned"
        assert preset["knobs"]["bass_style"] == "root"

    def test_save_persists_across_new_store_instances(self, tmp_path):
        store = _store(tmp_path)
        store.save("fifth", title="Fifth", knobs={"bass_style": "root"})
        # Новый экземпляр читает тот же learned-файл (persistence на диске,
        # не только в памяти процесса).
        other = ArrangementPresetStore(
            shipped_path=tmp_path / "shipped.json", learned_root=str(tmp_path / "learned")
        )
        assert other.get("fifth")["title"] == "Fifth"

    def test_save_overwrites_previous_learned_entry_for_same_key(self, tmp_path):
        store = _store(tmp_path)
        store.save("fifth", title="v1", knobs={"bass_style": "root"})
        store.save("fifth", title="v2", knobs={"bass_style": "off"})
        preset = store.get("fifth")
        assert preset["title"] == "v2"
        assert preset["knobs"]["bass_style"] == "off"

    def test_save_drops_unknown_knob_fields(self, tmp_path):
        store = _store(tmp_path)
        store.save("fifth", title="Fifth", knobs={"bass_style": "root", "lead_notes": "0,2,4"})
        assert "lead_notes" not in store.get("fifth")["knobs"]


def test_preset_knob_fields_covers_synths_form_bpm_and_compose_knobs():
    """ADR-0132 PR-7 §1 — «только ручки compose_music + synths/form/bpm»."""
    for field in (
        "lead_synth", "bass_synth", "pad_synth", "counter_synth",
        "drum_style", "form", "bpm",
        "key_detection", "chords", "harmonic_rhythm", "density",
        "bass_style", "bass_approach", "pad_style", "pad_register",
        "counter", "theme_octaves", "lead_octave", "lead_outliers", "levels",
    ):
        assert field in PRESET_KNOB_FIELDS
    # Не музыкальный материал (ноты, RTTTL-имя) — не ручка, не должно
    # затесаться в набор допустимых полей пресета.
    for forbidden in ("lead_notes", "bass_notes", "pad_notes", "name", "variants"):
        assert forbidden not in PRESET_KNOB_FIELDS


def test_shipped_data_file_has_valid_imperial_march_entry():
    """Проверяет реальный бандловый файл, не тестовую фикстуру — это то,
    что реально грузит ``ArrangementPresetStore()`` без аргументов."""
    from importlib.resources import files

    text = files("rob_box_mcp_tools.data").joinpath("arrangement_presets.json").read_text(
        encoding="utf-8"
    )
    data = json.loads(text)
    assert "starwars_4" in data
    entry = data["starwars_4"]
    assert entry["title"]
    knobs = entry["knobs"]
    assert set(knobs) <= set(PRESET_KNOB_FIELDS)
    assert knobs["drum_style"] == "march"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
