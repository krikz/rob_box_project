"""ADR-0154 PR-1: ``ScoreMaterial`` — валидатор, JSON-круг и отказы с причиной."""

from __future__ import annotations

import json
from dataclasses import replace

import pytest

from rob_box_music import material as mt
from rob_box_music.model import Key, PitchEvent


def make_material(**over) -> mt.ScoreMaterial:
    """Синтетический материал (наш, не чужая партитура): 8 тактов 4/4, до мажор."""
    melody = tuple(PitchEvent(m, i * 1.0, 1.0, 3 if i % 4 == 0 else 2)
                   for i, m in enumerate([60, 62, 64, 65, 67, 65, 64, 62] * 4))
    base = mt.ScoreMaterial(
        material_id="local:ab12cd34", title="Синтетика", composer="test", source="synthetic", license="PD",
        meter=(4, 4), bpm=120, key=Key(0, "major"), melody=melody,
        chords=(mt.ChordSpan(0.0, 4.0, 0, "maj", 0), mt.ChordSpan(4.0, 4.0, 7, "maj", 4),
                mt.ChordSpan(8.0, 4.0, 1, "other", None)),
        bass=(PitchEvent(36, 0.0, 2.0, 3), PitchEvent(43, 4.0, 2.0, 3)),
        phrases=(mt.Phrase(0, 4, "new", 2), mt.Phrase(4, 4, "repeat", 2)),
        sections=(mt.ScoreSection("theme", 0, 4, "text"),),
        stats=mt.MaterialStats(rating=4.5, n_views=10, complexity=2, key_fit=0.97, unique_bar_share=0.5,
                               textures={"sustained": 0.5, "broken": 0.25}))
    return replace(base, **over)


def test_valid_material_passes():
    mt.validate_material(make_material())


def test_json_round_trip_is_lossless_and_deterministic():
    m = make_material()
    text = mt.to_json(m)
    back = mt.from_json(text)
    assert back == m
    assert mt.to_json(back) == text
    assert json.loads(text)["schema"] == mt.SCHEMA_VERSION


def test_json_round_trip_of_minimal_material():
    m = make_material(chords=(), bass=(), phrases=(), sections=(), stats=mt.MaterialStats(), bpm=None)
    assert mt.from_json(mt.to_json(m)) == m


@pytest.mark.parametrize("over, path, fragment", [
    ({"license": "unknown"}, "license", "не позволяет"),
    ({"license": ""}, "license", "не позволяет"),
    ({"license": "license_conflict"}, "license", "не позволяет"),
    ({"material_id": "xx:1"}, "material_id", "pdmx"),
    ({"material_id": "local:a b"}, "material_id", "pdmx"),
    ({"title": "  "}, "title", "пустое"),
    ({"meter": (3, 3)}, "meter", "степень двойки"),
    ({"meter": (0, 4)}, "meter", "числитель"),
    ({"bpm": 5}, "bpm", "вне"),
    ({"key": Key(12, "major")}, "key", "knowledge.SCALES"),
    ({"key": Key(0, "wonky")}, "key", "knowledge.SCALES"),
    ({"melody": ()}, "melody", "пустая"),
    ({"melody": (PitchEvent(60, 1.0, 1.0), PitchEvent(62, 1.0, 1.0))}, "melody[1].beat", "по возрастанию"),
    ({"melody": (PitchEvent(130, 0.0, 1.0),)}, "melody[0].midi", "вне 0..127"),
    ({"melody": (PitchEvent(60, 0.0, 0.0),)}, "melody[0]", "dur > 0"),
    ({"bass": (PitchEvent(40, 2.0, 1.0), PitchEvent(40, 1.0, 1.0))}, "bass[1].beat", "по возрастанию"),
    ({"chords": (mt.ChordSpan(0.0, 4.0, 12, "maj"),)}, "chords[0].root_pc", "вне 0..11"),
    ({"chords": (mt.ChordSpan(0.0, 4.0, 0, "weird"),)}, "chords[0].quality", "не из"),
    ({"chords": (mt.ChordSpan(0.0, 4.0, 0, "maj", 7),)}, "chords[0].degree", "0..6"),
    ({"chords": (mt.ChordSpan(0.0, 4.0, 0, "maj"), mt.ChordSpan(2.0, 4.0, 5, "min"))}, "chords[1]", "не по порядку"),
    ({"phrases": (mt.Phrase(0, 4, "new", 0),)}, "phrases[0].repeats", "≥ 1"),
    ({"phrases": (mt.Phrase(0, 4, "mirror", 1),)}, "phrases[0].relation", "не из"),
    ({"phrases": (mt.Phrase(4, 4, "new"), mt.Phrase(0, 4, "new"))}, "phrases[1].bar", "раньше"),
    ({"sections": (mt.ScoreSection("", 0, 4, "text"),)}, "sections[0].name", "пустое"),
    ({"sections": (mt.ScoreSection("a", 0, 4, "guess"),)}, "sections[0].origin", "не из"),
    ({"stats": mt.MaterialStats(key_fit=1.5)}, "stats.key_fit", "вне 0..1"),
    ({"stats": mt.MaterialStats(textures={"x": -0.1})}, "stats.textures", "вне 0..1"),
])
def test_validator_refuses_with_path_and_reason(over, path, fragment):
    with pytest.raises(mt.MaterialError) as err:
        mt.validate_material(make_material(**over))
    assert err.value.path == path
    assert fragment in err.value.reason


def test_to_json_refuses_invalid_material():
    with pytest.raises(mt.MaterialError):
        mt.to_json(make_material(license="unknown"))


def _dict():
    return json.loads(mt.to_json(make_material()))


@pytest.mark.parametrize("mutate, path", [
    (lambda d: d.update(schema=99), "schema"),
    (lambda d: d.pop("melody"), "melody"),
    (lambda d: d.update(melody=[[60, 0.0, 1.0]]), "melody[0]"),
    (lambda d: d.update(title=5), "title"),
    (lambda d: d["key"].pop("mode"), "key.mode"),
    (lambda d: d.update(chords=[["x", 4.0, 0, "maj", 0]]), ""),
    (lambda d: d.update(license="unknown"), "license"),
])
def test_from_dict_refuses_broken_input_with_path(mutate, path):
    data = _dict()
    mutate(data)
    with pytest.raises(mt.MaterialError) as err:
        mt.from_dict(data)
    assert err.value.path == path


def test_from_json_refuses_non_json_and_non_object():
    with pytest.raises(mt.MaterialError, match="не JSON"):
        mt.from_json("{oops")
    with pytest.raises(mt.MaterialError, match="ожидается объект"):
        mt.from_json("[1, 2]")


def test_bar_beats():
    assert mt.bar_beats((4, 4)) == 4.0
    assert mt.bar_beats((3, 4)) == 3.0
    assert mt.bar_beats((6, 8)) == 3.0
    assert mt.bar_beats((2, 2)) == 4.0
