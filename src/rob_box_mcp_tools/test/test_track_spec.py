"""Тесты ``core.track_spec`` (ADR-0142 §4-§5, issue #3136).

Снимок схемы — ``fixtures/track_spec.schema.json``. Осознанное изменение
палитры — перегенерировать и приложить diff к PR::

    python src/rob_box_mcp_tools/test/test_track_spec.py --regen
"""

from __future__ import annotations

import copy
import json
import random
import sys
from pathlib import Path

import pytest

from rob_box_mcp_tools.core.arrangement_matrix import SECTION_TEMPLATES
from rob_box_mcp_tools.core.arranger import VALID_ROOTS
from rob_box_mcp_tools.core.club_arranger import (
    CLUB_SYNTHS,
    HATS_PATTERNS,
    KICK_PATTERNS,
    LAYER_LEVELS,
    ROLE_SYNTHS,
    build_matrix,
    calibrated_gates,
    club_form_beats,
    render_club,
    render_club_kit,
)
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando
from rob_box_mcp_tools.core.track_spec import (
    PROGRESSION_NAMES,
    SpecAnchors,
    SpecError,
    TrackSpec,
    render_spec,
    seeded_spec,
    track_spec_schema,
    validate_spec,
)

SCHEMA_SNAPSHOT = Path(__file__).parent / "fixtures" / "track_spec.schema.json"


def _base() -> dict:
    return seeded_spec(0).to_dict()


def _assert_sanitizer_clean(code: str) -> None:
    result = sanitize_renando(code, 0.85, known_synths=CLUB_SYNTHS)
    assert result.security_error is None
    assert result.quality_errors == ()
    assert result.slot_error is None
    assert result.warnings == ()
    assert result.code == code


# --- регресс-якорь: seeded-путь через спеку звучит ровно как сегодня ------


@pytest.mark.parametrize("seed", list(range(25)) + [123, 4242, 99991])
def test_seeded_spec_renders_byte_identical_to_render_club(seed):
    assert render_spec(seeded_spec(seed=seed)) == render_club(seed=seed)


@pytest.mark.parametrize("template", sorted(SECTION_TEMPLATES))
@pytest.mark.parametrize("kick", sorted(KICK_PATTERNS))
def test_seeded_spec_honours_explicit_template_and_kick(template, kick):
    spec = seeded_spec(seed=7, bpm=128, root="D", template=template, kick=kick)
    assert render_spec(spec, repeat=True, align_clock=True) == render_club(
        bpm=128, root="D", template=template, kick=kick, seed=7, repeat=True, align_clock=True,
    )


def test_render_club_kit_defaults_equal_render_club():
    from rob_box_mcp_tools.core.club_arranger import club_kit

    for seed in (0, 1, 5):
        assert render_club_kit(club_kit(seed), seed=seed) == render_club(seed=seed)


# --- валидная спека → код ---------------------------------------------------


def test_valid_spec_roundtrip_and_render():
    raw = _base()
    raw.update(bpm=126, root="F", progression="i-iv-VI-v", notes="тест")
    raw["timbre"] = {"lead": "marimba", "bass": "dub", "pad": "space"}
    raw["groove"] = {"kick": "four_on_floor", "hats": "offbeat"}
    raw["levels"] = {"lead": 0.5}
    spec = validate_spec(raw)
    assert validate_spec(spec.to_dict()) == spec
    code = render_spec(spec)
    assert "p1 >> marimba(" in code
    assert "p2 >> dub(" in code
    assert "p3 >> space(" in code
    assert "Clock.bpm = 126" in code
    assert "i-iv-VI-v" in code.splitlines()[0]
    assert 'd1 >> play("X...X...X...X..."' in code
    assert f'd2 >> play("{HATS_PATTERNS["offbeat"]}"' in code
    _assert_sanitizer_clean(code)


def test_level_multiplier_scales_gate_only_for_that_lane():
    raw = _base()
    raw["levels"] = {"lead": 0.5}
    base = render_spec(validate_spec(_base()))
    quiet = render_spec(validate_spec(raw))
    # Issue #3154: ``levels`` умножает откалиброванный гейт слоя (по блокам).
    spec = validate_spec(raw)
    matrix = build_matrix(spec.kit()["template"])
    lead_gate = calibrated_gates(matrix, spec.kit(), {"lead": 0.5})["lead"]
    assert f"amp={lead_gate}," in quiet
    changed = [(a, b) for a, b in zip(base.splitlines(), quiet.splitlines()) if a != b]
    assert len(changed) == 1 and changed[0][1].strip().startswith("amp=")


def test_explicit_progression_keeps_the_same_riff_rhythm():
    """Смена прогрессии не сдвигает поток сида: риф в индексах пула тот же."""
    a = _base()
    b = _base()
    b["progression"] = next(n for n in PROGRESSION_NAMES if n != a["progression"])
    code_a, code_b = render_spec(validate_spec(a)), render_spec(validate_spec(b))
    assert code_a != code_b
    assert code_a.count("\n") == code_b.count("\n")


def _random_spec(rng: random.Random) -> dict:
    raw = {
        "version": 1, "style": "club",
        "bpm": rng.choice([rng.randint(60, 180), round(rng.uniform(60, 180), 2)]),
        "root": rng.choice(VALID_ROOTS), "scale": "minor", "seed": rng.randint(0, 10**6),
        "form": {"template": rng.choice(sorted(SECTION_TEMPLATES))},
        "progression": rng.choice(PROGRESSION_NAMES),
        "groove": {"kick": rng.choice(sorted(KICK_PATTERNS)), "hats": rng.choice(sorted(HATS_PATTERNS))},
        "timbre": {role: rng.choice(synths) for role, synths in ROLE_SYNTHS.items()},
    }
    if rng.random() < 0.5:
        raw["levels"] = {
            lane: rng.choice([0, 1, round(rng.random(), 3)])
            for lane in rng.sample(sorted(LAYER_LEVELS), rng.randint(1, len(LAYER_LEVELS)))
        }
    return raw


def test_1000_random_valid_specs_render_and_pass_sanitizer_unchanged():
    rng = random.Random(3136)
    for _ in range(1000):
        spec = validate_spec(_random_spec(rng))
        code = render_spec(spec, repeat=rng.random() < 0.5, align_clock=rng.random() < 0.5)
        _assert_sanitizer_clean(code)
        assert render_spec(spec, repeat=True) == render_spec(spec, repeat=True)


# --- невалидная спека → отказ с путём и понятной причиной -----------------


def _mutate(path, value):
    raw = _base()
    target = raw
    keys = path.split(".")
    for key in keys[:-1]:
        target = target.setdefault(key, {})
    target[keys[-1]] = value
    return raw


INVALID_CASES = [
    ("levels.lead", 1.3, "levels.lead", "вне диапазона"),
    ("levels.lead", -0.1, "levels.lead", "вне диапазона"),
    ("levels.lead", True, "levels.lead", "ожидалось число"),
    ("levels.kick", "0.5", "levels.kick", "ожидалось число"),
    ("levels.drums", 0.5, "levels.drums", "неизвестное поле"),
    ("timbre.lead", "supersaw", "timbre.lead", "не из допустимых"),
    ("timbre.lead", "bass", "timbre.lead", "не из допустимых"),  # синт чужой роли
    ("timbre.pad", "", "timbre.pad", "не из допустимых"),
    ("bpm", 200, "bpm", "вне диапазона"),
    ("bpm", 59.9, "bpm", "вне диапазона"),
    ("bpm", "124", "bpm", "ожидалось число"),
    ("root", "H", "root", "не из допустимых"),
    ("scale", "major", "scale", "не из допустимых"),
    ("style", "classic", "style", "не из допустимых"),
    ("version", 2, "version", "поддержана только версия 1"),
    ("seed", 1.5, "seed", "ожидалось целое"),
    ("seed", -1, "seed", "вне диапазона"),
    ("form.template", "my_matrix", "form.template", "не из допустимых"),
    ("form.blocks", 16, "form.blocks", "неизвестное поле"),
    ("progression", "I-V-vi-IV", "progression", "не из допустимых"),
    ("groove.kick", "trap", "groove.kick", "не из допустимых"),
    ("groove.bass", {"rhythm": "x"}, "groove.bass", "неизвестное поле"),
    ("melody", {"library_id": "supermar"}, "melody", "неизвестное поле"),
    ("code", "d1 >> play('x')", "code", "неизвестное поле"),
    ("notes", 5, "notes", "ожидалась строка"),
    ("notes", "x" * 2001, "notes", "длиннее"),
]


@pytest.mark.parametrize("path, value, err_path, reason", INVALID_CASES, ids=lambda v: str(v)[:24])
def test_invalid_spec_rejected_with_path_and_reason(path, value, err_path, reason):
    with pytest.raises(SpecError) as exc:
        validate_spec(_mutate(path, value))
    assert exc.value.path == err_path
    assert reason in exc.value.reason


@pytest.mark.parametrize("missing", ["bpm", "timbre", "groove", "seed", "progression"])
def test_missing_required_field(missing):
    raw = _base()
    del raw[missing]
    with pytest.raises(SpecError) as exc:
        validate_spec(raw)
    assert exc.value.path == missing
    assert "обязательное" in exc.value.reason


def test_missing_nested_field():
    raw = _base()
    del raw["timbre"]["bass"]
    with pytest.raises(SpecError) as exc:
        validate_spec(raw)
    assert exc.value.path == "timbre.bass"


@pytest.mark.parametrize("raw", [None, [], "spec", 42])
def test_non_object_rejected(raw):
    with pytest.raises(SpecError) as exc:
        validate_spec(raw)
    assert exc.value.path == "$"


def test_spec_error_is_value_error_with_readable_message():
    with pytest.raises(ValueError, match=r"^levels\.lead: 1\.3 вне диапазона 0\.\.1$"):
        validate_spec(_mutate("levels.lead", 1.3))


def test_validator_does_not_mutate_input():
    raw = _mutate("levels.lead", 0.5)
    before = copy.deepcopy(raw)
    validate_spec(raw)
    assert raw == before


# --- якоря ------------------------------------------------------------------


def test_anchors_accept_matching_spec():
    spec = seeded_spec(0)
    anchors = SpecAnchors(bpm=124, root="A#", scale="minor", form_beats=club_form_beats(spec.template),
                          pad=spec.pad, hats=spec.hats)
    assert validate_spec(spec.to_dict(), anchors) == spec


@pytest.mark.parametrize("anchors, path", [
    (SpecAnchors(bpm=132), "bpm"),
    (SpecAnchors(root="C"), "root"),
    (SpecAnchors(pad="space"), "timbre.pad"),
    (SpecAnchors(hats="shuffle"), "groove.hats"),
    (SpecAnchors(form_beats=224), "form.template"),
])
def test_anchor_violation_is_spec_error(anchors, path):
    with pytest.raises(SpecError) as exc:
        validate_spec(_base(), anchors)
    assert exc.value.path == path
    assert "якорь" in exc.value.reason or "якорю" in exc.value.reason


# --- рендер -----------------------------------------------------------------


def test_render_reveal_entry_is_not_yet_supported():
    with pytest.raises(SpecError) as exc:
        render_spec(seeded_spec(0), entry="reveal")
    assert exc.value.path == "entry"


def test_render_club_kit_rejects_foreign_synth_even_without_validator():
    kit = seeded_spec(0).kit()
    kit["lead"] = "supersaw"
    with pytest.raises(ValueError, match="палитры роли lead"):
        render_club_kit(kit)


def test_track_spec_is_frozen():
    spec = seeded_spec(0)
    with pytest.raises(Exception):
        spec.bpm = 130  # type: ignore[misc]
    assert isinstance(spec, TrackSpec)


# --- схема: генерируется из констант, снимок совпадает ---------------------


def test_schema_snapshot_matches_generation():
    assert json.loads(SCHEMA_SNAPSHOT.read_text(encoding="utf-8")) == track_spec_schema()


def test_schema_enums_come_from_code_palette():
    props = track_spec_schema()["properties"]
    assert props["timbre"]["properties"]["lead"]["enum"] == list(ROLE_SYNTHS["lead"])
    assert props["groove"]["properties"]["kick"]["enum"] == list(KICK_PATTERNS)
    assert props["form"]["properties"]["template"]["enum"] == list(SECTION_TEMPLATES)
    assert set(props["levels"]["properties"]) == set(LAYER_LEVELS)
    assert "melody" not in props and "code" not in props


def test_schema_agrees_with_validator_when_jsonschema_available():
    """Перекрёстная сверка (jsonschema в зависимостях пакета НЕТ — только если стоит)."""
    jsonschema = pytest.importorskip("jsonschema")
    schema = track_spec_schema()
    rng = random.Random(42)
    for _ in range(50):
        jsonschema.validate(_random_spec(rng), schema)
    for path, value, _p, _r in INVALID_CASES:
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(_mutate(path, value), schema)


if __name__ == "__main__" and "--regen" in sys.argv:
    SCHEMA_SNAPSHOT.write_text(
        json.dumps(track_spec_schema(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8",
    )
    print(f"regenerated {SCHEMA_SNAPSHOT}")
