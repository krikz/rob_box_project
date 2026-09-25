#!/usr/bin/env python3
"""Tests for ``rob_box_telegram.face_card`` (issue #3025).

The Telegram bot reads the face store directly from disk via this
module — there's no FaceStore / ROS coupling in tests. We exercise
three things:

* the read-only directory listing (``list_people``) and the summary
  formatter (``format_faces_table``);
* name / short-id resolution (``find_summary``) and detail loading
  (``load_detail``) against a tmpdir tree shaped like the real
  ``/data/faces/<person_id>/`` layout;
* the PIL collage builder (``build_face_collage``) with tiny in-memory
  JPEGs (PIL is a soft requirement — the tests skip the collage if
  PIL is missing).

All filesystem operations happen under ``tmp_path`` — nothing touches
the real ``/data/faces`` or the live robot.

Tests use plain ``assert`` statements so they work under both
``pytest`` and ``unittest`` runners; ``tmp_path`` is the standard
pytest fixture (built-in, no extra deps).
"""

from __future__ import annotations

import io
import json
import sys
from pathlib import Path


# Make ``rob_box_telegram`` importable regardless of where pytest runs.
THIS_DIR = Path(__file__).resolve().parent
SRC = THIS_DIR.parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rob_box_telegram.face_card import (  # noqa: E402  (import after sys.path tweak)
    FaceSummary,
    build_face_collage,
    find_summary,
    format_face_summary,
    format_faces_table,
    list_people,
    load_detail,
)


def _write_minimal_person(
    root: Path,
    person_id: str,
    *,
    name: str | None = None,
    speaker_id: str | None = None,
    encounter_count: int = 0,
    last_encounter_ts: float | None = None,
    mode_recorded: str | None = None,
    with_reference: bool = True,
    with_encounters: list[str] | None = None,
    embeddings_bytes: bytes | None = None,
) -> Path:
    """Create one record directory with an optional reference.jpg and
    synthetic encounter files. Returns the person_dir.

    The fake ``encounters/*.jpg`` files are filled with garbage bytes —
    the readers in face_card treat them as opaque blobs (they only
    report sizes when PIL is present, else 0x0), so we don't need real
    JPEGs for ``list_people`` / ``load_detail`` (no PIL decode path).
    """
    person_dir = root / person_id
    person_dir.mkdir(parents=True, exist_ok=True)

    meta: dict = {"person_id": person_id}
    if name is not None:
        meta["name"] = name
    if speaker_id is not None:
        meta["speaker_id"] = speaker_id
    if encounter_count:
        meta["encounter_count"] = encounter_count
    if last_encounter_ts is not None:
        meta["last_encounter_ts"] = last_encounter_ts
    if mode_recorded is not None:
        meta["mode_recorded"] = mode_recorded

    (person_dir / "meta.json").write_text(
        json.dumps(meta, ensure_ascii=False), encoding="utf-8"
    )

    if with_reference:
        # 16x16 white JPEG so PIL can decode it in tests that use the
        # collage. ``list_people`` doesn't care about content.
        try:
            from PIL import Image

            ref = Image.new("RGB", (16, 16), color=(255, 255, 255))
            ref.save(person_dir / "reference.jpg", format="JPEG")
        except Exception:
            # No PIL available — leave a placeholder so the path exists.
            (person_dir / "reference.jpg").write_bytes(b"\xff\xd8\xff\xd9")

    if with_encounters:
        enc_dir = person_dir / "encounters"
        enc_dir.mkdir(exist_ok=True)
        for fname in with_encounters:
            (enc_dir / fname).write_bytes(b"\xff\xd8\xff\xd9")

    if embeddings_bytes is not None:
        (person_dir / "embeddings.npy").write_bytes(embeddings_bytes)

    return person_dir


# ─── list_people ──────────────────────────────────────────────────────────


def test_list_people_missing_root_returns_empty():
    assert list_people("/no/such/path/__test__") == []


def test_list_people_empty_root_returns_empty(tmp_path: Path):
    assert list_people(str(tmp_path)) == []


def test_list_people_named_first_then_strangers(tmp_path: Path):
    _write_minimal_person(tmp_path, "aaa11111", name="Алиса")
    _write_minimal_person(tmp_path, "bbb22222", name=None)  # stranger
    _write_minimal_person(tmp_path, "ccc33333", name="Борис")

    rows = list_people(str(tmp_path))
    # 3 rows, no dups
    assert len(rows) == 3
    # Named (Алиса, Борис) first — sorted by person_id within group;
    # strangers last
    names = [r.name for r in rows]
    assert names == ["Алиса", "Борис", None]

    for row in rows:
        assert isinstance(row, FaceSummary)
        assert len(row.person_id_short) == 8


def test_list_people_broken_meta_does_not_crash(tmp_path: Path):
    # meta.json with garbage bytes — must not raise
    (tmp_path / "zzz99999").mkdir()
    (tmp_path / "zzz99999" / "meta.json").write_bytes(b"{not json")
    rows = list_people(str(tmp_path))
    assert len(rows) == 1
    # Broken record shows up as stranger with zero counts
    assert rows[0].name is None
    assert rows[0].is_stranger is True
    assert rows[0].encounter_count == 0


# ─── format_faces_table ───────────────────────────────────────────────────


def test_format_faces_table_empty_message():
    out = format_faces_table([])
    assert "Лицевая база пуста" in out


def test_format_faces_table_contains_required_columns(tmp_path: Path):
    _write_minimal_person(
        tmp_path,
        "4ff0ddc5",
        name="Дэнчик",
        speaker_id="1ae4b0ac",
        encounter_count=27,
    )
    rows = list_people(str(tmp_path))
    text = format_faces_table(rows)
    assert "Дэнчик" in text
    assert "4ff0ddc5" in text
    assert "1ae4b0ac" in text
    assert "27" in text
    assert "👥" in text


# ─── find_summary ─────────────────────────────────────────────────────────


def test_find_summary_resolve_by_short_id(tmp_path: Path):
    _write_minimal_person(tmp_path, "4ff0ddc5abcdef", name="Дэнчик")
    _write_minimal_person(tmp_path, "ababcdcd0000", name="Борис")
    summary = find_summary(str(tmp_path), "4ff0ddc5")
    assert summary is not None
    assert summary.name == "Дэнчик"


def test_find_summary_resolve_by_full_id(tmp_path: Path):
    _write_minimal_person(tmp_path, "4ff0ddc5abcdef", name="Дэнчик")
    summary = find_summary(str(tmp_path), "4ff0ddc5abcdef")
    assert summary is not None
    assert summary.person_id_short == "4ff0ddc5"


def test_find_summary_resolve_by_name_case_insensitive(tmp_path: Path):
    _write_minimal_person(tmp_path, "4ff0ddc5abcdef", name="Дэнчик")
    summary = find_summary(str(tmp_path), "дэнчик")
    assert summary is not None
    assert summary.person_id == "4ff0ddc5abcdef"


def test_find_summary_unknown_returns_none(tmp_path: Path):
    _write_minimal_person(tmp_path, "aaa11111", name="Алиса")
    assert find_summary(str(tmp_path), "missing") is None


def test_find_summary_empty_query_returns_none(tmp_path: Path):
    _write_minimal_person(tmp_path, "aaa11111", name="Алиса")
    assert find_summary(str(tmp_path), "") is None


# ─── load_detail ──────────────────────────────────────────────────────────


def test_load_detail_missing_dir_returns_none(tmp_path: Path):
    assert load_detail(str(tmp_path), "nope") is None


def test_load_detail_dir_without_meta_returns_none(tmp_path: Path):
    (tmp_path / "abc12345").mkdir()
    assert load_detail(str(tmp_path), "abc12345") is None


def test_load_detail_full_record_loads(tmp_path: Path):
    _write_minimal_person(
        tmp_path,
        "4ff0ddc5abcdef",
        name="Дэнчик",
        speaker_id="1ae4b0ac",
        encounter_count=11,
        last_encounter_ts=1_700_000_000.0,
        mode_recorded="operator",
        with_reference=True,
        with_encounters=["000010_face.jpg", "000011_face.jpg", "m-000007.jpg"],
    )
    detail = load_detail(str(tmp_path), "4ff0ddc5abcdef")
    assert detail is not None
    assert detail.summary.name == "Дэнчик"
    assert detail.summary.speaker_id == "1ae4b0ac"
    assert detail.summary.encounter_count == 11
    assert detail.last_encounter_ts == 1_700_000_000.0
    assert detail.mode_recorded == "operator"
    # Reference first, then 3 encounters = 4 tiles
    assert len(detail.tiles) == 4
    assert detail.tiles[0].label == "reference"
    assert detail.tiles[0].raw_filename == "reference.jpg"
    # Encounter labels strip "_face" suffix
    assert detail.tiles[1].label == "000010"
    assert detail.tiles[2].label == "000011"
    # Merged files keep the m- prefix
    assert detail.tiles[3].label == "m-000007"
    assert detail.tiles[3].raw_filename == "m-000007.jpg"


def test_load_detail_missing_reference_only_encounters(tmp_path: Path):
    _write_minimal_person(
        tmp_path,
        "abc12345",
        name="Алиса",
        with_reference=False,
        with_encounters=["000001_face.jpg"],
    )
    detail = load_detail(str(tmp_path), "abc12345")
    assert detail is not None
    assert len(detail.tiles) == 1
    assert detail.tiles[0].label == "000001"


# ─── format_face_summary ──────────────────────────────────────────────────


def test_format_face_summary_contains_required_fields(tmp_path: Path):
    _write_minimal_person(
        tmp_path,
        "4ff0ddc5",
        name="Дэнчик",
        speaker_id="1ae4b0ac",
        encounter_count=27,
        last_encounter_ts=1_700_000_000.0,
        mode_recorded="operator",
    )
    detail = load_detail(str(tmp_path), "4ff0ddc5")
    assert detail is not None
    text = format_face_summary(detail)
    # Required fields per spec
    assert "Дэнчик" in text
    assert "4ff0ddc5" in text
    assert "1ae4b0ac" in text
    assert "27" in text
    # ADR-0106 note: explicit "последняя встреча по лицу" label
    assert "последняя встреча по лицу" in text
    assert "operator" in text


# ─── build_face_collage ───────────────────────────────────────────────────


def test_build_face_collage_no_tiles_returns_none(tmp_path: Path):
    _write_minimal_person(tmp_path, "abc12345", name="Алиса", with_reference=False)
    detail = load_detail(str(tmp_path), "abc12345")
    assert detail is not None
    # No reference + no encounters + no merges → empty collage
    assert build_face_collage(detail) is None


def test_build_face_collage_builds_jpeg_bytes_when_pil_present(tmp_path: Path):
    _write_minimal_person(
        tmp_path,
        "abc12345",
        name="Алиса",
        with_reference=True,
        with_encounters=["000001_face.jpg"],
    )
    detail = load_detail(str(tmp_path), "abc12345")
    assert detail is not None

    try:
        from PIL import Image  # noqa: F401
    except Exception:
        import pytest

        pytest.skip("PIL not installed — collage builder unavailable")

    out = build_face_collage(detail)
    assert out is not None
    # JPEG magic bytes
    assert out[:2] == b"\xff\xd8"
    # Re-decoding it round-trips
    recon = Image.open(io.BytesIO(out))
    recon.load()
    assert recon.format == "JPEG"
    # 4 cols × 3 rows grid → canvas width is roughly 4*(240+12)+pad
    assert recon.width >= 4 * 240
    assert recon.height >= 3 * 320