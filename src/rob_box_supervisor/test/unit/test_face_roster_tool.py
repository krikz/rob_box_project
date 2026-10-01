"""Тесты инструмента ТАРС ``list_faces`` (issue #3299, часть A).

Фикстура — настоящий каталог лиц в ``tmp_path`` в том же формате, что пишет
FaceStore (``<person_id>/meta.json`` + ``reference.jpg`` + ``embeddings.npy``).
Ходим тем же ``rob_box_core.face_roster``, что и Telegram ``/faces`` (#3025).

Запуск::

    pytest -v src/rob_box_supervisor/test/unit/test_face_roster_tool.py
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional
from unittest.mock import MagicMock

import pytest

from rob_box_supervisor.face_roster_tool import (
    DEFAULT_LIMIT,
    MAX_LIMIT,
    TOOL_NAME,
    build_faces_listing,
    register_list_faces_tool,
)

NOW = 1_800_000_000.0


def _person(
    root: Path,
    person_id: str,
    *,
    name: Optional[str] = None,
    encounters: int = 0,
    last: Optional[float] = None,
    created: Optional[float] = None,
) -> None:
    d = root / person_id
    d.mkdir(parents=True)
    meta: Dict[str, Any] = {"person_id": person_id, "encounter_count": encounters}
    if name:
        meta["name"] = name
    if last is not None:
        meta["last_encounter_ts"] = last
    if created is not None:
        meta["created_at"] = created
    meta["speaker_id"] = "spk-secret"
    (d / "meta.json").write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    (d / "reference.jpg").write_bytes(b"\xff\xd8\xff\xd9")
    (d / "embeddings.npy").write_bytes(b"not-a-real-npy")


@pytest.fixture()
def faces_root(tmp_path: Path) -> Path:
    root = tmp_path / "faces"
    root.mkdir()
    _person(root, "aaaaaaaa11", name="Деньчик", encounters=27, last=NOW - 120)
    _person(root, "bbbbbbbb22", name=None, encounters=5, last=NOW - 7200)
    _person(root, "cccccccc33", name="Борис", encounters=3, last=NOW - 3 * 86400)
    _person(root, "dddddddd44", name=None, encounters=1, created=NOW - 600)
    return root


def test_listing_sorted_by_last_encounter_newest_first(faces_root: Path) -> None:
    out = build_faces_listing(str(faces_root), now=NOW)
    assert out["status"] == "ok"
    assert [f["id"] for f in out["faces"]] == [
        "aaaaaaaa",  # 2 мин назад
        "dddddddd",  # нет встреч — берём created_at, 10 мин
        "bbbbbbbb",  # 2 ч
        "cccccccc",  # 3 дня
    ]
    first = out["faces"][0]
    assert first["name"] == "Деньчик"
    assert first["encounters"] == 27
    assert first["last_seen_ago"] == "2 мин назад"
    assert first["last_seen"].endswith("UTC")
    assert out["faces"][2]["name"] == "незнакомец"
    assert out["faces"][3]["last_seen_ago"] == "3 дн назад"
    assert (out["total_in_base"], out["named_in_base"]) == (4, 2)


def test_listing_leaks_no_embeddings_paths_or_speaker(faces_root: Path) -> None:
    out = build_faces_listing(str(faces_root), now=NOW)
    blob = json.dumps(out, ensure_ascii=False)
    assert "spk-secret" not in blob
    assert "embeddings" not in blob
    assert str(faces_root) not in blob
    assert ".jpg" not in blob
    assert set(out["faces"][0]) == {
        "id", "name", "encounters", "last_seen", "last_seen_ago",
    }


def test_limit_and_named_only(faces_root: Path) -> None:
    out = build_faces_listing(str(faces_root), limit=2, now=NOW)
    assert out["shown"] == 2 and out["total_in_base"] == 4
    named = build_faces_listing(str(faces_root), named_only=True, now=NOW)
    assert [f["name"] for f in named["faces"]] == ["Деньчик", "Борис"]
    assert named["total_in_base"] == 4


def test_limit_is_clamped(tmp_path: Path) -> None:
    root = tmp_path / "f"
    root.mkdir()
    for i in range(MAX_LIMIT + 5):
        _person(root, f"{i:08x}", name=f"p{i}", last=NOW - i)
    assert build_faces_listing(str(root), limit=10_000, now=NOW)["shown"] == MAX_LIMIT
    assert build_faces_listing(str(root), limit=0, now=NOW)["shown"] == 1


def test_empty_base_is_honest(tmp_path: Path) -> None:
    out = build_faces_listing(str(tmp_path), now=NOW)
    assert out["status"] == "ok"
    assert out["faces"] == [] and out["total_in_base"] == 0
    assert "пуста" in out["message"]


def test_missing_mount_is_unavailable_not_empty(tmp_path: Path) -> None:
    out = build_faces_listing(str(tmp_path / "nope"), now=NOW)
    assert out["status"] == "unavailable"
    assert "faces" not in out


def test_listing_never_writes_to_base(faces_root: Path) -> None:
    def snapshot() -> Dict[str, tuple]:
        return {
            str(p): (p.stat().st_mtime_ns, p.stat().st_size)
            for p in faces_root.rglob("*")
        }

    before = snapshot()
    build_faces_listing(str(faces_root), now=NOW)
    assert snapshot() == before


def test_broken_meta_does_not_break_listing(faces_root: Path) -> None:
    (faces_root / "cccccccc33" / "meta.json").write_text("{not json", encoding="utf-8")
    out = build_faces_listing(str(faces_root), now=NOW)
    assert out["status"] == "ok" and out["total_in_base"] == 4


def test_register_tool_spec_and_handler(faces_root: Path) -> None:
    registry = MagicMock()
    register_list_faces_tool(registry, root=str(faces_root))
    spec, handler = registry.register.call_args.args
    assert spec.name == TOOL_NAME == "list_faces"
    assert spec.parameters["required"] == []
    # описание самодостаточно: промпт ТАРС не правится (#3298)
    assert "face" in spec.description and "last seen" in spec.description
    out = handler({"limit": "2"})
    assert out["status"] == "ok" and out["shown"] == 2
    assert handler({"limit": "abc"})["shown"] == min(4, DEFAULT_LIMIT)
    assert registry.register.call_args.kwargs == {"override": True}


def test_register_tool_uses_env_root(
    faces_root: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("FACE_STORE_ROOT", str(faces_root))
    registry = MagicMock()
    register_list_faces_tool(registry)
    _, handler = registry.register.call_args.args
    assert handler({})["total_in_base"] == 4
    assert os.environ["FACE_STORE_ROOT"] == str(faces_root)
