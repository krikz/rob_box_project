"""#4694: манифест приёмки держит source, core, образ, сценарий и proven runtime раздельно."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "ci" / "e2e" / "acceptance_manifest.py"
REPO_YAML = REPO_ROOT / "scripts" / "ci" / "e2e" / "acceptance_manifest.yaml"

_SPEC = importlib.util.spec_from_file_location(
    "acceptance_manifest_under_test", SCRIPT_PATH
)
assert _SPEC is not None and _SPEC.loader is not None, f"{SCRIPT_PATH}: spec failed"
am = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(am)


def _valid(**overrides):
    data = {
        "schema": 1,
        "source_sha": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        "core_package_version": "0.7.4",
        "image_tag": "voice-assistant-humble-dev-deadbeef",
        "scenario_id": "night_marathon_act3_backlog_diarization_v1",
        "proven_runtime": None,
    }
    data.update(overrides)
    return data


def test_repo_yaml_has_five_distinct_empty_fields():
    raw = yaml.safe_load(REPO_YAML.read_text(encoding="utf-8"))
    assert raw["schema"] == 1
    for field in am.FIELDS:
        assert field in raw, field
    loaded = am.load(REPO_YAML)
    assert loaded["source_sha"] == ""
    assert loaded["core_package_version"] == ""
    assert loaded["image_tag"] == ""
    assert loaded["scenario_id"] == ""
    assert loaded["proven_runtime"] is None
    assert am.is_proven(loaded) is False


def test_one_string_for_everything_is_rejected():
    with pytest.raises(am.AcceptanceManifestError, match="словарь"):
        am.validate("dev-deadbeef")


def test_missing_field_is_rejected():
    data = _valid()
    del data["core_package_version"]
    with pytest.raises(am.AcceptanceManifestError, match="core_package_version"):
        am.validate(data)


def test_proven_runtime_string_tag_is_rejected():
    with pytest.raises(am.AcceptanceManifestError, match="не одна строка"):
        am.validate(_valid(proven_runtime="voice-assistant-humble-dev-deadbeef"))


def test_proven_runtime_copied_from_image_tag_is_rejected():
    tag = "voice-assistant-humble-dev-deadbeef"
    with pytest.raises(am.AcceptanceManifestError, match="не копирует"):
        am.validate(
            _valid(
                image_tag=tag,
                proven_runtime={
                    "run_id": tag,
                    "image_digest": "sha256:" + "a" * 64,
                },
            )
        )


def test_build_facts_alone_are_not_proven():
    loaded = am.validate(_valid())
    assert am.is_proven(loaded) is False
    assert loaded["proven_runtime"] is None


def test_proven_runtime_without_raw_is_rejected():
    with pytest.raises(am.AcceptanceManifestError, match="raw"):
        am.validate(_valid(proven_runtime={"run_id": "43191155293"}))


def test_live_raw_is_proven_and_does_not_fill_identity():
    digest = "sha256:" + "b" * 64
    loaded = am.validate(
        _valid(
            proven_runtime={
                "run_id": "43193585541",
                "image_digest": digest,
                "started_at": "2026-11-12T09:33:24Z",
                "installed_core": "0.7.4",
            }
        )
    )
    assert am.is_proven(loaded) is True
    assert loaded["proven_runtime"]["image_digest"] == digest
    assert loaded["source_sha"] != digest
    assert loaded["image_tag"] != digest
