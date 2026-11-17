"""Манифест приёмки: source, core, образ, сценарий и proven runtime — разные поля.

#4694: merged ≠ installed ≠ accepted. Один тег или факт сборки не заполняет
``proven_runtime``. Схема — этот модуль; экземпляр —
``acceptance_manifest.yaml`` рядом.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

SCHEMA = 1
FIELDS = (
    "source_sha",
    "core_package_version",
    "image_tag",
    "scenario_id",
    "proven_runtime",
)
IDENTITY_FIELDS = FIELDS[:-1]
PROVEN_RAW_KEYS = ("run_id", "image_digest", "started_at", "installed_core")
ALLOWED_KEYS = frozenset(("schema",) + FIELDS)
REPO_MANIFEST = Path(__file__).with_name("acceptance_manifest.yaml")


class AcceptanceManifestError(ValueError):
    """Манифест приёмки смешивает поля или выдаёт сборку за живой runtime."""


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise AcceptanceManifestError(msg)


def _as_optional_str(value: Any, field: str) -> str:
    if value is None:
        return ""
    _require(
        isinstance(value, str), f"{field}: ожидалась строка, не {type(value).__name__}"
    )
    return value.strip()


def _blank(value: Any) -> bool:
    return value is None or value == "" or value == {}


def is_proven(manifest: Mapping[str, Any]) -> bool:
    """True только если proven_runtime несёт raw живого прогона, не тег сборки."""
    proven = manifest.get("proven_runtime")
    if not isinstance(proven, dict) or not proven:
        return False
    run_id = proven.get("run_id")
    if not isinstance(run_id, str) or not run_id.strip():
        return False
    return any(
        isinstance(proven.get(key), str) and proven[key].strip()
        for key in PROVEN_RAW_KEYS[1:]
    )


def _validate_proven(proven: Any, identity: Mapping[str, str]) -> Any:
    if _blank(proven):
        return None
    _require(
        isinstance(proven, dict),
        "proven_runtime: ожидался словарь raw или пусто, не одна строка/тег",
    )
    unknown = sorted(set(proven) - set(PROVEN_RAW_KEYS))
    _require(not unknown, f"proven_runtime: неизвестные ключи {unknown}")
    cleaned = {
        key: _as_optional_str(proven.get(key), f"proven_runtime.{key}")
        for key in PROVEN_RAW_KEYS
    }
    if all(_blank(cleaned[key]) for key in PROVEN_RAW_KEYS):
        return None
    _require(
        bool(cleaned["run_id"]), "proven_runtime.run_id: пуст, живой прогон не записан"
    )
    _require(
        any(cleaned[key] for key in PROVEN_RAW_KEYS[1:]),
        "proven_runtime: нет raw (image_digest / started_at / installed_core)",
    )
    # installed_core может совпасть с core_package_version — это сверка пакета,
    # не подмена. Запрет — выдавать git SHA / тег образа / id сценария за raw.
    stolen = {
        identity["source_sha"],
        identity["image_tag"],
        identity["scenario_id"],
    } - {""}
    overlap = [
        key
        for key, value in cleaned.items()
        if key != "installed_core" and value in stolen
    ]
    _require(
        not overlap,
        "proven_runtime не копирует source_sha / image_tag / scenario_id: "
        + ", ".join(overlap),
    )
    return cleaned


def validate(obj: Any) -> dict[str, Any]:
    _require(
        isinstance(obj, dict),
        "манифест приёмки: ожидался словарь с пятью полями, не одна строка",
    )
    unknown = sorted(set(obj) - ALLOWED_KEYS)
    _require(not unknown, f"неизвестные ключи {unknown}")
    schema = obj.get("schema", SCHEMA)
    _require(schema == SCHEMA, f"schema: ожидалось {SCHEMA}, получено {schema!r}")
    missing = [field for field in FIELDS if field not in obj]
    _require(not missing, f"нет полей {missing}")
    identity = {field: _as_optional_str(obj[field], field) for field in IDENTITY_FIELDS}
    proven = _validate_proven(obj["proven_runtime"], identity)
    return {"schema": SCHEMA, **identity, "proven_runtime": proven}


def load(path: Path | None = None) -> dict[str, Any]:
    target = REPO_MANIFEST if path is None else Path(path)
    try:
        data = yaml.safe_load(target.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise AcceptanceManifestError(f"{target}: не читается: {exc}") from exc
    return validate(data)
