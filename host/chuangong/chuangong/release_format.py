"""Релиз как артефакт: схема release.yaml, раскладка OCI-артефакта, проверки целостности (ADR-0168 §2).

Здесь — единственное знание о формате релиза. Сборщик (`scripts/build/release.py`) и агент на Pi
импортируют отсюда; в других местах формат не повторять.

release.yaml (подписан minisign, подпись — соседний файл release.yaml.minisig):

    schema: 1
    release: 2026.11.09-1          # имя релиза, оно же каталог /opt/rob_box/releases/<release>
    channel: dev
    git: <sha коммита>
    created: 2026-11-09T14:00:00Z
    image_tag: dev-b704404         # SHA-тег сборки (для .image-versions на переходный период)
    docker_tag: dev                # изменяемый тег канала (сервисы без *_TAG в .image-versions)
    roles:
      vision:
        bundle: {file: bundle-vision-<release>.tar.gz, sha256: <hex>, size: <байт>}
        images: {oak-d: <registry>/krikz/rob_box@sha256:<hex>, ...}
        external_images: [eclipse/zenoh:1.6.2, ...]   # сторонние образы compose, пока по тегу (не закреплены)
        resource_pack: {manifest: docker/vision/scripts/resource_pack/manifest.yaml, sha256: <hex>}
        migrations: [scripts/setup/ensure_hailo_driver.sh]
    compat: {min_previous: null}

В реестре релиз — один OCI-манифест в репозитории `RELEASE_REPO` с тегом `<release>`; указатель канала —
тег `<channel>` на тот же манифест. Слои: release.yaml, release.yaml.minisig и бандлы трёх ролей.
"""

from __future__ import annotations

import hashlib
import json
import re
from typing import Any

import yaml

SCHEMA = 1
ROLES = ("main", "vision", "sound")
RELEASE_REPO = "krikz/rob_box/release"
RELEASE_FILE = "release.yaml"
SIGNATURE_FILE = "release.yaml.minisig"

ARTIFACT_TYPE = "application/vnd.robbox.release.v1"
MT_MANIFEST = "application/vnd.oci.image.manifest.v1+json"
MT_EMPTY = "application/vnd.oci.empty.v1+json"
MT_RELEASE = "application/vnd.robbox.release.manifest.v1+yaml"
MT_SIGNATURE = "application/vnd.robbox.release.signature.v1+minisign"
MT_BUNDLE = "application/vnd.robbox.release.bundle.v1.tar+gzip"
ANN_TITLE = "org.opencontainers.image.title"
ANN_ROLE = "org.robbox.role"
EMPTY_CONFIG = b"{}"

# Имя релиза становится каталогом на Pi и тегом в реестре: только безопасные символы, без «/» и «..».
_RELEASE_RE = re.compile(r"^[0-9A-Za-z][0-9A-Za-z._-]{0,63}$")
_CHANNEL_RE = re.compile(r"^[a-z][a-z0-9-]{0,31}$")
_HEX64 = re.compile(r"^[0-9a-f]{64}$")
_PINNED_REF = re.compile(r"^[a-z0-9][a-z0-9._:/-]*@sha256:[0-9a-f]{64}$")
_SERVICE = re.compile(r"^[a-z0-9][a-z0-9-]*$")


class ReleaseError(ValueError):
    """release.yaml не соответствует схеме или не сошлась целостность."""


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def oci_digest(data: bytes) -> str:
    return f"sha256:{sha256_hex(data)}"


def bundle_file_name(role: str, release: str) -> str:
    return f"bundle-{role}-{release}.tar.gz"


def check_release_name(name: str) -> str:
    if not isinstance(name, str) or not _RELEASE_RE.match(name) or ".." in name:
        raise ReleaseError(f"имя релиза {name!r}: допустимы [0-9A-Za-z._-], до 64 символов, без «..»")
    return name


def check_channel(name: str) -> str:
    if not isinstance(name, str) or not _CHANNEL_RE.match(name):
        raise ReleaseError(f"канал {name!r}: допустимы [a-z0-9-], с буквы")
    return name


def dumps(release: dict[str, Any]) -> bytes:
    """Каноничные байты release.yaml (подписываются ровно они)."""
    validate(release)
    return yaml.safe_dump(release, sort_keys=True, allow_unicode=True, default_flow_style=False).encode()


def loads(data: bytes) -> dict[str, Any]:
    try:
        release = yaml.safe_load(data.decode("utf-8"))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ReleaseError(f"release.yaml не читается: {exc}") from exc
    validate(release)
    return release


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ReleaseError(msg)


def _validate_bundle(role: str, bundle: Any, release: str) -> None:
    _require(isinstance(bundle, dict), f"roles.{role}.bundle: ожидался словарь")
    _require(bundle.get("file") == bundle_file_name(role, release), f"roles.{role}.bundle.file: неверное имя")
    _require(_HEX64.match(str(bundle.get("sha256", ""))) is not None, f"roles.{role}.bundle.sha256: не sha256")
    size = bundle.get("size")
    _require(isinstance(size, int) and size > 0, f"roles.{role}.bundle.size: ожидалось целое > 0")


def _validate_role(role: str, entry: Any, release: str) -> None:
    _require(isinstance(entry, dict), f"roles.{role}: ожидался словарь")
    _validate_bundle(role, entry.get("bundle"), release)
    images = entry.get("images")
    _require(isinstance(images, dict) and images, f"roles.{role}.images: пусто")
    for svc, ref in images.items():
        _require(_SERVICE.match(str(svc)) is not None, f"roles.{role}.images: имя сервиса {svc!r}")
        _require(_PINNED_REF.match(str(ref)) is not None, f"roles.{role}.images.{svc}: не закреплён по digest: {ref!r}")
    _require(isinstance(entry.get("external_images", []), list), f"roles.{role}.external_images: ожидался список")
    rp = entry.get("resource_pack")
    if rp is not None:
        _require(isinstance(rp, dict), f"roles.{role}.resource_pack: ожидался словарь")
        _require(_HEX64.match(str(rp.get("sha256", ""))) is not None, f"roles.{role}.resource_pack.sha256")
    _require(isinstance(entry.get("migrations", []), list), f"roles.{role}.migrations: ожидался список")


def validate(release: Any) -> None:
    _require(isinstance(release, dict), "release.yaml: ожидался словарь")
    _require(release.get("schema") == SCHEMA, f"schema: ожидалась {SCHEMA}, пришла {release.get('schema')!r}")
    name = check_release_name(release.get("release"))
    check_channel(release.get("channel"))
    roles = release.get("roles")
    _require(isinstance(roles, dict) and set(roles) == set(ROLES), f"roles: нужны ровно {list(ROLES)}")
    for role in ROLES:
        _validate_role(role, roles[role], name)
    compat = release.get("compat")
    _require(isinstance(compat, dict) and "min_previous" in compat, "compat.min_previous: нет поля")
    if compat["min_previous"] is not None:
        check_release_name(compat["min_previous"])


def check_bundle(release: dict[str, Any], role: str, data: bytes) -> None:
    """Бандл совпал с подписанным release.yaml по размеру и sha256 — иначе ReleaseError."""
    want = release["roles"][role]["bundle"]
    if len(data) != want["size"] or sha256_hex(data) != want["sha256"]:
        raise ReleaseError(f"бандл {role}: не совпал с release.yaml (размер/sha256) — подмена или порча")


# --- OCI-артефакт ---------------------------------------------------------------------------------------------


def _layer(media_type: str, data_digest: str, size: int, title: str, role: str | None = None) -> dict[str, Any]:
    ann = {ANN_TITLE: title}
    if role:
        ann[ANN_ROLE] = role
    return {"mediaType": media_type, "digest": data_digest, "size": size, "annotations": ann}


def artifact_manifest(release_bytes: bytes, signature_bytes: bytes, release: dict[str, Any]) -> bytes:
    """OCI image manifest релиза: release.yaml, подпись, бандлы ролей. Байты — то, что кладётся в реестр."""
    layers = [
        _layer(MT_RELEASE, oci_digest(release_bytes), len(release_bytes), RELEASE_FILE),
        _layer(MT_SIGNATURE, oci_digest(signature_bytes), len(signature_bytes), SIGNATURE_FILE),
    ]
    for role in ROLES:
        b = release["roles"][role]["bundle"]
        layers.append(_layer(MT_BUNDLE, f"sha256:{b['sha256']}", b["size"], b["file"], role))
    manifest = {
        "schemaVersion": 2,
        "mediaType": MT_MANIFEST,
        "artifactType": ARTIFACT_TYPE,
        "config": {"mediaType": MT_EMPTY, "digest": oci_digest(EMPTY_CONFIG), "size": len(EMPTY_CONFIG)},
        "layers": layers,
        "annotations": {"org.robbox.release": release["release"], "org.robbox.channel": release["channel"]},
    }
    return json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode()


def find_layer(manifest: dict[str, Any], media_type: str, role: str | None = None) -> dict[str, Any]:
    """Слой артефакта по типу (и роли для бандла). Нет ровно одного — ReleaseError."""
    found = [
        lay
        for lay in manifest.get("layers") or []
        if lay.get("mediaType") == media_type and (role is None or (lay.get("annotations") or {}).get(ANN_ROLE) == role)
    ]
    if len(found) != 1:
        raise ReleaseError(f"артефакт: слоёв {media_type} (роль {role}) — {len(found)}, ожидался один")
    return found[0]
