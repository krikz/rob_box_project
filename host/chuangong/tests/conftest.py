"""Общий мир тестов chuangong: подписанный релиз в фейковом реестре (фейки — fakes.py)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from chuangong import minisign  # noqa: E402
from chuangong import release_format as rf  # noqa: E402
from chuangong.registry import Registry  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fakes import REG_HOST, FakeTransport, digest_of, make_bundle  # noqa: E402,F401


class World:
    """Реестр с опубликованным подписанным релизом и ключ проверки на «диске Pi»."""

    def __init__(self, tmp: Path, release_name: str = "2026.11.09-1") -> None:
        self.transport = FakeTransport()
        self.registry = Registry(f"http://{REG_HOST}", self.transport)
        secret, public = minisign.generate_keypair()
        self.secret = minisign.load_secret_key(secret)
        self.pubkey = tmp / "release.pub"
        self.pubkey.write_text(public, encoding="utf-8")
        self.images: dict[str, dict[str, str]] = {}
        self.bundles: dict[str, bytes] = {}
        self.publish(release_name)

    def _image(self, role: str, svc: str) -> str:
        layer = f"layer-{role}-{svc}".encode() * 10
        cfg = b"{}"
        self.transport.blobs[digest_of(layer)] = layer
        manifest = json.dumps(
            {
                "schemaVersion": 2,
                "mediaType": "application/vnd.oci.image.manifest.v1+json",
                "config": {
                    "mediaType": "application/vnd.oci.image.config.v1+json",
                    "digest": digest_of(cfg),
                    "size": 2,
                },
                "layers": [{"mediaType": "x", "digest": digest_of(layer), "size": len(layer)}],
            }
        ).encode()
        d = digest_of(manifest)
        self.transport.manifests[("krikz/rob_box", d)] = manifest
        return f"{REG_HOST}/krikz/rob_box@{d}"

    def release_dict(self, name: str) -> dict:
        roles = {}
        for role in rf.ROLES:
            bundle = make_bundle(
                {
                    f"docker/{role}/docker-compose.yaml": f"# {role} {name}\n".encode(),
                    f"docker/{role}/run.sh": b"#!/bin/sh\n",
                }
            )
            self.bundles[role] = bundle
            self.images[role] = {f"{role}-svc{i}": self._image(role, f"svc{i}") for i in range(2)}
            roles[role] = {
                "bundle": {
                    "file": rf.bundle_file_name(role, name),
                    "sha256": rf.sha256_hex(bundle),
                    "size": len(bundle),
                },
                "images": self.images[role],
                "external_images": ["eclipse/zenoh:1.6.2"],
                "resource_pack": None,
                "migrations": [],
            }
        return {
            "schema": 1,
            "release": name,
            "channel": "dev",
            "git": "abc1234",
            "created": "2026-11-09T12:00:00Z",
            "image_tag": "dev-abc1234",
            "docker_tag": "dev",
            "roles": roles,
            "compat": {"min_previous": None},
        }

    def publish(self, name: str, release_bytes: bytes | None = None, signature: str | None = None) -> None:
        release = self.release_dict(name)
        data = rf.dumps(release)
        sig = signature or minisign.sign(data, self.secret, f"release:{name}")
        data = release_bytes if release_bytes is not None else data
        for role in rf.ROLES:
            self.transport.blobs[digest_of(self.bundles[role])] = self.bundles[role]
        self.transport.blobs[digest_of(data)] = data
        self.transport.blobs[digest_of(sig.encode())] = sig.encode()
        manifest = rf.artifact_manifest(data, sig.encode(), release)
        for ref in (name, "dev"):
            self.transport.manifests[(rf.RELEASE_REPO, ref)] = manifest
        self.release = release


@pytest.fixture
def world(tmp_path):
    return World(tmp_path)
