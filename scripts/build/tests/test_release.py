"""scripts/build/release.py: сборка release.yaml из репо, бандлы, подпись/проверка, .image-versions, публикация.

Без сети и Docker: digest образов — из фейкового резолвера, реестр — фейк из host/chuangong/tests/fakes.py.
Ключ подписи генерируется в тесте.
"""

from __future__ import annotations

import copy
import gzip
import hashlib
import importlib.util
import io
import re
import sys
import tarfile
from pathlib import Path, PurePosixPath

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "host" / "chuangong"))
sys.path.insert(0, str(REPO_ROOT / "host" / "chuangong" / "tests"))

_SPEC = importlib.util.spec_from_file_location("release_under_test", REPO_ROOT / "scripts" / "build" / "release.py")
release_py = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(release_py)

from chuangong import minisign  # noqa: E402
from chuangong import release_format as rf  # noqa: E402
from chuangong.agent import Config, Deps, tick  # noqa: E402
from chuangong.registry import Registry  # noqa: E402
from fakes import FakeDocker, FakeTransport, FakeZenoh  # noqa: E402

PREFIX = "10.1.1.249:5000/krikz/rob_box"


def _fake_digest(tag: str) -> str:
    return "sha256:" + hashlib.sha256(tag.encode()).hexdigest()


@pytest.fixture(scope="module")
def built():
    asked: list[str] = []

    def resolve(tag):
        asked.append(tag)
        return _fake_digest(tag)

    release, bundles = release_py.build_release(
        REPO_ROOT, "2026.11.09-1", "dev", "b704404", "dev-b704404", "dev", resolve, created="2026-11-09T00:00:00Z"
    )
    return release, bundles, asked


def _names(bundle: bytes) -> list[str]:
    with tarfile.open(fileobj=io.BytesIO(bundle), mode="r:gz") as tf:
        return tf.getnames()


def test_roles_take_images_from_their_compose(built):
    release, _, asked = built
    assert set(release["roles"]["vision"]["images"]) == {"ceiling-camera", "led-matrix", "oak-d", "vision-hailo"}
    assert {"voice-assistant", "supervisor", "telegram-bot", "quest", "supercollider", "perception"} == set(
        release["roles"]["sound"]["images"]
    )
    assert "teleop" in release["roles"]["main"]["images"] and "nav2" in release["roles"]["main"]["images"]
    for role in rf.ROLES:
        for ref in release["roles"][role]["images"].values():
            assert re.match(rf"^{re.escape(PREFIX)}@sha256:[0-9a-f]{{64}}$", ref)
        assert "eclipse/zenoh:1.6.2" in release["roles"][role]["external_images"]
    # *_TAG-сервисы — по SHA-тегу, остальные — по тегу канала (как compose берёт сегодня)
    assert "oak-d-humble-dev-b704404" in asked
    assert "supercollider-dev" in asked and "teleop-humble-dev" in asked and "quest-humble-dev" in asked
    assert release["roles"]["vision"]["images"]["oak-d"] == f"{PREFIX}@{_fake_digest('oak-d-humble-dev-b704404')}"


def test_resource_pack_and_migrations(built):
    release, _, _ = built
    data = (REPO_ROOT / release_py.RESOURCE_PACK_MANIFEST).read_bytes().replace(b"\r\n", b"\n")
    for role in ("vision", "sound"):
        rp = release["roles"][role]["resource_pack"]
        assert rp["manifest"] == "docker/vision/scripts/resource_pack/manifest.yaml"
        assert rp["sha256"] == hashlib.sha256(data).hexdigest()
    assert release["roles"]["main"]["resource_pack"] is None
    assert release["roles"]["vision"]["migrations"] == ["scripts/setup/ensure_hailo_driver.sh"]


def test_bundles_are_deterministic_and_have_no_secrets(built):
    release, bundles, _ = built
    again = {role: release_py.role_bundle(REPO_ROOT, role) for role in rf.ROLES}
    assert again == bundles
    for role in rf.ROLES:
        names = _names(bundles[role])
        assert f"docker/{role}/docker-compose.yaml" in names
        assert not [n for n in names if n.endswith(".env.secrets") or "/.env.local" in n]
        assert release["roles"][role]["bundle"]["sha256"] == hashlib.sha256(bundles[role]).hexdigest()
    with tarfile.open(fileobj=io.BytesIO(bundles["main"]), mode="r:gz") as tf:
        assert {m.mtime for m in tf.getmembers()} == {0}
    assert gzip.decompress(bundles["main"])  # обычный gzip


def test_secret_like_file_is_refused():
    with pytest.raises(release_py.BuildError, match="секрет"):
        release_py.build_bundle([("docker/sound/.env.secrets", b"TOKEN=1", False)])


def test_compose_relative_paths_are_inside_role_bundle():
    """Всё, на что compose роли ссылается через ../, едет в бандле роли (иначе compose в бандле не встанет)."""
    for role in rf.ROLES:
        text = (REPO_ROOT / "docker" / role / "docker-compose.yaml").read_text(encoding="utf-8")
        refs = set(re.findall(r"(\.\./[A-Za-z0-9_./-]+)", text))
        assert refs, role
        for ref in refs:
            path = PurePosixPath("docker", role, ref)
            parts: list[str] = []
            for part in path.parts:
                parts = parts[:-1] if part == ".." else parts + [part]
            target = "/".join(parts)
            assert any(
                target == p or target.startswith(p + "/") for p in release_py.BUNDLE_PATHS[role]
            ), f"{role}: {ref} → {target} не в BUNDLE_PATHS"


def test_image_versions_from_release_match_current_files(built):
    """Переходный период: .image-versions, сгенерированный из релиза, несёт те же поля, что файлы в репо сегодня."""
    release, _, _ = built
    manifest = release_py.GBM.load_manifest(REPO_ROOT / release_py.BUILD_MANIFEST)
    for role in rf.ROLES:
        text = release_py.image_versions_text(release, role, manifest)
        got = dict(ln.split("=", 1) for ln in text.splitlines() if ln and not ln.startswith("#"))
        current = (REPO_ROOT / "docker" / role / ".image-versions.dev").read_text(encoding="utf-8")
        want = {ln.split("=", 1)[0] for ln in current.splitlines() if ln.strip() and not ln.startswith("#")}
        assert set(got) == want, role
        assert set(got.values()) == {"dev-b704404"}
    pinned = release_py.image_versions_text(release, "vision", manifest, pinned=True)
    assert f"OAK_D_TAG=dev-b704404@{_fake_digest('oak-d-humble-dev-b704404')}" in pinned


def _signed_dir(tmp_path, built):
    release, bundles, _ = built
    out = tmp_path / "out"
    release_py.write_release_dir(out, release, bundles)
    secret, public = minisign.generate_keypair()
    release_py.sign_release_dir(out, secret)
    return out, public


def test_sign_and_verify(tmp_path, built):
    out, public = _signed_dir(tmp_path, built)
    assert release_py.verify_release_dir(out, public)["release"] == "2026.11.09-1"
    assert "trusted comment: release:2026.11.09-1 channel:dev git:b704404" in (out / rf.SIGNATURE_FILE).read_text()


def test_verify_refuses_substituted_digest(tmp_path, built):
    out, public = _signed_dir(tmp_path, built)
    path = out / rf.RELEASE_FILE
    text = path.read_text(encoding="utf-8")
    good = _fake_digest("nav2-humble-dev-b704404").split(":")[1]
    path.write_text(text.replace(good, "0" * 64), encoding="utf-8")
    with pytest.raises(minisign.SignatureError):
        release_py.verify_release_dir(out, public)


def test_verify_refuses_substituted_bundle(tmp_path, built):
    out, public = _signed_dir(tmp_path, built)
    release = built[0]
    (out / release["roles"]["sound"]["bundle"]["file"]).write_bytes(built[1]["main"])
    with pytest.raises(rf.ReleaseError, match="подмена"):
        release_py.verify_release_dir(out, public)


def test_unpinned_image_cannot_be_released(built):
    bad = copy.deepcopy(built[0])
    bad["roles"]["main"]["images"]["nav2"] = f"{PREFIX}:nav2-humble-dev"
    with pytest.raises(rf.ReleaseError):
        rf.dumps(bad)


def test_cli_build_sign_verify(tmp_path):
    digests = {}
    manifest = release_py.GBM.load_manifest(REPO_ROOT / release_py.BUILD_MANIFEST)
    for name, spec in release_py.all_services(manifest).items():
        for tag in ("dev-b704404", "dev"):
            t = release_py.image_tag_for(name, spec, tag, tag, "humble")
            digests[t] = _fake_digest(t)
    (tmp_path / "d.json").write_text(__import__("json").dumps(digests))
    secret, public = minisign.generate_keypair()
    (tmp_path / "k").write_text(secret)
    (tmp_path / "k.pub").write_text(public)
    out = str(tmp_path / "out")
    build = ["build", "--release", "r1", "--image-tag", "dev-b704404", "--git", "b704404", "--out", out]
    assert release_py.main(build + ["--digests", str(tmp_path / "d.json")]) == 0
    assert release_py.main(["sign", "--dir", out, "--key", str(tmp_path / "k")]) == 0
    assert release_py.main(["verify", "--dir", out, "--pubkey", str(tmp_path / "k.pub")]) == 0
    other = minisign.generate_keypair()[1]
    (tmp_path / "other.pub").write_text(other)
    assert release_py.main(["verify", "--dir", out, "--pubkey", str(tmp_path / "other.pub")]) == 1


def test_publish_then_agent_sees_release(tmp_path, built):
    """Сквозной путь без сети: release.py publish → реестр → агент observe видит релиз и верную подпись."""
    out, public = _signed_dir(tmp_path, built)
    transport = FakeTransport()
    registry = Registry("http://10.1.1.249:5000", transport)
    digest = release_py.publish_release_dir(out, registry)
    assert transport.manifests[(rf.RELEASE_REPO, "dev")] == transport.manifests[(rf.RELEASE_REPO, digest)]

    (tmp_path / "release.pub").write_text(public)
    cfg = Config(robot_id="RBXU100001", node="sound", pubkey=tmp_path / "release.pub", state_dir=tmp_path / "state")
    rec = tick(cfg, Deps(registry, FakeDocker(), FakeZenoh()))
    assert rec["state"] == "available", rec["reason"]
    assert rec["available"] == "2026.11.09-1" and rec["release_git"] == "b704404"


def test_compose_images_rejects_unknown_template():
    with pytest.raises(release_py.BuildError):
        release_py.compose_images(yaml.safe_load("services:\n  x:\n    image: ${OTHER}:tag\n"))
