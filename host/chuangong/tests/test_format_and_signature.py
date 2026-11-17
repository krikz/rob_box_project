"""minisign (подпись/проверка, совместимость с утилитой), схема release.yaml, безопасная распаковка бандла."""

from __future__ import annotations

import copy
import io
import shutil
import subprocess
import tarfile

import pytest
from chuangong import minisign
from chuangong import release_format as rf
from chuangong.stage import StageError, extract_bundle
from fakes import make_bundle


def test_sign_verify_roundtrip_and_tamper():
    secret_text, public_text = minisign.generate_keypair()
    secret, public = minisign.load_secret_key(secret_text), minisign.load_public_key(public_text)
    sig = minisign.sign(b"release body", secret, "release:2026.11.09-1")
    assert minisign.verify(b"release body", sig, public) == "release:2026.11.09-1"
    with pytest.raises(minisign.SignatureError, match="не сошлась"):
        minisign.verify(b"release bodY", sig, public)
    lines = sig.splitlines()
    lines[2] = "trusted comment: release:evil"
    with pytest.raises(minisign.SignatureError, match="не сошлась"):
        minisign.verify(b"release body", "\n".join(lines), public)


def test_signature_from_other_key_and_garbage():
    s1, _ = minisign.generate_keypair()
    _, p2 = minisign.generate_keypair()
    sig = minisign.sign(b"x", minisign.load_secret_key(s1), "c")
    with pytest.raises(minisign.SignatureError, match="другим ключом"):
        minisign.verify(b"x", sig, minisign.load_public_key(p2))
    with pytest.raises(minisign.SignatureError):
        minisign.verify(b"x", "garbage", minisign.load_public_key(p2))


def test_secret_key_checksum_is_checked():
    secret_text, _ = minisign.generate_keypair()
    head, body = secret_text.splitlines()[:2]
    raw = bytearray(__import__("base64").b64decode(body))
    raw[70] ^= 0xFF  # байт секретного ключа
    broken = head + "\n" + __import__("base64").b64encode(bytes(raw)).decode() + "\n"
    with pytest.raises(minisign.SignatureError, match="контрольная сумма"):
        minisign.load_secret_key(broken)


def test_cli_style_key_with_zero_checksum():
    """`minisign -G -W` оставляет контрольную сумму нулями (CI 09.11 поймал это тестом совместимости)."""
    import base64

    secret_text, _ = minisign.generate_keypair()
    head, body = secret_text.splitlines()[:2]
    raw = bytearray(base64.b64decode(body))
    raw[126:158] = bytes(32)
    as_cli = head + "\n" + base64.b64encode(bytes(raw)).decode() + "\n"
    assert minisign.load_secret_key(as_cli).seed == minisign.load_secret_key(secret_text).seed
    raw[70] ^= 0xFF
    broken = head + "\n" + base64.b64encode(bytes(raw)).decode() + "\n"
    with pytest.raises(minisign.SignatureError, match="открытая половина"):
        minisign.load_secret_key(broken)


@pytest.mark.skipif(shutil.which("minisign") is None, reason="утилиты minisign нет на этой машине")
def test_interop_with_minisign_cli(tmp_path):
    """Подпись этого модуля проверяет настоящий `minisign -V`, и наоборот (ключ -W, без пароля)."""
    msg = tmp_path / "release.yaml"
    msg.write_bytes(b"schema: 1\n")
    secret_text, public_text = minisign.generate_keypair()
    (tmp_path / "k.pub").write_text(public_text)
    (tmp_path / "release.yaml.minisig").write_text(
        minisign.sign(msg.read_bytes(), minisign.load_secret_key(secret_text), "release:t")
    )
    res = subprocess.run(["minisign", "-Vm", str(msg), "-p", str(tmp_path / "k.pub")], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr

    subprocess.run(
        ["minisign", "-G", "-W", "-f", "-p", str(tmp_path / "cli.pub"), "-s", str(tmp_path / "cli.key")],
        check=True,
        capture_output=True,
    )
    subprocess.run(
        ["minisign", "-S", "-s", str(tmp_path / "cli.key"), "-m", str(msg), "-t", "release:cli"],
        check=True,
        capture_output=True,
    )
    public = minisign.load_public_key((tmp_path / "cli.pub").read_text())
    assert minisign.verify(msg.read_bytes(), (tmp_path / "release.yaml.minisig").read_text(), public) == "release:cli"
    minisign.load_secret_key((tmp_path / "cli.key").read_text())


def test_release_validation(world):
    good = world.release
    assert rf.loads(rf.dumps(good)) == good

    bad = copy.deepcopy(good)
    bad["roles"]["main"]["images"]["nav2"] = "10.1.1.249:5000/krikz/rob_box:nav2-humble-dev"
    with pytest.raises(rf.ReleaseError, match="не закреплён по digest"):
        rf.dumps(bad)

    bad = copy.deepcopy(good)
    bad["release"] = "../../etc"
    with pytest.raises(rf.ReleaseError, match="имя релиза"):
        rf.dumps(bad)

    bad = copy.deepcopy(good)
    del bad["roles"]["sound"]
    with pytest.raises(rf.ReleaseError, match="roles"):
        rf.dumps(bad)


def test_check_bundle_rejects_substitution(world):
    rf.check_bundle(world.release, "vision", world.bundles["vision"])
    with pytest.raises(rf.ReleaseError, match="подмена"):
        rf.check_bundle(world.release, "vision", world.bundles["main"])


def test_artifact_manifest_layers(world):
    import json

    data = rf.dumps(world.release)
    manifest = json.loads(rf.artifact_manifest(data, b"sig", world.release))
    assert manifest["artifactType"] == rf.ARTIFACT_TYPE
    assert rf.find_layer(manifest, rf.MT_RELEASE)["digest"] == rf.oci_digest(data)
    assert (
        rf.find_layer(manifest, rf.MT_BUNDLE, "sound")["annotations"][rf.ANN_TITLE]
        == "bundle-sound-2026.11.09-1.tar.gz"
    )


def _tar_with(member: tarfile.TarInfo, data: bytes = b"") -> bytes:
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tf:
        tf.addfile(member, io.BytesIO(data) if data else None)
    return buf.getvalue()


@pytest.mark.parametrize(
    "member",
    [
        tarfile.TarInfo("../escape.txt"),
        tarfile.TarInfo("/etc/passwd"),
        (lambda t: (setattr(t, "type", tarfile.SYMTYPE), setattr(t, "linkname", "/etc"), t)[-1])(tarfile.TarInfo("ln")),
    ],
)
def test_extract_rejects_unsafe_members(tmp_path, member):
    archive = tmp_path / "b.tar.gz"
    archive.write_bytes(_tar_with(member))
    dest = tmp_path / "dest"
    dest.mkdir()
    with pytest.raises(StageError, match="бандла"):
        extract_bundle(archive, dest)
    assert not (tmp_path / "escape.txt").exists()


def test_extract_regular_bundle(tmp_path):
    archive = tmp_path / "b.tar.gz"
    archive.write_bytes(make_bundle({"docker/main/a.yaml": b"a: 1\n", "docker/main/x.sh": b"#!/bin/sh\n"}))
    dest = tmp_path / "d"
    dest.mkdir()
    assert extract_bundle(archive, dest) == len(b"a: 1\n") + len(b"#!/bin/sh\n")
    assert (dest / "docker/main/a.yaml").read_bytes() == b"a: 1\n"
