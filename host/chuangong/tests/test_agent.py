"""Тик агента chuangong на фейках: observe, stage, битая подпись, нет места, обрыв и повтор, dry-run."""

from __future__ import annotations

import json
import os

from chuangong import minisign
from chuangong import release_format as rf
from chuangong.agent import Config, Deps, tick
from chuangong.state import Mode
from fakes import FakeDocker, FakeZenoh, digest_of

BIG = 1 << 40


def _cfg(tmp_path, world, **kw):
    base = dict(
        robot_id="RBXU100001",
        node="vision",
        pubkey=world.pubkey,
        state_dir=tmp_path / "var" / "chuangong",
        releases_root=tmp_path / "opt" / "releases",
        docker_root=tmp_path,
        hil_lock=tmp_path / "hil.lock",
        reserve_bytes=0,
    )
    base.update(kw)
    return Config(**base)


def _deps(world, docker=None, zenoh=None, free=BIG):
    return Deps(world.registry, docker or FakeDocker(), zenoh or FakeZenoh(), disk_free=lambda p: free)


def _files_under(path):
    return sorted(str(p.relative_to(path)) for p in path.rglob("*")) if path.exists() else []


def test_observe_reports_available_and_writes_only_state_dir(tmp_path, world):
    cfg = _cfg(tmp_path, world)
    zenoh, docker = FakeZenoh(), FakeDocker(running=["10.1.1.249:5000/krikz/rob_box:oak-d-humble-dev"])
    rec = tick(cfg, _deps(world, docker, zenoh))

    assert rec["state"] == "available"
    assert rec["available"] == "2026.11.09-1"
    assert rec["current"] is None and rec["staged"] is None
    assert rec["legacy_running"] == ["10.1.1.249:5000/krikz/rob_box:oak-d-humble-dev"]
    assert _files_under(tmp_path / "opt") == []
    assert docker.pulled == []
    assert sorted(_files_under(cfg.state_dir)) == [".lock", "metrics.prom", "state.json"]
    assert json.loads(cfg.state_file.read_text(encoding="utf-8"))["state"] == "available"
    assert zenoh.puts[0][0] == "robots/RBXU100001/ops/chuangong/state/vision"
    metrics = cfg.metrics_file.read_text(encoding="utf-8")
    assert 'rbx_release_info{robot_id="RBXU100001",node="vision",release="2026.11.09-1",state="available"} 1' in metrics


def test_stage_downloads_bundle_and_images_then_is_idempotent(tmp_path, world):
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE)
    docker = FakeDocker()
    rec = tick(cfg, _deps(world, docker))

    assert rec["state"] == "staged", rec["reason"]
    assert rec["staged"] == "2026.11.09-1"
    dest = cfg.releases_root / "2026.11.09-1" / "vision"
    assert (dest / "docker" / "vision" / "docker-compose.yaml").read_text() == "# vision 2026.11.09-1\n"
    if os.name == "posix":  # на Windows бита исполнения нет
        assert (dest / "docker" / "vision" / "run.sh").stat().st_mode & 0o100
    assert sorted(docker.pulled) == sorted(world.images["vision"].values())
    assert rec["stage"]["images_present"] == 2
    assert not (cfg.releases_root / ".partial" / "bundle-vision-2026.11.09-1.tar.gz.part").exists()

    bundle_digest = digest_of(world.bundles["vision"])
    gets_before = len(world.transport.blob_gets(bundle_digest))
    rec2 = tick(cfg, _deps(world, docker))
    assert rec2["state"] == "staged"
    assert len(world.transport.blob_gets(bundle_digest)) == gets_before, "готовый бандл не качается заново"
    assert len(docker.pulled) == 2, "скачанные образы не тянутся заново"


def test_bad_signature_is_fetch_error_and_nothing_staged(tmp_path, world):
    tampered = rf.dumps(world.release).replace(b"abc1234", b"evil999")
    world.publish("2026.11.09-1", release_bytes=tampered)
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE)
    docker = FakeDocker()
    rec = tick(cfg, _deps(world, docker))

    assert rec["state"] == "fetch_error"
    assert rec["reason"].startswith("bad_signature")
    assert docker.pulled == [] and _files_under(cfg.releases_root) == []


def test_signature_by_other_key_is_rejected(tmp_path, world):
    other_secret, _ = minisign.generate_keypair()
    data = rf.dumps(world.release_dict("2026.11.09-1"))
    world.publish("2026.11.09-1", signature=minisign.sign(data, minisign.load_secret_key(other_secret), "x"))
    rec = tick(_cfg(tmp_path, world), _deps(world))
    assert rec["state"] == "fetch_error"
    assert "другим ключом" in rec["reason"]


def test_no_disk_space_stops_before_any_download(tmp_path, world):
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE, reserve_bytes=2 * 1024**3)
    docker = FakeDocker()
    rec = tick(cfg, _deps(world, docker, free=1024**3))

    assert rec["state"] == "stage_error"
    assert rec["reason"].startswith("no_space")
    assert rec["disk"]["free_bytes"] == 1024**3 and rec["disk"]["need_bytes"] > 2 * 1024**3
    assert world.transport.blob_gets(digest_of(world.bundles["vision"])) == []
    assert docker.pulled == []


def test_interrupted_download_resumes_on_next_tick(tmp_path, world):
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE)
    bundle_digest = digest_of(world.bundles["vision"])
    world.transport.cut[bundle_digest] = 50
    docker = FakeDocker()

    rec = tick(cfg, _deps(world, docker))
    assert rec["state"] == "stage_error"
    assert rec["reason"].startswith("bundle_download")
    part = cfg.releases_root / ".partial" / "bundle-vision-2026.11.09-1.tar.gz.part"
    assert part.stat().st_size == 50
    assert not (cfg.releases_root / "2026.11.09-1").exists()

    rec2 = tick(cfg, _deps(world, docker))
    assert rec2["state"] == "staged", rec2["reason"]
    assert world.transport.blob_gets(bundle_digest)[-1].get("Range") == "bytes=50-"
    assert not part.exists()


def test_substituted_bundle_blob_is_refused(tmp_path, world):
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE)
    world.transport.blobs[digest_of(world.bundles["vision"])] = b"not the signed bundle"
    rec = tick(cfg, _deps(world))
    assert rec["state"] == "stage_error"
    assert "digest" in rec["reason"]
    assert not (cfg.releases_root / ".partial" / "bundle-vision-2026.11.09-1.tar.gz.part").exists()
    assert not (cfg.releases_root / "2026.11.09-1").exists()


def _republish_with_bundle(world, name: str, role: str, payload: bytes) -> None:
    """Опубликовать релиз заново, заменив бандл роли (sha256 честный — из подписанного release.yaml)."""
    import gzip

    blob = gzip.compress(payload, mtime=0)
    release = world.release_dict(name)
    release["roles"][role]["bundle"] = {
        "file": rf.bundle_file_name(role, name),
        "sha256": rf.sha256_hex(blob),
        "size": len(blob),
    }
    data = rf.dumps(release)
    sig = minisign.sign(data, world.secret, f"release:{name}")
    for r in rf.ROLES:
        bundle = blob if r == role else world.bundles[r]
        world.transport.blobs[digest_of(bundle)] = bundle
    world.transport.blobs[digest_of(data)] = data
    world.transport.blobs[digest_of(sig.encode())] = sig.encode()
    manifest = rf.artifact_manifest(data, sig.encode(), release)
    for ref in (name, release["channel"]):
        world.transport.manifests[(rf.RELEASE_REPO, ref)] = manifest
    world.release = release


def test_bundle_that_is_not_a_tar_is_stage_error_not_crash(tmp_path, world):
    _republish_with_bundle(world, "2026.11.09-1", "vision", b"not a tar at all")
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE)
    rec = tick(cfg, _deps(world, FakeDocker()))

    assert rec["state"] == "stage_error"
    assert rec["reason"].startswith("bundle_unpack"), rec["reason"]
    assert not (cfg.releases_root / "2026.11.09-1").exists()


def test_failed_pull_is_reported_and_retried(tmp_path, world):
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE)
    bad = world.images["vision"]["vision-svc1"]
    docker = FakeDocker(fail={bad})
    rec = tick(cfg, _deps(world, docker))
    assert rec["state"] == "stage_error" and rec["reason"].startswith("pull_failed")
    assert rec["stage"]["images_present"] == 1

    docker.fail.clear()
    rec2 = tick(cfg, _deps(world, docker))
    assert rec2["state"] == "staged"
    assert docker.pulled.count(bad) == 1


def test_dry_run_writes_nothing(tmp_path, world):
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE, dry_run=True)
    zenoh, docker = FakeZenoh(), FakeDocker()
    rec = tick(cfg, _deps(world, docker, zenoh))
    assert rec["state"] == "available" and rec["dry_run"] is True
    assert len(rec["stage"]["plan"]["images_missing"]) == 2
    assert _files_under(tmp_path / "var") == [] and _files_under(tmp_path / "opt") == []
    assert zenoh.puts == [] and docker.pulled == []


def test_no_channel_pointer_is_no_release(tmp_path, world):
    world.transport.manifests.clear()
    rec = tick(_cfg(tmp_path, world), _deps(world))
    assert rec["state"] == "no_release"


def test_hil_lock_and_hotpatch_are_reported(tmp_path, world):
    cfg = _cfg(tmp_path, world, node="main")
    cfg.hil_lock.write_text("hil on")
    cfg.state_dir.mkdir(parents=True)
    cfg.dirty_flag.write_text("hotpatch")
    rec = tick(cfg, _deps(world, FakeDocker(hotpatched=["nav2"])))
    assert rec["hil_locked"] is True
    assert rec["dirty"] is True
    assert rec["dirty_sources"] == [str(cfg.dirty_flag), "container:nav2"]
    metrics = cfg.metrics_file.read_text(encoding="utf-8")
    assert 'rbx_release_dirty{robot_id="RBXU100001",node="main"} 1' in metrics
    assert 'rbx_release_hil_locked{robot_id="RBXU100001",node="main"} 1' in metrics


def test_zenoh_down_keeps_state_file(tmp_path, world):
    from chuangong.zenoh_rest import ZenohError

    class DownZenoh:
        def put(self, key, value):
            raise ZenohError("connection refused")

    cfg = _cfg(tmp_path, world)
    rec = tick(cfg, _deps(world, zenoh=DownZenoh()))
    assert rec["state"] == "available"
    assert "connection refused" in json.loads(cfg.state_file.read_text(encoding="utf-8"))["report_error"]
