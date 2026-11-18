"""Слот 6: офлайн-сценарии приёмки доставки (runbook docs/runbooks/delivery-acceptance.md).

Фикстуры World + существующий minisign.verify / ActivationMachine.
S2 (авто-путь) = отказ health; S3 (ручной) = rollback_planned. Стек не восстанавливают:
восстановление стека доказывает тест исполнителя, не эта фикстура.
Не execute compose, не робот, не biguan в коде. Пилот не утверждается.
"""

from __future__ import annotations

import json

import pytest
from chuangong import minisign
from chuangong import release_format as rf
from chuangong import state as st
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
        legacy_root=tmp_path / "legacy",
        docker_root=tmp_path,
        hil_lock=tmp_path / "hil.lock",
        reserve_bytes=0,
    )
    base.update(kw)
    return Config(**base)


def _deps(world, docker=None, zenoh=None, free=BIG):
    return Deps(world.registry, docker or FakeDocker(), zenoh or FakeZenoh(), disk_free=lambda p: free)


def _machine(cfg, deps, health=None):
    from chuangong.activation import ActivationMachine

    return ActivationMachine(cfg, deps, health=health)


def _stage(cfg, deps, release):
    from chuangong.stage import RoleStager

    return RoleStager(
        deps.registry, deps.docker, cfg.releases_root, cfg.docker_root, cfg.reserve_bytes, deps.disk_free
    ).run(release, cfg.node)


def _save_history(cfg, current, previous=None):
    rec = st.new_record(cfg.robot_id, cfg.node, cfg.channel, Mode.OBSERVE, None)
    rec["current"] = current
    rec["previous"] = previous
    st.save(cfg.state_file, rec)


def _files_under(path):
    return sorted(str(p.relative_to(path)) for p in path.rglob("*")) if path.exists() else []


class Health:
    def __init__(self, status="not_verified", detail="health not executed in dry-run"):
        self.status = status
        self.detail = detail

    def check(self, role, release, dry_run):
        from chuangong.activation import HealthResult

        return HealthResult(self.status, self.detail)


# --- S0 baseline ---


def test_baseline_observe_signed_release(tmp_path, world):
    """S0: подписанный релиз виден в observe; verify проходит тем же minisign."""
    data = rf.dumps(world.release)
    _, manifest = world.registry.get_manifest(rf.RELEASE_REPO, world.release["release"])
    sig_layer = rf.find_layer(manifest, rf.MT_SIGNATURE)
    sig_text = world.registry.get_blob(rf.RELEASE_REPO, sig_layer["digest"], 4096).decode("utf-8")
    public = minisign.load_public_key(world.pubkey.read_text(encoding="utf-8"))
    trusted = minisign.verify(data, sig_text, public)
    assert trusted.startswith("release:")

    rec = tick(_cfg(tmp_path, world), _deps(world))
    assert rec["state"] == "available"
    assert rec["available"] == world.release["release"]
    assert rec["reason"] is None
    assert rec["dirty"] is False


# --- S1 healthy vs S4 bad signature (контраст) ---


def test_healthy_release_activation_planned(tmp_path, world):
    """S1: исправный staged релиз → dry-run activation_planned, история не меняется."""
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE)
    docker = FakeDocker()
    deps = _deps(world, docker)

    staged = tick(cfg, deps)
    assert staged["state"] == "staged", staged.get("reason")
    assert staged["staged"] == world.release["release"]
    assert docker.pulled

    _save_history(cfg, "2026.11.08-1")
    rec = _machine(cfg, deps, Health()).activate(world.release["release"])

    assert rec["state"] == "activation_planned"
    assert rec["would"] == {"current": world.release["release"], "previous": "2026.11.08-1"}
    assert all(step["execute"] is False for step in rec["plan"])
    assert [step["role"] for step in rec["plan"]] == ["vision", "sound", "main"]
    saved = json.loads(cfg.state_file.read_text(encoding="utf-8"))
    assert saved["current"] == "2026.11.08-1"


def test_bad_signature_rejected(tmp_path, world):
    """S4: изменённый release.yaml при старой подписи — verify/fetch_error, stage пуст."""
    good = rf.dumps(world.release)
    public = minisign.load_public_key(world.pubkey.read_text(encoding="utf-8"))
    # Достаём подпись из реестра (слой signature).
    from chuangong import release_format as rf_mod

    _, manifest = world.registry.get_manifest(rf_mod.RELEASE_REPO, world.release["release"])
    sig_layer = rf_mod.find_layer(manifest, rf_mod.MT_SIGNATURE)
    sig_text = world.registry.get_blob(rf_mod.RELEASE_REPO, sig_layer["digest"], 4096).decode("utf-8")

    minisign.verify(good, sig_text, public)  # контроль: до порчи подпись сходится

    tampered = good.replace(b"abc1234", b"evil999")
    with pytest.raises(minisign.SignatureError):
        minisign.verify(tampered, sig_text, public)

    world.publish(world.release["release"], release_bytes=tampered)
    cfg = _cfg(tmp_path, world, mode=Mode.STAGE)
    docker = FakeDocker()
    rec = tick(cfg, _deps(world, docker))

    assert rec["state"] == "fetch_error"
    assert rec["reason"].startswith("bad_signature")
    assert docker.pulled == []
    assert _files_under(cfg.releases_root) == []


def test_healthy_passes_bad_signature_fails_contrast(tmp_path, world):
    """Явный контраст runbook §6: один API verify — PASS на целых байтах, FAIL на порче."""
    data = rf.dumps(world.release)
    secret_text, public_text = minisign.generate_keypair()
    secret = minisign.load_secret_key(secret_text)
    public = minisign.load_public_key(public_text)
    sig = minisign.sign(data, secret, f"release:{world.release['release']}")

    assert minisign.verify(data, sig, public) == f"release:{world.release['release']}"
    with pytest.raises(minisign.SignatureError, match="не сошлась"):
        minisign.verify(data + b"x", sig, public)
    with pytest.raises(minisign.SignatureError, match="не сошлась"):
        minisign.verify(data.replace(b"abc1234", b"evil999"), sig, public)


# --- S2 auto-rollback path: refusal only (health fail does not call rollback) ---


def test_failing_release_health_blocks(tmp_path, world):
    """S2 авто-откат (офлайн): unhealthy → blocked/health_check_failed, не rollback_planned.

    Критерий §1.4 до фазы C: отказ activate, авто-связь health→rollback не сработала.
    Восстановление стека доказывает тест исполнителя, не эта фикстура.
    """
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1", previous=None)
    pulled_after_stage = list(docker.pulled)

    rec = _machine(cfg, deps, Health("unhealthy", "vision-svc0: exited (127)")).activate(
        world.release["release"]
    )

    # Отказ — не план отката и не ручной rollback_planned (§2.1 / S3).
    assert rec["state"] == "blocked"
    assert rec["reason"] == "health_check_failed"
    assert rec["detail"] == "vision-svc0: exited (127)"
    assert rec["state"] != "rollback_planned"
    assert "plan" not in rec or rec.get("plan") in (None, [])
    assert "would" not in rec or rec.get("would") is None
    saved = json.loads(cfg.state_file.read_text(encoding="utf-8"))
    assert saved["current"] == "2026.11.08-1"
    assert saved.get("previous") in (None, "null", "")
    # Стек не восстановлен и не переключён: compose/pull после stage не трогали.
    assert docker.pulled == pulled_after_stage


# --- S3 manual rollback: plan only (operator), not auto, not stack restore ---


def test_manual_rollback_plan(tmp_path, world):
    """S3 ручной откат (офлайн): явный rollback() → rollback_planned, история не swap.

    Критерий §2.1 — другой, чем S2/§1.4: оператор вызывает rollback, не health-автомат.
    Восстановление стека доказывает тест исполнителя, не эта фикстура.
    """
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker()
    deps = _deps(world, docker)
    previous = world.release["release"]
    _stage(cfg, deps, world.release)
    world.publish("2026.11.10-1")
    _save_history(cfg, "2026.11.10-1", previous)
    pulled_after_stage = list(docker.pulled)

    rec = _machine(cfg, deps, Health()).rollback()

    assert rec["state"] == "rollback_planned", rec.get("detail")
    assert rec["reason"] != "health_check_failed"
    assert rec["would"] == {"current": previous, "previous": "2026.11.10-1"}
    assert rec["current"] == "2026.11.10-1" and rec["previous"] == previous
    assert all(step["execute"] is False for step in rec["plan"])
    assert [step["role"] for step in rec["plan"]] == ["vision", "sound", "main"]
    # История на диске не swap — только план.
    saved = json.loads(cfg.state_file.read_text(encoding="utf-8"))
    assert saved["current"] == "2026.11.10-1"
    assert saved["previous"] == previous
    assert docker.pulled == pulled_after_stage


# --- S5 HIL-lock ---


def test_hil_lock_blocks_activation(tmp_path, world):
    """S5: /tmp/hil.lock на main → blocked hil_locked."""
    cfg = _cfg(tmp_path, world, node="main")
    deps = _deps(world, FakeDocker())
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1")
    cfg.hil_lock.write_text("hil on", encoding="utf-8")

    rec = _machine(cfg, deps).activate(world.release["release"])

    assert rec["state"] == "blocked"
    assert rec["reason"] == "hil_locked"
    assert rec["hil_locked"] is True
    assert rec["detail"] == str(cfg.hil_lock)


# --- S6 hotpatch ---


def test_hotpatch_dirty_blocks_activation(tmp_path, world):
    """S6: контейнер с HOTPATCH → dirty → blocked."""
    cfg = _cfg(tmp_path, world)
    docker = FakeDocker(hotpatched=["nav2"])
    deps = _deps(world, docker)
    _stage(cfg, deps, world.release)
    _save_history(cfg, "2026.11.08-1")

    rec = _machine(cfg, deps).activate(world.release["release"])

    assert rec["state"] == "blocked"
    assert rec["reason"] == "dirty"
    assert rec["dirty"] is True
    assert rec["detail"] == ["container:nav2"]


# --- S7 single role failure ---


def test_single_role_fetch_failure(tmp_path, world):
    """S7: одна роль с битой подписью падает; другая на валидном артефакте — available."""
    # Роль vision: портим указатель канала.
    tampered = rf.dumps(world.release).replace(b"abc1234", b"evil999")
    world.publish(world.release["release"], release_bytes=tampered)

    vision_cfg = _cfg(tmp_path, world, node="vision", state_dir=tmp_path / "var" / "vision")
    vision_rec = tick(vision_cfg, _deps(world))
    assert vision_rec["state"] == "fetch_error"
    assert vision_rec["reason"].startswith("bad_signature")

    # Роль sound: отдельный World с валидной подписью (изолированный реестр).
    sound_root = tmp_path / "sound_world"
    sound_root.mkdir()
    from conftest import World

    sound_world = World(sound_root, release_name=world.release["release"])
    sound_cfg = _cfg(
        tmp_path,
        sound_world,
        node="sound",
        state_dir=tmp_path / "var" / "sound",
        releases_root=tmp_path / "opt" / "sound_releases",
    )
    sound_rec = tick(sound_cfg, _deps(sound_world))
    assert sound_rec["state"] == "available"
    assert sound_rec["available"] == world.release["release"]

    # Сводный status.verdict не ok, пока один узел в ERROR_STATES.
    from chuangong.status import verdict

    found = {
        "vision": vision_rec,
        "sound": sound_rec,
        "main": {"state": "available", "available": world.release["release"], "reason": None},
    }
    ok, problems = verdict(found, release=world.release["release"])
    assert ok is False
    assert any("vision" in p and "fetch_error" in p for p in problems)
