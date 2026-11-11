"""Один тик агента chuangong (фаза B): fetch → verify → [stage] → report. Переключений нет.

Тик запускает systemd-таймер раз в 5 минут. Режим `observe` (по умолчанию) пишет только в `state_dir`;
режим `stage` дополнительно кладёт бандл в `releases_root` и тянет образы роли по digest. `dry_run` не пишет
ничего: ни файлов, ни `docker pull`, ни публикации в zenoh — только печатает запись тика.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from . import minisign
from . import release_format as rf
from . import state as st
from .registry import Registry, RegistryError
from .stage import Docker, RoleStager, StageError
from .zenoh_rest import ZenohError

log = logging.getLogger("chuangong")

MAX_RELEASE_BYTES = 1 << 20


@dataclass
class Config:
    robot_id: str
    node: str
    channel: str = "dev"
    registry_url: str = "http://10.1.1.249:5000"
    pubkey: Path = Path("/etc/chuangong/release.pub")
    state_dir: Path = Path("/var/lib/chuangong")
    releases_root: Path = Path("/opt/rob_box/releases")
    docker_root: Path = Path("/var/lib/docker")
    zenoh_url: str = "http://127.0.0.1:8000"
    mode: st.Mode = st.Mode.OBSERVE
    dry_run: bool = False
    hil_lock: Path = Path("/tmp/hil.lock")
    reserve_bytes: int = 2 * 1024**3
    probe_hotpatch: bool = True

    def __post_init__(self) -> None:
        if self.node not in st.NODES:
            raise ValueError(f"node {self.node!r}: ожидалось одно из {st.NODES}")
        rf.check_channel(self.channel)

    @property
    def state_file(self) -> Path:
        return self.state_dir / "state.json"

    @property
    def metrics_file(self) -> Path:
        return self.state_dir / "metrics.prom"

    @property
    def dirty_flag(self) -> Path:
        # Флаг горячей заливки на хосте. Сегодня hotpatch.sh его не ставит (фаза C); агент уже читает.
        return self.state_dir / "dirty"


@dataclass
class Deps:
    registry: Registry
    docker: Docker
    zenoh: Any  # .put(key, value)
    disk_free: Callable[[Path], int] | None = None


class FetchError(RuntimeError):
    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail


def fetch_release(cfg: Config, registry: Registry) -> dict[str, Any] | None:
    """Указатель канала → release.yaml с проверенной подписью. None — канала в реестре нет."""
    try:
        _, manifest = registry.get_manifest(rf.RELEASE_REPO, cfg.channel)
    except RegistryError as exc:
        if "HTTP 404" in str(exc):
            return None
        raise FetchError("registry_unreachable", str(exc)) from exc
    try:
        rel_layer = rf.find_layer(manifest, rf.MT_RELEASE)
        sig_layer = rf.find_layer(manifest, rf.MT_SIGNATURE)
        data = registry.get_blob(rf.RELEASE_REPO, rel_layer["digest"], MAX_RELEASE_BYTES)
        sig = registry.get_blob(rf.RELEASE_REPO, sig_layer["digest"], 4096)
    except (rf.ReleaseError, RegistryError) as exc:
        raise FetchError("artifact_broken", str(exc)) from exc
    try:
        public = minisign.load_public_key(cfg.pubkey.read_text(encoding="utf-8"))
        minisign.verify(data, sig.decode("utf-8", "replace"), public)
    except OSError as exc:
        raise FetchError("no_pubkey", f"{cfg.pubkey}: {exc}") from exc
    except minisign.SignatureError as exc:
        raise FetchError("bad_signature", str(exc)) from exc
    try:
        release = rf.loads(data)
    except rf.ReleaseError as exc:
        raise FetchError("bad_release", str(exc)) from exc
    if release["channel"] != cfg.channel:
        raise FetchError("wrong_channel", f"указатель {cfg.channel} ведёт на релиз канала {release['channel']}")
    return release


def probe_flags(cfg: Config, docker: Docker, record: dict[str, Any]) -> None:
    """Входы будущих условий активации (фаза C): замок HIL и горячая заливка. Здесь — только в отчёт."""
    record["hil_locked"] = cfg.hil_lock.exists() if cfg.node == "main" else None
    sources = []
    if cfg.dirty_flag.exists():
        sources.append(str(cfg.dirty_flag))
    if cfg.probe_hotpatch:
        sources += [f"container:{name}" for name in docker.hotpatched_containers()]
    record["dirty"] = bool(sources)
    record["dirty_sources"] = sources
    record["legacy_running"] = docker.running_images()


def _stage(cfg: Config, deps: Deps, release: dict[str, Any], record: dict[str, Any]) -> None:
    stager = RoleStager(
        deps.registry, deps.docker, cfg.releases_root, cfg.docker_root, cfg.reserve_bytes, deps.disk_free
    )
    if cfg.dry_run:
        plan = stager.plan(release, cfg.node)
        record["stage"] = {"plan": plan}
        record["state"] = st.State.STAGED.value if _plan_done(plan) else st.State.AVAILABLE.value
        return
    try:
        result = stager.run(release, cfg.node)
    except StageError as exc:
        record["state"] = st.State.STAGE_ERROR.value
        record["reason"] = f"{exc.reason}: {exc.detail}"
        record["stage"] = exc.extra.get("progress")
        record["disk"] = exc.extra.get("disk")
        return
    record["stage"] = result["progress"]
    record["disk"] = result["disk"]
    record["staged"] = release["release"]
    record["state"] = st.State.STAGED.value


def _plan_done(plan: dict[str, Any]) -> bool:
    return plan["bundle_done"] and not plan["images_missing"]


def evaluate(cfg: Config, deps: Deps, prev: dict[str, Any] | None) -> dict[str, Any]:
    record = st.new_record(cfg.robot_id, cfg.node, cfg.channel, cfg.mode, prev)
    record["dry_run"] = cfg.dry_run
    probe_flags(cfg, deps.docker, record)
    try:
        release = fetch_release(cfg, deps.registry)
    except FetchError as exc:
        record["state"] = st.State.FETCH_ERROR.value
        record["reason"] = f"{exc.reason}: {exc.detail}"
        return record
    if release is None:
        record["state"] = st.State.NO_RELEASE.value
        record["reason"] = f"в реестре нет {rf.RELEASE_REPO}:{cfg.channel}"
        return record
    record["available"] = release["release"]
    record["release_git"] = release.get("git")
    if cfg.mode is st.Mode.STAGE:
        _stage(cfg, deps, release, record)
    elif record["staged"] == release["release"]:
        record["state"] = st.State.STAGED.value
    else:
        record["state"] = st.State.AVAILABLE.value
    return record


def report(cfg: Config, deps: Deps, record: dict[str, Any]) -> dict[str, Any]:
    if cfg.dry_run:
        return record
    st.save(cfg.state_file, record)
    cfg.metrics_file.write_text(st.metrics_text(record), encoding="utf-8")
    try:
        deps.zenoh.put(st.state_key(cfg.robot_id, cfg.node), record)
    except ZenohError as exc:
        # Отчёт в zenoh — не повод терять тик: state.json и метрика на месте, ошибку видно в журнале.
        log.warning("zenoh: %s", exc)
        record["report_error"] = str(exc)
        st.save(cfg.state_file, record)
    return record


def tick(cfg: Config, deps: Deps) -> dict[str, Any]:
    prev = st.load(cfg.state_file)
    record = evaluate(cfg, deps, prev)
    return report(cfg, deps, record)
