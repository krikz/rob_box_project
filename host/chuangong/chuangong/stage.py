"""Stage роли: скачать заранее бандл и образы релиза, ничего не переключая (ADR-0168 §3).

Бандл: блоб из реестра в `<releases_root>/.partial/<файл>.part` (докачка после обрыва) → проверка sha256 по
подписанному release.yaml → распаковка во временный каталог → rename в `<releases_root>/<релиз>/<роль>` с
маркером `.chuangong-staged`. Образы: `docker pull <repo>@sha256:…` только тех, которых ещё нет.
Повторный запуск идемпотентен: готовый бандл и уже скачанные образы не трогаются.
"""

from __future__ import annotations

import shutil
import subprocess
import tarfile
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Protocol

from . import release_format as rf
from .registry import Registry, RegistryError

MARKER = ".chuangong-staged"
MAX_BUNDLE_UNPACKED = 512 * 1024 * 1024
# Сжатые слои в реестре → распакованный образ в /var/lib/docker: на наших образах 2–4× (docker images vs registry).
IMAGE_UNPACK_FACTOR = 3


class StageError(RuntimeError):
    def __init__(self, reason: str, detail: str, **extra: Any) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail
        self.extra = extra


class Docker(Protocol):
    def image_present(self, ref: str) -> bool: ...

    def pull(self, ref: str) -> None: ...

    def running_images(self) -> list[str]: ...

    def hotpatched_containers(self) -> list[str]: ...


class DockerCli:
    """docker CLI хоста. Только чтение и `docker pull`; никаких stop/rm/up."""

    def __init__(self, timeout_s: float = 1800.0) -> None:
        self.timeout_s = timeout_s

    def _run(self, *args: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
        try:
            return subprocess.run(
                ["docker", *args], capture_output=True, text=True, timeout=timeout, stdin=subprocess.DEVNULL
            )
        except (OSError, subprocess.TimeoutExpired) as exc:  # нет docker CLI, нет доступа, завис
            return subprocess.CompletedProcess(["docker", *args], 127, "", f"docker {args[0]}: {exc}")

    def image_present(self, ref: str) -> bool:
        return self._run("image", "inspect", "--format", "{{.Id}}", ref).returncode == 0

    def pull(self, ref: str) -> None:
        res = self._run("pull", "--quiet", ref, timeout=self.timeout_s)
        if res.returncode != 0:
            raise StageError("pull_failed", f"{ref}: {res.stderr.strip()[-300:]}")

    def running_images(self) -> list[str]:
        res = self._run("ps", "--format", "{{.Image}}")
        return sorted({ln.strip() for ln in res.stdout.splitlines() if ln.strip()}) if res.returncode == 0 else []

    def hotpatched_containers(self) -> list[str]:
        """Контейнеры с меткой горячей заливки /ws/HOTPATCH (её пишет scripts/robot_dev/hotpatch.sh push)."""
        res = self._run("ps", "--format", "{{.Names}}")
        names = [n.strip() for n in res.stdout.splitlines() if n.strip()] if res.returncode == 0 else []
        return [n for n in names if self._run("exec", n, "test", "-f", "/ws/HOTPATCH").returncode == 0]


def _repo_of(ref: str) -> tuple[str, str]:
    """`host:5000/krikz/rob_box@sha256:…` → ("krikz/rob_box", "sha256:…")."""
    name, digest = ref.split("@", 1)
    first, _, rest = name.partition("/")
    if rest and ("." in first or ":" in first or first == "localhost"):
        name = rest
    return name, digest


def _member_path(member: tarfile.TarInfo) -> PurePosixPath:
    p = PurePosixPath(member.name)
    if p.is_absolute() or ".." in p.parts or not (member.isfile() or member.isdir()):
        raise StageError("bundle_unsafe", f"элемент бандла {member.name!r} (абсолютный путь, «..» или не файл)")
    return p


def extract_bundle(archive: Path, dest: Path) -> int:
    """Распаковать бандл в пустой `dest`: только обычные файлы и каталоги, без ссылок и выхода наружу."""
    total = 0
    with tarfile.open(archive, "r:gz") as tf:
        for member in tf:
            rel = _member_path(member)
            target = dest.joinpath(*rel.parts)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            total += member.size
            if total > MAX_BUNDLE_UNPACKED:
                raise StageError("bundle_unsafe", f"распакованный бандл больше {MAX_BUNDLE_UNPACKED} байт")
            target.parent.mkdir(parents=True, exist_ok=True)
            src = tf.extractfile(member)
            with target.open("wb") as fh:
                shutil.copyfileobj(src, fh)
            target.chmod(0o755 if member.mode & 0o111 else 0o644)
    return total


def bundle_staged(dest: Path, sha256: str) -> bool:
    marker = dest / MARKER
    return marker.is_file() and marker.read_text(encoding="utf-8").strip() == sha256


def _images_need(registry: Registry, refs: list[str]) -> int:
    need = 0
    for ref in refs:
        repo, digest = _repo_of(ref)
        try:
            manifest = registry.platform_manifest(repo, digest)
        except RegistryError as exc:
            raise StageError("image_missing_in_registry", f"{ref}: {exc}") from exc
        need += sum(int(lay.get("size", 0)) for lay in manifest.get("layers") or []) * IMAGE_UNPACK_FACTOR
    return need


class RoleStager:
    def __init__(
        self,
        registry: Registry,
        docker: Docker,
        releases_root: Path,
        docker_root: Path,
        reserve_bytes: int,
        disk_free: Callable[[Path], int] | None = None,
    ) -> None:
        self.registry = registry
        self.docker = docker
        self.root = releases_root
        self.docker_root = docker_root
        self.reserve = reserve_bytes
        self.disk_free = disk_free or (lambda p: shutil.disk_usage(p).free)

    def plan(self, release: dict[str, Any], role: str) -> dict[str, Any]:
        """Что осталось скачать (без записи на диск)."""
        entry = release["roles"][role]
        dest = self.root / release["release"] / role
        missing = [ref for ref in entry["images"].values() if not self.docker.image_present(ref)]
        return {
            "dest": str(dest),
            "bundle_done": bundle_staged(dest, entry["bundle"]["sha256"]),
            "images_total": len(entry["images"]),
            "images_missing": missing,
        }

    def check_space(self, release: dict[str, Any], role: str, plan: dict[str, Any]) -> dict[str, int]:
        bundle_need = 0 if plan["bundle_done"] else release["roles"][role]["bundle"]["size"] * 3
        need = bundle_need + _images_need(self.registry, plan["images_missing"]) + self.reserve
        probe = self.root if self.root.exists() else self.root.parent
        free = min(self.disk_free(probe), self.disk_free(self.docker_root))
        disk = {"free_bytes": free, "need_bytes": need}
        if free < need:
            raise StageError("no_space", f"свободно {free} байт, нужно {need} (с запасом {self.reserve})", disk=disk)
        return disk

    def stage_bundle(self, release: dict[str, Any], role: str) -> None:
        bundle = release["roles"][role]["bundle"]
        dest = self.root / release["release"] / role
        partial = self.root / ".partial"
        part = partial / f"{bundle['file']}.part"
        try:
            self.registry.download_blob(rf.RELEASE_REPO, f"sha256:{bundle['sha256']}", part)
        except RegistryError as exc:
            raise StageError("bundle_download", str(exc)) from exc
        tmp = partial / f"{release['release']}-{role}.tmp"
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir(parents=True)
        extract_bundle(part, tmp)
        (tmp / MARKER).write_text(bundle["sha256"] + "\n", encoding="utf-8")
        if dest.exists():  # недоделанный прошлый stage этого же релиза — наш каталог, маркера в нём нет
            shutil.rmtree(dest)
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp.rename(dest)
        part.unlink(missing_ok=True)

    def pull_images(self, refs: list[str]) -> list[str]:
        failed = []
        for ref in refs:
            try:
                self.docker.pull(ref)
            except StageError as exc:
                failed.append(exc.detail)
        return failed

    def run(self, release: dict[str, Any], role: str) -> dict[str, Any]:
        plan = self.plan(release, role)
        disk = self.check_space(release, role, plan)
        if not plan["bundle_done"]:
            self.stage_bundle(release, role)
        failed = self.pull_images(plan["images_missing"])
        progress = {
            "bundle": "done",
            "images_total": plan["images_total"],
            "images_present": plan["images_total"] - len(failed),
            "dest": plan["dest"],
        }
        if failed:
            raise StageError("pull_failed", "; ".join(failed), progress=progress, disk=disk)
        return {"progress": progress, "disk": disk}
