"""Stage роли: скачать заранее бандл и образы релиза, ничего не переключая (ADR-0168 §3).

Бандл: блоб из реестра в `<releases_root>/.partial/<файл>.part` (докачка после обрыва) → проверка sha256 по
подписанному release.yaml → распаковка во временный каталог → rename в `<releases_root>/<релиз>/<роль>` с
маркером `.chuangong-staged`. Образы: `docker pull <repo>@sha256:…` только тех, которых ещё нет.
Повторный запуск идемпотентен: готовый бандл и уже скачанные образы не трогаются.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import tarfile
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Protocol

import yaml

from . import release_format as rf
from .registry import Registry, RegistryError

MARKER = ".chuangong-staged"
MAX_BUNDLE_UNPACKED = 512 * 1024 * 1024
# Сжатые слои в реестре → распакованный образ в /var/lib/docker: на наших образах 2–4× (docker images vs registry).
IMAGE_UNPACK_FACTOR = 3
# digest в ref образа (как release_format / image_versions_text): только sha256:64hex.
_DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
_BUILD_MANIFEST_NAME = "build-manifest.yaml"


class StageError(RuntimeError):
    def __init__(self, reason: str, detail: str, **extra: Any) -> None:
        super().__init__(f"{reason}: {detail}")
        self.reason = reason
        self.detail = detail
        self.extra = extra


class Docker(Protocol):
    def image_present(self, ref: str) -> bool: ...

    def image_id(self, ref: str) -> str | None: ...

    def running_compose_image_ids(self) -> dict[str, list[str]]: ...

    def pull(self, ref: str) -> None: ...

    def running_images(self) -> list[str]: ...

    def hotpatched_containers(self) -> list[str]: ...


def _service_tag_vars(manifest: dict[str, Any]) -> dict[str, str]:
    """service → *_TAG из docker/build-manifest.yaml (image_versions != false).

    То же соответствие, что gen_build_matrix.image_versions_map / release.all_services:
    nav2 → NAV2_TAG. Сервисы с image_versions: false сюда не попадают.
    """
    out: dict[str, str] = {}
    for pi in (manifest.get("pis") or {}).values():
        for name, spec in ((pi or {}).get("services") or {}).items():
            iv = (spec or {}).get("image_versions", False)
            if iv is not False:
                out[str(name)] = str(iv)
    return out


def _load_service_tag_vars(manifest_path: Path) -> dict[str, str]:
    try:
        raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise StageError("compose_up_manifest", f"{manifest_path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise StageError("compose_up_manifest", f"{manifest_path}: ожидался mapping")
    return _service_tag_vars(raw)


def _resolve_build_manifest(project_dir: Path, manifest_path: Path | None) -> Path:
    """Манифест рядом с compose: ``docker/<role>`` → ``docker/build-manifest.yaml``."""
    if manifest_path is not None:
        path = Path(manifest_path)
        if not path.is_file():
            raise StageError("compose_up_manifest", f"нет файла {path}")
        return path
    candidate = Path(project_dir).resolve().parent / _BUILD_MANIFEST_NAME
    if candidate.is_file():
        return candidate
    raise StageError(
        "compose_up_manifest",
        f"нет {_BUILD_MANIFEST_NAME} рядом с {project_dir} (ожидался {candidate})",
    )


def _pinned_tag_value(service: str, ref: str, image_tag: str) -> str:
    """Значение *_TAG как image_versions_text(pinned=True): ``<image_tag>@sha256:…``.

    Digest — часть ref после ``@`` (тот же split, что в release.py). Тег сборки —
    ``image_tag`` релиза, не «dev»/«latest» из воздуха. Полный ref
    ``prefix:name-humble-TAG@sha256:…`` разбирается так же: в env идёт image_tag + digest.
    """
    if not isinstance(ref, str) or "@" not in ref:
        raise StageError("compose_up_bad_ref", f"{service}: нет @digest в {ref!r}")
    digest = ref.split("@", 1)[1]
    if not _DIGEST_RE.match(digest):
        raise StageError("compose_up_bad_ref", f"{service}: digest не sha256:64hex в {ref!r}")
    if not isinstance(image_tag, str) or not image_tag.strip():
        raise StageError("compose_up_unpinned", f"{service}: пустой image_tag, digests без тега сборки")
    return f"{image_tag.strip()}@{digest}"


def external_override(release: dict[str, Any], role: str) -> str:
    """Compose-override: каждый сторонний образ роли закреплён своим digest из релиза.

    stage подставляет digest только в переменные ``*_TAG`` наших сервисов, а сторонние
    образы (eclipse/zenoh и т.п.) в compose остаются плавающим тегом. Релиз их уже
    закрепил (``external_images``), и adopt отказывает, если они не закреплены, но в
    итоговый ``docker compose config`` digest сам не попадал. Override закрывает это:
    сервис, чей образ в compose равен ref до ``@``, получает образ с digest.
    """
    pinned_by_ref = {
        ref.split("@", 1)[0]: ref
        for ref in release["roles"][role]["external_images"]
        if "@" in ref
    }
    # Сервис без image_versions (supercollider) compose берёт голым тегом канала:
    # переменной *_TAG для него нет, поэтому digest из релиза подставляем здесь же.
    pinned_by_service = {
        name: ref for name, ref in release["roles"][role]["images"].items() if "@" in str(ref)
    }
    compose = yaml.safe_load(
        (Path("docker") / role / "docker-compose.yaml").read_text(encoding="utf-8")
    )
    services = {}
    matched = set()
    for name, svc in (compose.get("services") or {}).items():
        image = (svc or {}).get("image")
        if image in pinned_by_ref:
            services[name] = {"image": pinned_by_ref[image]}
            matched.add(image)
        elif name in pinned_by_service and "${" in str(image):
            services[name] = {"image": pinned_by_service[name]}
    missing = set(pinned_by_ref) - matched
    if missing:
        raise StageError(
            "external_override_unmatched",
            f"{role}: в compose нет сервиса для {sorted(missing)}",
        )
    return yaml.safe_dump({"services": services}, sort_keys=True)


def compose_digest_env(
    images: dict[str, str],
    *,
    image_tag: str,
    service_vars: dict[str, str],
) -> dict[str, str]:
    """service→digest-ref + манифест → env ``{NAV2_TAG: image_tag@sha256:…, …}``.

    Сервис без image_versions в манифесте в env не попадает (compose берёт IMAGE_TAG/docker_tag).
    """
    env: dict[str, str] = {}
    for service, ref in images.items():
        var = service_vars.get(str(service))
        if not var:
            continue
        env[var] = _pinned_tag_value(str(service), ref, image_tag)
    return env


class DockerCli:
    """docker CLI хоста. Только чтение, `docker pull` и compose up/down по digest-env."""

    def __init__(self, timeout_s: float = 1800.0, *, manifest_path: Path | None = None) -> None:
        self.timeout_s = timeout_s
        self.manifest_path = Path(manifest_path) if manifest_path is not None else None

    def _run(self, *args: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
        try:
            return subprocess.run(
                ["docker", *args], capture_output=True, text=True, timeout=timeout, stdin=subprocess.DEVNULL
            )
        except (OSError, subprocess.TimeoutExpired) as exc:  # нет docker CLI, нет доступа, завис
            return subprocess.CompletedProcess(["docker", *args], 127, "", f"docker {args[0]}: {exc}")

    def image_present(self, ref: str) -> bool:
        return self.image_id(ref) is not None

    def image_id(self, ref: str) -> str | None:
        res = self._run("image", "inspect", "--format", "{{.Id}}", ref)
        image_id = res.stdout.strip()
        return image_id if res.returncode == 0 and image_id else None

    def running_compose_image_ids(self) -> dict[str, list[str]]:
        res = self._run("ps", "--format", "{{.ID}}")
        if res.returncode != 0:
            error = res.stderr.strip() or res.stdout.strip() or f"docker ps exited {res.returncode}"
            raise StageError("compose_probe_failed", error, service="unknown", container="docker ps", error=error)
        services: dict[str, list[str]] = {}
        for container in (line.strip() for line in res.stdout.splitlines() if line.strip()):
            inspected = self._run(
                "inspect", "--format", '{{index .Config.Labels "com.docker.compose.service"}} {{.Image}}', container
            )
            if inspected.returncode != 0:
                error = (
                    inspected.stderr.strip()
                    or inspected.stdout.strip()
                    or f"docker inspect exited {inspected.returncode}"
                )
                raise StageError("compose_probe_failed", error, service="unknown", container=container, error=error)
            raw = inspected.stdout.strip()
            service, separator, image_id = raw.rpartition(" ")
            if not separator or not image_id:
                raise StageError(
                    "compose_probe_failed",
                    f"unexpected docker inspect output: {raw!r}",
                    service=service or "unknown",
                    container=container,
                    error=f"unexpected docker inspect output: {raw!r}",
                )
            if service != "<no value>":
                services.setdefault(service, []).append(image_id)
        return {service: sorted(ids) for service, ids in services.items()}

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
        if res.returncode != 0:
            error = res.stderr.strip() or res.stdout.strip() or f"docker ps exited {res.returncode}"
            raise StageError("hotpatch_probe_failed", error)
        names = [n.strip() for n in res.stdout.splitlines() if n.strip()]
        hotpatched = []
        for name in names:
            probe = self._run("exec", name, "test", "-f", "/ws/HOTPATCH")
            if probe.returncode == 0:
                hotpatched.append(name)
            elif probe.returncode != 1 or probe.stderr.strip():
                error = probe.stderr.strip() or probe.stdout.strip() or f"docker exec exited {probe.returncode}"
                raise StageError("hotpatch_probe_failed", f"{name}: {error}")
        return hotpatched

    def _compose(
        self, project_dir: Path, *args: str, env: dict[str, str] | None = None
    ) -> subprocess.CompletedProcess:
        run_env = None if env is None else {**os.environ, **env}
        try:
            return subprocess.run(
                ["docker", "compose", *args],
                cwd=str(project_dir),
                capture_output=True,
                text=True,
                timeout=self.timeout_s,
                stdin=subprocess.DEVNULL,
                env=run_env,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return subprocess.CompletedProcess(["docker", "compose", *args], 127, "", f"docker compose: {exc}")

    def compose_down(self, project_dir: Path) -> None:
        """``docker compose down`` в каталоге роли. Тесты подставляют FakeDocker."""
        res = self._compose(project_dir, "down", "--remove-orphans", "--timeout", "20")
        if res.returncode != 0:
            error = res.stderr.strip() or res.stdout.strip() or f"compose down exited {res.returncode}"
            raise StageError("compose_down_failed", error, dir=str(project_dir))

    def compose_up(
        self,
        project_dir: Path,
        images: dict[str, str] | None = None,
        *,
        image_tag: str | None = None,
        manifest_path: Path | None = None,
    ) -> None:
        """``docker compose up -d`` с digest-env из ``images`` (service → ref@sha256).

        Env — ``{NAV2_TAG: <image_tag>@sha256:…, …}`` как ``image_versions_text(pinned=True)``:
        имя переменной из ``docker/build-manifest.yaml`` (image_versions), значение —
        тег сборки релиза + digest из ref. Пустой ``images``/``None`` — отказ (не dev/latest).
        """
        pinned = dict(images or {})
        if not pinned:
            raise StageError(
                "compose_up_unpinned",
                "images пуст: compose up без digest-pin (dev/latest не подставляем)",
                dir=str(project_dir),
                images={},
            )
        if not isinstance(image_tag, str) or not image_tag.strip():
            raise StageError(
                "compose_up_unpinned",
                "нужен image_tag релиза для *_TAG=<image_tag>@sha256 (как image_versions_text pinned)",
                dir=str(project_dir),
                images=pinned,
            )
        manifest = _resolve_build_manifest(project_dir, manifest_path or self.manifest_path)
        service_vars = _load_service_tag_vars(manifest)
        env = compose_digest_env(pinned, image_tag=image_tag.strip(), service_vars=service_vars)
        if not env:
            raise StageError(
                "compose_up_unpinned",
                "ни один сервис images не имеет image_versions в манифесте — pin пуст",
                dir=str(project_dir),
                images=pinned,
            )
        res = self._compose(project_dir, "up", "-d", "--force-recreate", "--pull", "never", env=env)
        if res.returncode != 0:
            error = res.stderr.strip() or res.stdout.strip() or f"compose up exited {res.returncode}"
            raise StageError(
                "compose_up_failed",
                error,
                dir=str(project_dir),
                images=pinned,
                env=dict(env),
            )

    def check_container_status(self, container: str) -> str | None:
        """None = контейнер running, иначе деталь. Тот же контракт, что у FakeDocker."""
        res = self._run("inspect", "--format", "{{.State.Status}}", container)
        if res.returncode != 0:
            return res.stderr.strip() or res.stdout.strip() or f"docker inspect exited {res.returncode}"
        status = res.stdout.strip()
        if status != "running":
            return f"container {container} status {status or 'unknown'}, expected running"
        return None

    def check_container_logs(self, container: str) -> str | None:
        """None = логи читаются, иначе деталь. Содержимое не толкует — это проверка доступности."""
        res = self._run("logs", "--tail", "50", container)
        if res.returncode != 0:
            return res.stderr.strip() or f"docker logs exited {res.returncode}"
        return None

    def check_ros2_topics(self, expected_topics, container: str | None = None) -> str | None:
        """Сверка expected_topics с ros2 topic list внутри контейнера роли.

        None = все на месте. Нет контейнера, нет ros2 или темы не нашлось — деталь,
        не исключение: execute_role отличает сбой проверки от падения адаптера.
        """
        expected = list(expected_topics)
        if not container:
            return "check_ros2_topics: не задан контейнер роли"
        res = self._run("exec", container, "ros2", "topic", "list", timeout=60.0)
        if res.returncode != 0:
            return res.stderr.strip() or res.stdout.strip() or f"ros2 topic list exited {res.returncode}"
        present = {line.strip() for line in res.stdout.splitlines() if line.strip()}
        missing = [topic for topic in expected if topic not in present]
        if missing:
            return f"missing topics: {missing}"
        return None


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
    """Распаковать бандл в пустой `dest`: только обычные файлы и каталоги, без ссылок и выхода наружу.

    Битый gzip/tar — StageError("bundle_unpack"), а не traceback: тик остаётся штатным stage_error."""
    total = 0
    try:
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
                if src is None:
                    raise StageError("bundle_unpack", f"элемент бандла {member.name!r} не читается")
                with target.open("wb") as fh:
                    shutil.copyfileobj(src, fh)
                target.chmod(0o755 if member.mode & 0o111 else 0o644)
    except tarfile.TarError as exc:
        raise StageError("bundle_unpack", f"бандл не распаковывается: {exc}") from exc
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
