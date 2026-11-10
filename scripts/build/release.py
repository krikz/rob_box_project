#!/usr/bin/env python3
"""Релиз как артефакт (ADR-0168, фаза B; rbt_infra ADR-0003 §4.1): собрать, подписать, проверить, опубликовать.

    release.py build   --release 2026.11.09-1 --channel dev --image-tag dev-b704404 --docker-tag dev \
                       --resolve-registry http://localhost:5000 --out out/        # или --digests digests.json
    release.py sign    --dir out/ --key secret.key        # ключ — файл или env CHUANGONG_SIGNING_KEY
    release.py verify  --dir out/ --pubkey release.pub    # подпись + схема + sha256 бандлов
    release.py image-versions --dir out/ --role vision    # текст .image-versions.<tag> из релиза (stdout)
    release.py publish --dir out/ --registry http://localhost:5000 [--no-channel-tag]

Что в релизе (формат — host/chuangong/chuangong/release_format.py, единственный источник):
  * образы роли по digest — список сервисов берётся из docker/<роль>/docker-compose.yaml (то, что роль реально
    запускает), имя тега — из docker/build-manifest.yaml (`*_TAG` → SHA-тег, без поля → изменяемый тег канала);
  * бандл роли — tar.gz из ОТСЛЕЖИВАЕМЫХ git файлов `BUNDLE_PATHS[роль]`, детерминированный (mtime 0, uid 0,
    сортировка), без секретов (`.env.secrets` не отслеживается и ещё раз отсекается здесь);
  * манифест ресурсного пака (sha256), хостовые миграции, compat.min_previous.

Ничего не меняет в текущем деплое: `.image-versions.*` в репо не трогает. В L: Build All Services зовётся
последними шагами job'а summary с continue-on-error (ADR-0168 §5 п. 2). Подпись — minisign (решение Шифу 09.11).
"""

from __future__ import annotations

import argparse
import gzip
import importlib.util
import io
import json
import os
import re
import subprocess
import sys
import tarfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "host" / "chuangong"))

from chuangong import minisign  # noqa: E402
from chuangong import release_format as rf  # noqa: E402
from chuangong.registry import Registry  # noqa: E402

BUILD_MANIFEST = "docker/build-manifest.yaml"
RESOURCE_PACK_MANIFEST = "docker/vision/scripts/resource_pack/manifest.yaml"
DEFAULT_IMAGE_PREFIX = "10.1.1.249:5000/krikz/rob_box"

# Что едет в бандл роли: каталог роли и всё, на что её compose ссылается через ../ (гард в тестах сверяет).
BUNDLE_PATHS: dict[str, tuple[str, ...]] = {
    "main": ("docker/main", "docker/common", "host/main", "src/rob_box_description"),
    "vision": ("docker/vision", "docker/common", "host/vision", "scripts/setup/ensure_hailo_driver.sh"),
    # Скрипт ресурсного пака живёт в docker/vision, но им пользуется и Sound Pi (L: Deploy and Verify).
    "sound": ("docker/sound", "docker/common", "sound_pack", "migrations", "docker/vision/scripts/resource_pack"),
}
# Хостовые шаги, которые деплой сегодня делает до compose up (фаза C исполнит их из бандла).
MIGRATIONS: dict[str, list[str]] = {"main": [], "vision": ["scripts/setup/ensure_hailo_driver.sh"], "sound": []}
RESOURCE_PACK_ROLES = ("vision", "sound")
_SECRET_NAMES = re.compile(r"(^|/)\.env\.(secrets|local)$|\.secrets$|\.key$")
_OWN_IMAGE = re.compile(r"^\$\{SERVICE_IMAGE_PREFIX:-[^}]+\}:(?P<svc>[a-z0-9-]+?)-\$\{(ROS_DISTRO|IMAGE_TAG)")


class BuildError(RuntimeError):
    pass


def _load_gen_build_matrix():
    spec = importlib.util.spec_from_file_location(
        "gen_build_matrix", REPO_ROOT / "scripts" / "ci" / "gen_build_matrix.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


GBM = _load_gen_build_matrix()


# --- состав роли ----------------------------------------------------------------------------------------------


def compose_images(compose: dict[str, Any]) -> tuple[list[str], list[str]]:
    """(наши сервисы-образы, сторонние образы) из compose роли — все профили."""
    own, external = set(), set()
    for svc in (compose.get("services") or {}).values():
        image = (svc or {}).get("image")
        if not image:
            continue
        m = _OWN_IMAGE.match(image)
        if m:
            own.add(m.group("svc"))
        elif "${" in image:
            raise BuildError(f"образ {image!r}: не наш шаблон и не литерал — не знаю, как закрепить")
        else:
            external.add(image)
    return sorted(own), sorted(external)


def all_services(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for pi in manifest["pis"]:
        out.update(GBM.services_for_pi(manifest, pi))
    return out


def image_tag_for(name: str, spec: dict[str, Any], image_tag: str, docker_tag: str, ros_distro: str) -> str:
    """Тег, который compose возьмёт сегодня: `*_TAG` из .image-versions (SHA-тег) или голый тег канала."""
    tag = image_tag if spec.get("image_versions", False) is not False else docker_tag
    ctx = GBM.BuildContext("", "", "", ros_distro, "", tag)
    return GBM.tag_basename(name, spec, ctx)


# --- бандл ----------------------------------------------------------------------------------------------------


def tracked_files(repo_root: Path, paths: Iterable[str]) -> list[tuple[str, str, bool]]:
    """[(путь, sha блоба в индексе, исполняемый)] — только обычные файлы git (ни ссылок, ни сабмодулей)."""
    paths = list(paths)
    out = subprocess.run(
        ["git", "-C", str(repo_root), "ls-files", "-s", "-z", "--", *paths], capture_output=True, check=True
    ).stdout.decode()
    files = []
    for rec in filter(None, out.split("\0")):
        meta, path = rec.split("\t", 1)
        mode, blob, _stage = meta.split()
        if mode in ("100644", "100755"):
            files.append((path, blob, mode == "100755"))
    if not files:
        raise BuildError(f"в git нет файлов по путям {paths}")
    return sorted(files)


def git_blobs(repo_root: Path, shas: list[str]) -> dict[str, bytes]:
    """Содержимое блобов как в git (без autocrlf рабочей копии) — бандл одинаков на Windows и в CI."""
    proc = subprocess.run(
        ["git", "-C", str(repo_root), "cat-file", "--batch"],
        input="".join(f"{s}\n" for s in shas).encode(),
        capture_output=True,
        check=True,
    )
    data, pos, out = proc.stdout, 0, {}
    for sha in shas:
        nl = data.index(b"\n", pos)
        size = int(data[pos:nl].split()[2])
        out[sha] = data[nl + 1 : nl + 1 + size]
        pos = nl + 1 + size + 1
    return out


def build_bundle(files: Iterable[tuple[str, bytes, bool]]) -> bytes:
    """Детерминированный tar.gz: одинаковые файлы → одинаковые байты → одинаковый sha256."""
    buf = io.BytesIO()
    with gzip.GzipFile(fileobj=buf, mode="wb", mtime=0, compresslevel=9) as gz:
        with tarfile.open(fileobj=gz, mode="w", format=tarfile.PAX_FORMAT) as tf:
            for name, data, executable in sorted(files):
                if _SECRET_NAMES.search(name):
                    raise BuildError(f"{name}: похоже на секрет — в бандл не кладётся")
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = 0o755 if executable else 0o644
                info.mtime = 0
                info.uid = info.gid = 0
                info.uname = info.gname = ""
                tf.addfile(info, io.BytesIO(data))
    return buf.getvalue()


def role_bundle(repo_root: Path, role: str) -> bytes:
    files = tracked_files(repo_root, BUNDLE_PATHS[role])
    blobs = git_blobs(repo_root, sorted({blob for _, blob, _ in files}))
    return build_bundle((path, blobs[blob], exe) for path, blob, exe in files)


# --- релиз ----------------------------------------------------------------------------------------------------


def _role_entry(repo_root, role, release, services, resolve, args) -> tuple[dict[str, Any], bytes]:
    compose = yaml.safe_load((repo_root / "docker" / role / "docker-compose.yaml").read_text(encoding="utf-8"))
    own, external = compose_images(compose)
    images = {}
    for name in own:
        if name not in services:
            raise BuildError(f"{role}: сервис {name} есть в compose, но нет в {BUILD_MANIFEST}")
        tag = image_tag_for(name, services[name], args["image_tag"], args["docker_tag"], args["ros_distro"])
        images[name] = f"{args['image_prefix']}@{resolve(tag)}"
    bundle = role_bundle(repo_root, role)
    entry = {
        "bundle": {"file": rf.bundle_file_name(role, release), "sha256": rf.sha256_hex(bundle), "size": len(bundle)},
        "images": images,
        "external_images": external,
        "resource_pack": None,
        "migrations": list(MIGRATIONS[role]),
    }
    if role in RESOURCE_PACK_ROLES:
        data = git_blobs(repo_root, [f":{RESOURCE_PACK_MANIFEST}"])[f":{RESOURCE_PACK_MANIFEST}"]
        entry["resource_pack"] = {"manifest": RESOURCE_PACK_MANIFEST, "sha256": rf.sha256_hex(data)}
    return entry, bundle


def build_release(
    repo_root: Path,
    release: str,
    channel: str,
    git_sha: str,
    image_tag: str,
    docker_tag: str,
    resolve: Callable[[str], str],
    image_prefix: str = DEFAULT_IMAGE_PREFIX,
    min_previous: str | None = None,
    created: str | None = None,
) -> tuple[dict[str, Any], dict[str, bytes]]:
    """(release dict, {роль: байты бандла}). `resolve(тег) -> sha256:…` — digest образа `<repo>:<тег>`."""
    rf.check_release_name(release)
    manifest = GBM.load_manifest(repo_root / BUILD_MANIFEST)
    services = all_services(manifest)
    args = {
        "image_tag": image_tag,
        "docker_tag": docker_tag,
        "image_prefix": image_prefix,
        "ros_distro": manifest["defaults"]["ros_distro"],
    }
    roles, bundles = {}, {}
    for role in rf.ROLES:
        roles[role], bundles[role] = _role_entry(repo_root, role, release, services, resolve, args)
    doc = {
        "schema": rf.SCHEMA,
        "release": release,
        "channel": channel,
        "git": git_sha,
        "created": created or datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "image_tag": image_tag,
        "docker_tag": docker_tag,
        "roles": roles,
        "compat": {"min_previous": min_previous},
    }
    rf.validate(doc)
    return doc, bundles


def write_release_dir(out: Path, release: dict[str, Any], bundles: dict[str, bytes]) -> None:
    out.mkdir(parents=True, exist_ok=True)
    (out / rf.RELEASE_FILE).write_bytes(rf.dumps(release))
    for role, data in bundles.items():
        (out / release["roles"][role]["bundle"]["file"]).write_bytes(data)


def sign_release_dir(out: Path, secret_text: str) -> None:
    data = (out / rf.RELEASE_FILE).read_bytes()
    release = rf.loads(data)
    trusted = f"release:{release['release']} channel:{release['channel']} git:{release['git']}"
    sig = minisign.sign(data, minisign.load_secret_key(secret_text), trusted, "rob_box release signature")
    (out / rf.SIGNATURE_FILE).write_text(sig, encoding="utf-8")


def verify_release_dir(out: Path, public_text: str) -> dict[str, Any]:
    """Подпись, схема и sha256 каждого бандла. Любое расхождение — исключение (ReleaseError/SignatureError)."""
    data = (out / rf.RELEASE_FILE).read_bytes()
    minisign.verify(data, (out / rf.SIGNATURE_FILE).read_text(encoding="utf-8"), minisign.load_public_key(public_text))
    release = rf.loads(data)
    for role in rf.ROLES:
        rf.check_bundle(release, role, (out / release["roles"][role]["bundle"]["file"]).read_bytes())
    return release


def image_versions_text(release: dict[str, Any], role: str, manifest: dict[str, Any], pinned: bool = False) -> str:
    """Текст `.image-versions.<tag>` роли из релиза (переходный период: compose читает *_TAG как сегодня).

    pinned=True даёт `X_TAG=<тег>@sha256:…` — compose тогда тянет ровно образ релиза.
    """
    services = all_services(manifest)
    lines = [
        f"# {role} image versions — сгенерировано из релиза {release['release']} ({release['git']})",
        "# scripts/build/release.py image-versions; руками не править",
        "",
    ]
    for name, ref in sorted(release["roles"][role]["images"].items()):
        var = services[name].get("image_versions", False)
        if var is False:
            continue
        value = release["image_tag"] + ("@" + ref.split("@", 1)[1] if pinned else "")
        lines.append(f"{var}={value}")
    return "\n".join(lines) + "\n"


def publish_release_dir(out: Path, registry: Registry, channel_tag: bool = True) -> str:
    """Положить релиз в реестр (`RELEASE_REPO:<release>`) и, если надо, сдвинуть тег канала. Возвращает digest."""
    data = (out / rf.RELEASE_FILE).read_bytes()
    sig = (out / rf.SIGNATURE_FILE).read_bytes()
    release = rf.loads(data)
    blobs = [rf.EMPTY_CONFIG, data, sig] + [
        (out / release["roles"][r]["bundle"]["file"]).read_bytes() for r in rf.ROLES
    ]
    for blob in blobs:
        registry.push_blob(rf.RELEASE_REPO, blob)
    manifest = rf.artifact_manifest(data, sig, release)
    registry.put_manifest(rf.RELEASE_REPO, release["release"], manifest, rf.MT_MANIFEST)
    if channel_tag:
        registry.put_manifest(rf.RELEASE_REPO, release["channel"], manifest, rf.MT_MANIFEST)
    return rf.oci_digest(manifest)


# --- CLI ------------------------------------------------------------------------------------------------------


def _resolver(args: argparse.Namespace) -> Callable[[str], str]:
    if args.digests:
        table = json.loads(Path(args.digests).read_text(encoding="utf-8"))
        return lambda tag: table[tag]
    if not args.resolve_registry:
        raise BuildError("нужен --digests или --resolve-registry")
    reg = Registry(args.resolve_registry)
    repo = args.image_prefix.split("/", 1)[1]
    return lambda tag: reg.manifest_digest(repo, tag)


def _secret(args: argparse.Namespace) -> str:
    if args.key:
        return Path(args.key).read_text(encoding="utf-8")
    env = os.environ.get("CHUANGONG_SIGNING_KEY")
    if not env:
        raise BuildError("нет ключа: --key или CHUANGONG_SIGNING_KEY")
    return env


def _git_sha(repo_root: Path) -> str:
    return subprocess.run(
        ["git", "-C", str(repo_root), "rev-parse", "--short=7", "HEAD"], capture_output=True, text=True, check=True
    ).stdout.strip()


def cmd_build(args: argparse.Namespace) -> int:
    release, bundles = build_release(
        REPO_ROOT,
        args.release,
        args.channel,
        args.git or _git_sha(REPO_ROOT),
        args.image_tag,
        args.docker_tag,
        _resolver(args),
        image_prefix=args.image_prefix,
        min_previous=args.min_previous or None,
    )
    write_release_dir(Path(args.out), release, bundles)
    print(f"release {release['release']}: {args.out}/{rf.RELEASE_FILE}")
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    release = verify_release_dir(Path(args.dir), Path(args.pubkey).read_text(encoding="utf-8"))
    print(f"OK: {release['release']} ({release['channel']}, git {release['git']}) — подпись и бандлы сошлись")
    return 0


def cmd_image_versions(args: argparse.Namespace) -> int:
    release = rf.loads((Path(args.dir) / rf.RELEASE_FILE).read_bytes())
    manifest = GBM.load_manifest(REPO_ROOT / BUILD_MANIFEST)
    sys.stdout.write(image_versions_text(release, args.role, manifest, pinned=args.pinned))
    return 0


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="release.py", description=__doc__.split("\n\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--release", required=True)
    b.add_argument("--channel", default="dev")
    b.add_argument("--image-tag", required=True, help="SHA-тег сборки, например dev-b704404")
    b.add_argument("--docker-tag", default="dev", help="изменяемый тег канала для сервисов без *_TAG")
    b.add_argument("--image-prefix", default=DEFAULT_IMAGE_PREFIX, help="имя репозитория образов, как его видит робот")
    b.add_argument("--resolve-registry", default="", help="откуда брать digest (например http://localhost:5000)")
    b.add_argument("--digests", default="", help="JSON {тег: sha256:…} вместо запроса к реестру")
    b.add_argument("--git", default="")
    b.add_argument("--min-previous", default="")
    b.add_argument("--out", required=True)
    s = sub.add_parser("sign")
    s.add_argument("--dir", required=True)
    s.add_argument("--key", default="")
    v = sub.add_parser("verify")
    v.add_argument("--dir", required=True)
    v.add_argument("--pubkey", required=True)
    i = sub.add_parser("image-versions")
    i.add_argument("--dir", required=True)
    i.add_argument("--role", required=True, choices=rf.ROLES)
    i.add_argument("--pinned", action="store_true")
    u = sub.add_parser("publish")
    u.add_argument("--dir", required=True)
    u.add_argument("--registry", required=True)
    u.add_argument("--no-channel-tag", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.cmd == "build":
            return cmd_build(args)
        if args.cmd == "sign":
            sign_release_dir(Path(args.dir), _secret(args))
            return 0
        if args.cmd == "verify":
            return cmd_verify(args)
        if args.cmd == "image-versions":
            return cmd_image_versions(args)
        print(publish_release_dir(Path(args.dir), Registry(args.registry), channel_tag=not args.no_channel_tag))
        return 0
    except (BuildError, rf.ReleaseError, minisign.SignatureError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
