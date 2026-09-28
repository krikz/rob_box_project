#!/usr/bin/env python3
"""Локальная сборка docker-образов rob_box ТЕМ ЖЕ путём, что и CI.

Единый подход (run 36351952819 — разбор, почему он нужен):

    docker/build-manifest.yaml          ← что собирать (сервисы + базы)
      └─ scripts/ci/gen_build_matrix.py ← теги / build-args / хуки
           └─ scripts/build/buildx_build.sh ← как собирать (buildx, билдер,
                                               host-gateway, кеш, push)

CI (L-Build Base Images / Main Pi / Vision Pi / Single Service) идёт по этой
же цепочке через composite ./.github/actions/l-build-service, который только
переводит inputs в BUILD_* и зовёт buildx_build.sh. Этот скрипт делает то же
самое без GitHub Actions: те же данные, тот же движок, та же семантика
source_hash / submodule_sha / pre_build.

Примеры:

    scripts/build/build.py list                     # что вообще можно собрать
    scripts/build/build.py base all                 # 4 базы (pcl после ros2-zenoh)
    scripts/build/build.py base depthai
    scripts/build/build.py service oak-d            # один сервис (Pi ищется сам)
    scripts/build/build.py pi vision                # все сервисы Vision Pi
    scripts/build/build.py all                      # базы + оба Pi
    scripts/build/build.py service quest --dry-run  # показать BUILD_*, не собирать

Push: образы сервисов собираются FROM localhost:5000/... (базы), а
docker-container билдер берёт FROM из registry, а не из локального daemon.
Поэтому по умолчанию push в локальный registry включён, если он отвечает
(`--push` / `--no-push` — явно). Без registry: `docker run -d -p 5000:5000
--restart=always --name registry registry:2`.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
MANIFEST = REPO_ROOT / "docker" / "build-manifest.yaml"
GEN = REPO_ROOT / "scripts" / "ci" / "gen_build_matrix.py"
BUILDX = REPO_ROOT / "scripts" / "build" / "buildx_build.sh"
SOURCE_HASH = REPO_ROOT / "scripts" / "ci" / "source_hash.sh"
PRE_BUILD_HOOKS = {
    "fetch_hailort_wheel": REPO_ROOT / "scripts" / "ci" / "fetch_hailort_wheel.sh"
}
PIS = ("vision", "main")


def _gen(*args: str) -> object:
    out = subprocess.run(
        [sys.executable, str(GEN), "--manifest", str(MANIFEST), *args],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    return json.loads(out.stdout)


def base_entries(ros_distro: str | None) -> list[dict]:
    """Базы в порядке сборки: зависимости (pcl ← ros2-zenoh) — после целей."""
    extra = ["--ros-distro", ros_distro] if ros_distro else []
    bases = _gen("--mode", "base", *extra)
    ordered: list[dict] = []
    pending = {name: entries[0] for name, entries in bases.items()}
    while pending:
        ready = [
            n
            for n, e in pending.items()
            if all(d not in pending for d in e["depends_on"].split())
        ]
        if not ready:
            raise SystemExit(f"base_images: цикл в depends_on: {sorted(pending)}")
        for n in ready:
            ordered.append(pending.pop(n))
    return ordered


def pi_entries(pi: str, docker_tag: str, ros_distro: str | None) -> list[dict]:
    """Сервисы Pi: сначала цепочка (топологически, как named job'ы), потом matrix."""
    extra = ["--ros-distro", ros_distro] if ros_distro else []
    common = ["--pi", pi, "--docker-tag", docker_tag, *extra]
    chained = _gen(*common, "--mode", "chained")
    matrix = _gen(*common, "--mode", "matrix")
    return [e[0] for e in chained.values()] + list(matrix)


def registry_up(host: str) -> bool:
    try:
        with urllib.request.urlopen(
            f"http://{host}/v2/", timeout=2
        ):  # noqa: S310 — локальный registry
            return True
    except (urllib.error.URLError, OSError):
        return False


def build_env(entry: dict, args: argparse.Namespace, push: bool) -> dict[str, str]:
    build_args = [a for a in entry["build_args"].splitlines() if a]
    if entry.get("source_hash_arg"):
        digest = subprocess.run(
            ["bash", str(SOURCE_HASH), entry["source_hash_spec"]],
            check=True,
            capture_output=True,
            text=True,
            cwd=REPO_ROOT,
        ).stdout.strip()
        build_args.append(f"{entry['source_hash_arg']}={digest}")
    return {
        "BUILD_SERVICE_NAME": entry["name"],
        "BUILD_DOCKERFILE": entry["dockerfile"],
        "BUILD_CONTEXT": entry["build_context"],
        "BUILD_PLATFORM": args.platform,
        "BUILD_TAGS": entry["tags"],
        "BUILD_ARGS": "\n".join(build_args),
        "BUILD_SUBMODULE_SHA": entry.get("submodule_sha", ""),
        "BUILD_LOAD_ONLY": "false" if push else "true",
        "BUILD_LOCAL_REGISTRY": args.registry,
        "BUILD_CACHE": "false" if args.no_registry_cache else "true",
        "BUILD_NO_CACHE": "true" if args.no_cache else "false",
        "BUILD_PROGRESS": args.progress,
    }


def run_one(entry: dict, args: argparse.Namespace, push: bool) -> bool:
    env = build_env(entry, args, push)
    hook = entry.get("pre_build")
    print(f"\n━━━ {entry['name']} ━━━", flush=True)
    if args.dry_run:
        if hook:
            print(f"  pre_build: bash {PRE_BUILD_HOOKS[hook].relative_to(REPO_ROOT)}")
        for k, v in env.items():
            print(f"  {k}={shlex.quote(v)}")
        return True
    if hook:
        subprocess.run(["bash", str(PRE_BUILD_HOOKS[hook])], check=True, cwd=REPO_ROOT)
    rc = subprocess.run(
        ["bash", str(BUILDX)], env={**os.environ, **env}, cwd=REPO_ROOT
    ).returncode
    return rc == 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("\n\n", 1)[1],
    )
    p.add_argument("what", choices=["list", "base", "service", "pi", "all"])
    p.add_argument(
        "names", nargs="*", help="base: имя|all; service: имена; pi: vision|main"
    )
    p.add_argument(
        "--docker-tag",
        default="local",
        help="dev|test|latest|local (как в CI), по умолчанию local",
    )
    p.add_argument("--ros-distro", default=None)
    p.add_argument("--platform", default="linux/arm64")
    p.add_argument("--registry", default="localhost:5000")
    push = p.add_mutually_exclusive_group()
    push.add_argument("--push", dest="push", action="store_true", default=None)
    push.add_argument("--no-push", dest="push", action="store_false")
    p.add_argument(
        "--no-cache", action="store_true", help="buildx --no-cache (force rebuild)"
    )
    p.add_argument(
        "--no-registry-cache", action="store_true", help="без --cache-from/--cache-to"
    )
    p.add_argument("--progress", default="plain")
    p.add_argument(
        "--dry-run", action="store_true", help="показать BUILD_* и не собирать"
    )
    p.add_argument(
        "--keep-going", action="store_true", help="не останавливаться на первой ошибке"
    )
    args = p.parse_args(argv)

    bases = base_entries(args.ros_distro)
    by_pi = {pi: pi_entries(pi, args.docker_tag, args.ros_distro) for pi in PIS}

    if args.what == "list":
        print("base:    " + " ".join(e["name"] for e in bases))
        for pi, entries in by_pi.items():
            print(f"{pi + ':':8} " + " ".join(e["name"] for e in entries))
        return 0

    if args.what == "base":
        wanted = set(args.names or ["all"])
        plan = bases if "all" in wanted else [e for e in bases if e["name"] in wanted]
        unknown = wanted - {"all"} - {e["name"] for e in bases}
    elif args.what == "service":
        if not args.names:
            p.error("service: укажите имя сервиса (см. `build.py list`)")
        everything = [e for entries in by_pi.values() for e in entries]
        plan = [e for e in everything if e["name"] in set(args.names)]
        unknown = set(args.names) - {e["name"] for e in everything}
    elif args.what == "pi":
        unknown = set(args.names) - set(PIS)
        plan = [e for pi in args.names if pi in by_pi for e in by_pi[pi]]
        if not args.names:
            p.error("pi: укажите vision и/или main")
    else:  # all
        plan = bases + by_pi["vision"] + by_pi["main"]
        unknown = set()
    if unknown:
        p.error(f"неизвестные имена: {' '.join(sorted(unknown))} (см. `build.py list`)")

    do_push = args.push
    if do_push is None:
        do_push = registry_up(args.registry)
        print(
            f"registry {args.registry}: {'доступен → push включён' if do_push else 'не отвечает → только --load'}"
            " (--push/--no-push — явно)"
        )
    if not args.dry_run and not os.access(BUILDX, os.R_OK):
        raise SystemExit(f"не найден {BUILDX}")

    failed: list[str] = []
    built = 0
    for entry in plan:
        if run_one(entry, args, do_push):
            built += 1
            continue
        failed.append(entry["name"])
        if not args.keep_going:
            break

    print(
        f"\nИтог: {built}/{len(plan)} собрано"
        + (f", упали: {' '.join(failed)}" if failed else "")
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
