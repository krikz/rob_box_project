"""Guard для ".github/workflows/L-Build Base Images.yml" (run 36351952819).

Что сломалось: этот workflow был единственным L-Build, который жил на своём
inline `docker buildx build ... --add-host=host.docker.internal:host-gateway`.
Композит ./.github/actions/l-build-service делал `docker buildx use
robbox-<host>` (docker-container), это запоминалось в ~/.docker раннера, и
базы падали за 0 секунд: host-gateway is not supported by the
docker-container driver.

Что проверяем:
  * ни одного inline buildx/docker push в workflow — только композит
    (→ scripts/build/buildx_build.sh, тот же скрипт у локальной сборки);
  * composite вызывается после checkout;
  * состав баз = docker/build-manifest.yaml (base_images), а job'ы берут
    свой элемент по ИМЕНИ;
  * `gen_build_matrix.py --mode base` выдаёт ровно те теги/build-args, что
    были вписаны руками в workflow до переезда (коммит 3941f84).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "L-Build Base Images.yml"
MANIFEST = REPO_ROOT / "docker" / "build-manifest.yaml"
GEN = REPO_ROOT / "scripts" / "ci" / "gen_build_matrix.py"


def _gen():
    spec = importlib.util.spec_from_file_location("gen_build_matrix", GEN)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _workflow() -> dict:
    with WORKFLOW.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _build_jobs() -> dict:
    return {k: v for k, v in _workflow()["jobs"].items() if k.startswith("build-")}


def test_no_inline_buildx_in_base_images_workflow():
    text = WORKFLOW.read_text(encoding="utf-8")
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        assert "docker buildx build" not in stripped, line
        assert "docker push" not in stripped, line
        assert "host-gateway" not in stripped, line


def test_every_base_job_builds_through_the_composite_after_checkout():
    jobs = _build_jobs()
    assert jobs, "в workflow нет build-* job'ов"
    for name, job in jobs.items():
        uses = [s.get("uses", "") for s in job["steps"]]
        assert "./.github/actions/l-build-service" in uses, (name, uses)
        checkout = next(
            i for i, u in enumerate(uses) if u.startswith("actions/checkout")
        )
        composite = uses.index("./.github/actions/l-build-service")
        assert checkout < composite, f"{name}: composite до checkout"


def test_base_jobs_match_manifest_and_select_by_name():
    manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    images = manifest["base_images"]["images"]
    jobs = _build_jobs()
    assert sorted(jobs) == sorted(f"build-{n}" for n in images)
    for name in images:
        include = jobs[f"build-{name}"]["strategy"]["matrix"]["include"]
        assert f"['{name}']" in include, (name, include)
        for dep in images[name].get("depends_on") or []:
            assert f"build-{dep}" in jobs[f"build-{name}"]["needs"], (name, dep)


def test_generator_reproduces_pre_migration_tags_and_build_args():
    gen = _gen()
    bases = gen.base_image_entries(gen.load_manifest(MANIFEST))
    ghcr, local = "ghcr.io/krikz/rob_box_base", "localhost:5000/krikz/rob_box_base"
    apt = "APT_PROXY=http://host.docker.internal:3142"
    for name in ("ros2-zenoh", "rtabmap", "depthai", "pcl"):
        (entry,) = bases[name]
        assert entry["dockerfile"] == f"docker/base/Dockerfile.{name}"
        assert entry["build_context"] == "docker/base"
        assert entry["tags"].splitlines() == [
            f"{ghcr}:{name}-humble-latest",
            f"{local}:{name}-humble-latest",
            f"{local}:{name}-humble",
        ]
    assert bases["pcl"][0]["build_args"].splitlines() == [
        f"BASE_IMAGE={local}:ros2-zenoh-humble-latest",
        apt,
    ]
    for name in ("ros2-zenoh", "rtabmap", "depthai"):
        assert bases[name][0]["build_args"] == apt


def test_service_base_family_refs_point_at_built_base_tags():
    """Сервисы берут FROM <local>:<family>-<distro> — такой тег обязан
    собираться базовым job'ом, иначе сервис молча тянет старую базу."""
    gen = _gen()
    manifest = gen.load_manifest(MANIFEST)
    built = {
        tag
        for entries in gen.base_image_entries(manifest).values()
        for tag in entries[0]["tags"].splitlines()
    }
    for pi in manifest["pis"]:
        for svc in (manifest["pis"][pi].get("services") or {}).values():
            family = (svc.get("base") or {}).get("family")
            if family:
                assert (
                    f"localhost:5000/krikz/rob_box_base:{family}-humble" in built
                ), family
