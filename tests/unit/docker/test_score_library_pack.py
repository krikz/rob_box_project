"""Ресурсный пак ``score-library`` (ADR-0154 §8 В7): сборщик на katana, lock-файл, хук-фетчер, запись манифеста, деплой.

Без сети: registry katana подменяется ``opener``-фейком (сборщик и хук принимают его параметром),
``apply_resource_pack.sh`` гоняется на фикстуре-манифесте с подменённым ``python3``. Партитуры — синтетические
(пишутся здесь же через music21, если он есть; иначе этот один тест пропускается).
Настоящие партитуры и архив библиотеки в git не лежат.
"""

from __future__ import annotations

import hashlib
import importlib.util
import io
import json
import os
import re
import shutil
import sqlite3
import subprocess
import tarfile
from pathlib import Path
from types import SimpleNamespace
from urllib.error import URLError

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
PACK_DIR = REPO_ROOT / "docker" / "vision" / "scripts" / "resource_pack"
APPLY = PACK_DIR / "apply_resource_pack.sh"
LOCK = PACK_DIR / "score_library.lock.json"
MANIFEST = PACK_DIR / "manifest.yaml"
BUILDER = REPO_ROOT / "scripts" / "music" / "build_score_library.py"
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "L-Deploy and Verify.yml"
BASH = shutil.which("bash")


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def fetcher():
    return _load(PACK_DIR / "fetch_score_library.py", "fetch_score_library_under_test")


@pytest.fixture(scope="module")
def builder():
    return _load(BUILDER, "build_score_library_under_test")


# ── синтетическая библиотека (без music21): индекс + JSON в раскладке score_import ──────────────────────────────

def make_library(lib: Path, ids=("pdmx:Qm1", "local:abcd1234"), licenses=("cc-zero", "private-local")) -> Path:
    lib.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(lib / "score_index.db"))
    conn.execute("CREATE TABLE score_index (material_id TEXT, title TEXT, composer TEXT, rating REAL, "
                 "n_ratings INTEGER, license TEXT, file TEXT)")
    for mid, lic in zip(ids, licenses):
        name = mid.replace(":", "_") + ".json"
        (lib / name).write_text(json.dumps({"material_id": mid}), encoding="utf-8")
        conn.execute("INSERT INTO score_index VALUES (?,?,?,?,?,?,?)", (mid, mid, "", 4.5, 3, lic, name))
    conn.commit()
    conn.close()
    return lib


def built(builder, tmp_path: Path, version: str = "2026.10.07.1", **kw):
    out = tmp_path / "out"
    out.mkdir(parents=True, exist_ok=True)
    lib = make_library(out / f"scores-{version}", **kw)
    manifest = builder.finish(lib, out, version)
    return out, manifest


def lock_for(builder, manifest) -> dict:
    return builder.lock_of(manifest, "http://registry.test:5000", "krikz/rob_box_resources")


class Blob:
    """Ответ registry: контекст-менеджер с ``read`` (как ``urlopen``)."""

    def __init__(self, data: bytes, headers=None):
        self._buf = io.BytesIO(data)
        self.headers = headers or {}

    def read(self, *args):
        return self._buf.read(*args)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def serving(data: bytes, calls: list | None = None):
    def opener(url):
        if calls is not None:
            calls.append(url)
        return Blob(data)
    return opener


# ── сборщик: упаковка, манифест, версия ─────────────────────────────────────────────────────────────────────────

def test_archive_is_flat_deterministic_and_described_by_its_manifest(builder, tmp_path):
    out, manifest = built(builder, tmp_path)
    archive = out / manifest["archive"]
    assert manifest["sha256"] == hashlib.sha256(archive.read_bytes()).hexdigest()
    assert manifest["size"] == archive.stat().st_size
    assert manifest["scores"] == 2
    assert manifest["licenses"] == {"cc-zero": 1, "private-local": 1}
    assert manifest["sources"] == {"local": 1, "pdmx": 1}
    with tarfile.open(archive) as tar:
        names = tar.getnames()
        assert all(m.mtime == 0 and m.isfile() for m in tar.getmembers())
    assert names == sorted(names) and "score_index.db" in names and "LIBRARY.json" in names
    assert all("/" not in n for n in names)
    again = tmp_path / "again.tar.gz"
    builder.pack(out / "scores-2026.10.07.1", again)
    assert again.read_bytes() == archive.read_bytes(), "та же библиотека — тот же sha256 архива"


def test_index_row_without_json_fails_the_build(builder, tmp_path):
    lib = make_library(tmp_path / "lib")
    (lib / "pdmx_Qm1.json").unlink()
    with pytest.raises(builder.BuildError, match="не сходится с индексом"):
        builder.summarize(lib, "2026.10.07.1")


def test_empty_library_is_an_error_not_an_empty_pack(builder, tmp_path):
    (tmp_path / "lib").mkdir()
    with pytest.raises(builder.BuildError, match="ни одна партитура"):
        builder.summarize(tmp_path / "lib", "2026.10.07.1")


@pytest.mark.parametrize("bad", ["20261007", "2026-10-07", "2026.10.07", "v1", "2026.10.07.1-abcdef0"])
def test_version_format_is_enforced(builder, bad):
    with pytest.raises(builder.BuildError):
        builder.check_version(bad)


def test_tag_survives_registry_rotation(builder):
    """cleanup_registry.sh --keep удаляет только теги «<префикс>-<hex 7..40>» — тег пака под это не попадает."""
    script = (REPO_ROOT / "docker" / "build" / "scripts" / "cleanup_registry.sh").read_text(encoding="utf-8")
    assert r'sha_re = re.compile(r"^(.*)-([0-9a-f]{7,40})$")' in script, "регулярка ротации поменялась — сверить тест"
    assert builder.check_version("2026.10.07.1")
    assert not builder.CLEANUP_SHA_RE.match(builder.tag_of("2026.10.07.1"))
    assert not builder.CLEANUP_SHA_RE.match(builder.tag_of("2027.01.01.12"))


# ── сборщик: публикация в registry (фейк) ───────────────────────────────────────────────────────────────────────

class FakeRegistry:
    """Минимум Docker Registry API v2: HEAD/GET блоба, POST+PUT загрузки, PUT манифеста."""

    def __init__(self):
        self.blobs: dict = {}
        self.manifests: dict = {}

    def __call__(self, req, timeout=None):
        url, method = req.full_url, req.get_method()
        path = url.split("://", 1)[1].split("/", 1)[1]
        if method == "POST" and path.endswith("/blobs/uploads/"):
            return Blob(b"", {"Location": "/v2/krikz/rob_box_resources/blobs/uploads/u1?_state=x"})
        if method == "PUT" and "/blobs/uploads/" in path:
            digest = re.search(r"digest=(sha256:[0-9a-f]{64})", url).group(1)
            assert "?_state=x&digest=" in url, url
            data = req.data
            assert "sha256:" + hashlib.sha256(data).hexdigest() == digest
            self.blobs[digest] = data
            return Blob(b"")
        if "/manifests/" in path and method == "PUT":
            self.manifests[path.rsplit("/", 1)[1]] = (req.headers.get("Content-type"), json.loads(req.data))
            return Blob(b"")
        if "/blobs/" in path:
            digest = path.rsplit("/", 1)[1]
            if digest not in self.blobs:
                raise URLError("404")
            return Blob(self.blobs[digest] if method == "GET" else b"")
        raise AssertionError(f"неожиданный запрос {method} {url}")


def test_publish_pushes_tagged_oci_artifact_and_writes_the_lock(builder, tmp_path):
    out, manifest = built(builder, tmp_path)
    reg = FakeRegistry()
    lock_out = tmp_path / "score_library.lock.json"
    args = SimpleNamespace(manifest=str(out / "scores-2026.10.07.1.manifest.json"), registry="http://localhost:5000",
                           robot_registry="http://10.1.1.249:5000", repository="krikz/rob_box_resources",
                           lock_out=str(lock_out))
    assert builder.publish(args, opener=reg) == 0
    layer = "sha256:" + manifest["sha256"]
    assert reg.blobs[layer] == (out / manifest["archive"]).read_bytes()
    media, body = reg.manifests["score-library-2026.10.07.1"]
    assert media == builder.MEDIA_MANIFEST
    assert body["layers"][0]["digest"] == layer and body["config"]["digest"] in reg.blobs
    lock = json.loads(lock_out.read_text(encoding="utf-8"))
    assert lock["registry"] == "http://10.1.1.249:5000" and lock["sha256"] == manifest["sha256"]
    assert lock["tag"] == "score-library-2026.10.07.1" and lock["scores"] == 2


def test_publish_refuses_an_archive_changed_after_build(builder, tmp_path):
    out, manifest = built(builder, tmp_path)
    (out / manifest["archive"]).write_bytes(b"other")
    args = SimpleNamespace(manifest=str(out / "scores-2026.10.07.1.manifest.json"), registry="http://r",
                           robot_registry="http://r", repository="x/y", lock_out=None)
    with pytest.raises(builder.BuildError, match="менялся после сборки"):
        builder.publish(args, opener=FakeRegistry())


def test_build_on_synthetic_scores_end_to_end(builder, fetcher, tmp_path):
    """Синтетические MusicXML → score_import → архив → хук ставит библиотеку, которую читает score_library."""
    pytest.importorskip("music21")
    from music21 import meter, note, stream

    src = tmp_path / "local"
    src.mkdir()
    for n, tune in enumerate((["C5", "D5", "E5", "G5"], ["A4", "C5", "E5", "D5"])):
        part = stream.Part()
        for b in range(8):
            m = stream.Measure(number=b + 1)
            if b == 0:
                m.append(meter.TimeSignature("4/4"))
            for p in tune if b % 2 == 0 else tune[::-1]:
                m.append(note.Note(p, quarterLength=1.0))
            part.append(m)
        score = stream.Score()
        score.insert(0, part)
        score.write("musicxml", fp=str(src / f"synthetic_{n}.musicxml"))
    args = builder.build_parser().parse_args(
        ["build", "--version", "2026.10.07.1", "--out", str(tmp_path / "out"), "--local", str(src),
         "--local-license", "CC0 (synthetic test)"])
    assert builder.build(args) == 0
    manifest = json.loads((tmp_path / "out" / "scores-2026.10.07.1.manifest.json").read_text(encoding="utf-8"))
    assert manifest["scores"] == 2 and manifest["licenses"] == {"CC0 (synthetic test)": 2}
    data = (tmp_path / "out" / manifest["archive"]).read_bytes()
    target = tmp_path / "opt" / "scores"
    fetcher.apply(lock_for(builder, manifest), target, opener=serving(data))
    conn = sqlite3.connect(str(target / "score_index.db"))
    rows = conn.execute("SELECT material_id, file FROM score_index").fetchall()
    conn.close()
    assert len(rows) == 2 and all((target / f).is_file() for _, f in rows)


def test_build_refuses_local_scores_without_license(builder, tmp_path):
    args = builder.build_parser().parse_args(
        ["build", "--version", "2026.10.07.1", "--out", str(tmp_path / "out"), "--local", str(tmp_path)])
    with pytest.raises(builder.BuildError, match="--local-license"):
        builder.build(args)


# ── хук-фетчер ──────────────────────────────────────────────────────────────────────────────────────────────────

def test_hook_installs_verifies_and_is_idempotent(builder, fetcher, tmp_path):
    out, manifest = built(builder, tmp_path)
    data = (out / manifest["archive"]).read_bytes()
    lock = lock_for(builder, manifest)
    target = tmp_path / "opt" / "scores"
    calls: list = []
    fetcher.apply(lock, target, opener=serving(data, calls))
    assert calls == [f"http://registry.test:5000/v2/krikz/rob_box_resources/blobs/sha256:{manifest['sha256']}"]
    assert sorted(p.name for p in target.iterdir()) == sorted(
        [".complete", "LIBRARY.json", "local_abcd1234.json", "pdmx_Qm1.json", "score_index.db"])
    assert (target / ".complete").read_text(encoding="utf-8").splitlines()[0] == manifest["sha256"]
    assert "no-op" in fetcher.apply(lock, target, opener=serving(b"", calls))
    assert len(calls) == 1, "версия на месте — сети нет"


def test_hook_refuses_archive_with_foreign_sha(builder, fetcher, tmp_path):
    out, manifest = built(builder, tmp_path)
    target = tmp_path / "opt" / "scores"
    with pytest.raises(fetcher.PackError, match="НЕ установлен"):
        fetcher.apply(lock_for(builder, manifest), target, opener=serving(b"tampered"), sleep=lambda s: None)
    assert [p.name for p in target.iterdir()] == [], "ничего не поставлено, временных файлов нет"


def test_hook_network_failure_retries_then_fails(builder, fetcher, tmp_path):
    _, manifest = built(builder, tmp_path)
    tries = []

    def down(url):
        tries.append(url)
        raise URLError("no route to host")

    with pytest.raises(fetcher.PackError, match="сеть"):
        fetcher.apply(lock_for(builder, manifest), tmp_path / "t", opener=down, sleep=lambda s: None)
    assert len(tries) == fetcher.RETRIES
    assert not (tmp_path / "t" / ".complete").exists()


def test_new_version_replaces_files_in_the_same_directory(builder, fetcher, tmp_path):
    """Каталог смонтирован в контейнер: его inode не меняется, лишние файлы прошлой версии уходят."""
    out1, m1 = built(builder, tmp_path / "v1")
    out2, m2 = built(builder, tmp_path / "v2", version="2026.10.08.1", ids=("pdmx:Qm2",), licenses=("cc-zero",))
    target = tmp_path / "opt" / "scores"
    fetcher.apply(lock_for(builder, m1), target, opener=serving((out1 / m1["archive"]).read_bytes()))
    inode = os.stat(target).st_ino
    fetcher.apply(lock_for(builder, m2), target, opener=serving((out2 / m2["archive"]).read_bytes()))
    assert os.stat(target).st_ino == inode
    assert sorted(p.name for p in target.iterdir()) == [".complete", "LIBRARY.json", "pdmx_Qm2.json", "score_index.db"]
    assert json.loads((target / "LIBRARY.json").read_text(encoding="utf-8"))["version"] == "2026.10.08.1"


def test_hook_rejects_nested_or_hidden_members(builder, fetcher, tmp_path):
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        info = tarfile.TarInfo("../evil.json")
        info.size = 2
        tar.addfile(info, io.BytesIO(b"{}"))
    data = buf.getvalue()
    lock = {"version": "2026.10.07.1", "registry": "http://r", "repository": "x/y", "scores": 1,
            "sha256": hashlib.sha256(data).hexdigest(), "size": len(data)}
    with pytest.raises(fetcher.PackError, match="недопустимый элемент"):
        fetcher.apply(lock, tmp_path / "t", opener=serving(data))
    assert not (tmp_path / "evil.json").exists()


def test_hook_main_returns_1_and_says_why(fetcher, tmp_path, capsys):
    lock = tmp_path / "lock.json"
    lock.write_text(json.dumps({"version": "x"}), encoding="utf-8")
    assert fetcher.main(["--target", str(tmp_path / "t"), "--lock", str(lock)]) == 1
    assert "нет поля" in capsys.readouterr().err


# ── lock-файл в git и запись манифеста ──────────────────────────────────────────────────────────────────────────

def test_committed_lock_is_well_formed(builder, fetcher):
    lock = fetcher.load_lock(LOCK)
    assert re.fullmatch(r"[0-9a-f]{64}", lock["sha256"])
    assert builder.check_version(lock["version"]) and lock["tag"] == builder.tag_of(lock["version"])
    assert lock["registry"] == builder.ROBOT_REGISTRY and lock["repository"] == builder.REPOSITORY
    assert lock["archive"] == f"scores-{lock['version']}.tar.gz"
    assert lock["size"] > 0 and lock["scores"] == sum(lock["licenses"].values()) == sum(lock["sources"].values())
    assert LOCK.read_text(encoding="utf-8").count('"sha256"') == 1, "apply_resource_pack.sh берёт первую sha256"


def test_no_scores_or_archives_ship_in_git():
    for pattern in ("*.mxl", "*.musicxml", "scores-*.tar.gz", "score_index.db"):
        hits = [p for p in (REPO_ROOT / "docker").rglob(pattern)] + [p for p in (REPO_ROOT / "scripts").rglob(pattern)]
        assert not hits, hits


def _manifest_entry() -> dict:
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    return next(r for r in data["resources"] if r["name"] == "score-library")


def test_manifest_entry_is_a_soft_hook_on_the_library_path():
    entry = _manifest_entry()
    assert entry["type"] == "open" and entry["fetch_hook"] == "fetch_score_library"
    assert entry["target"] == "/opt/rob_box/scores" and entry["required"] == "soft"
    assert entry["verify_file"] == ".complete" and entry["sha256"] == ""
    assert entry["size_bytes"] == json.loads(LOCK.read_text(encoding="utf-8"))["size"]
    lib = (REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "engine" / "score_library.py")
    assert 'DEFAULT_DIR = "/opt/rob_box/scores"' in lib.read_text(encoding="utf-8")


def test_apply_script_and_deploy_know_the_pack():
    script = APPLY.read_text(encoding="utf-8")
    assert "fetch_score_library)" in script and 'SCORE_LIBRARY_LOCK="score_library.lock.json"' in script
    assert "fetch_score_library.py" in script
    workflow = WORKFLOW.read_text(encoding="utf-8")
    assert "--only renardo-samples,dj-dave-samples,sonicpi-samples,muldjord-kit,score-library" in workflow
    restart = (REPO_ROOT / "docker" / "vision" / "update_and_restart.sh").read_text(encoding="utf-8")
    assert "score-library" in restart


# ── apply_resource_pack.sh: маркер версии ───────────────────────────────────────────────────────────────────────

def _run_apply(tmp_path: Path, marker_first_line: str | None, python_exit: int = 0):
    entry = {**_manifest_entry()}
    entry.pop("consumers", None)
    manifest = tmp_path / "m.yaml"
    manifest.write_text(yaml.safe_dump({"version": 1, "resources": [entry]}, allow_unicode=True, sort_keys=False),
                        encoding="utf-8")
    root = tmp_path / "opt"
    if marker_first_line is not None:
        (root / "scores").mkdir(parents=True)
        (root / "scores" / ".complete").write_text(marker_first_line + "\nversion x\n", encoding="utf-8")
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    (fake_bin / "python3").write_text(f"#!/bin/sh\necho \"fake hook $*\"\nexit {python_exit}\n", encoding="utf-8")
    (fake_bin / "python3").chmod(0o755)
    env = {**os.environ, "PATH": f"{fake_bin}{os.pathsep}{os.environ['PATH']}"}
    cmd = [BASH, str(APPLY), "--manifest", str(manifest), "--root", str(root), "--only", "score-library"]
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120, env=env)


@pytest.mark.skipif(BASH is None, reason="bash недоступен")
def test_marker_of_the_locked_version_is_a_noop(tmp_path):
    sha = json.loads(LOCK.read_text(encoding="utf-8"))["sha256"]
    result = _run_apply(tmp_path, sha)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "OK no-op" in result.stdout and "fake hook" not in result.stdout
    assert f"маркер = sha256 архива из lock-файла ({sha})" in result.stdout


@pytest.mark.skipif(BASH is None, reason="bash недоступен")
def test_marker_of_another_version_runs_the_hook(tmp_path):
    result = _run_apply(tmp_path, "0" * 64)
    assert "от другой версии" in result.stdout
    assert "fake hook" in result.stdout and "fetch_score_library.py" in result.stdout
    # фейковый хук маркер не обновил → это не «установлено», а объявленная деградация
    assert result.returncode == 0 and "не от версии lock-файла" in result.stderr


@pytest.mark.skipif(BASH is None, reason="bash недоступен")
def test_hook_failure_degrades_softly_with_rtttl_note(tmp_path):
    result = _run_apply(tmp_path, None, python_exit=1)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ДЕГРАДАЦИЯ" in result.stderr and "RTTTL" in result.stderr
