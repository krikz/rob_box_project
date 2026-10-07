#!/usr/bin/env python3
"""Сборка библиотеки партитур на katana → архив ресурсного пака ``score-library`` (ADR-0154 §3.2, §8 В7).

Два шага, оба на katana (решение Шифу по В7, 07.10: библиотека — ресурсный пак на хосте робота
``/opt/rob_box/scores``, источник — katana):

``build``
    Источники (PDMX ``.mxl`` по списку + локальные партитуры) → ``score_import.py`` (тот же разбор, без своей копии)
    → каталог библиотеки (``score_index.db`` + JSON, раскладка ``engine/score_library.py``) → ``LIBRARY.json``
    → детерминированный ``scores-<версия>.tar.gz`` и рядом ``scores-<версия>.manifest.json``
    (sha256 и размер архива, число партитур, лицензии, источники).

``publish``
    Архив → локальный docker registry katana (``10.1.1.249:5000``) как OCI-артефакт: блоб архива + конфиг
    (``LIBRARY.json``) + манифест с тегом ``score-library-<версия>``. Тег держит блоб от GC реестра. Блоб
    адресуется своим sha256 — робот берёт ``GET /v2/<repo>/blobs/sha256:<sha>`` (хук
    ``docker/vision/scripts/resource_pack/fetch_score_library.py``) и сверяет ту же сумму. Шаг печатает и пишет
    (``--lock-out``) lock-файл, который коммитится в git: ``score_library.lock.json``. Сам архив в git — НЕТ
    (ADR-0154 В1, ADR-0155 В1: партитуры и библиотека — только на роботе и katana).

    python3 scripts/music/build_score_library.py build --version 2026.10.07.1 --out ~/scores_build \\
        --pdmx-root ~/pdmx/pr5 --pdmx-list ~/pdmx/pr5_list.txt --pdmx-csv ~/pdmx/pr5/pdmx_subset.csv \\
        --local ~/scores_src/local --local-license "private-local (Shifu only, not PD)" --jobs 8
    python3 scripts/music/build_score_library.py publish --manifest ~/scores_build/scores-2026.10.07.1.manifest.json \\
        --lock-out docker/vision/scripts/resource_pack/score_library.lock.json

Зависимости: ``build`` — music21 и ``src/rob_box_music`` (как у ``score_import.py``; путь к пакету скрипт кладёт в
``sys.path`` сам); ``publish`` и упаковка — только stdlib.
"""

from __future__ import annotations

import argparse
import collections
import gzip
import hashlib
import io
import json
import pathlib
import re
import sqlite3
import sys
import tarfile
import time
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence
from urllib.parse import urljoin
from urllib.request import Request, urlopen

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent.parent
INDEX_FILE = "score_index.db"  # engine/score_library.py:INDEX_FILE
LIBRARY_FILE = "LIBRARY.json"
#: Версия — дата сборки и номер за день. Без «-<hex>» в конце: такой тег cleanup_registry.sh --keep счёл бы
#: SHA-версией образа и удалил бы при ротации (docker/build/scripts/cleanup_registry.sh, sha_re).
VERSION_RE = re.compile(r"\d{4}\.\d{2}\.\d{2}\.\d+")
CLEANUP_SHA_RE = re.compile(r"^(.*)-([0-9a-f]{7,40})$")  # копия sha_re из cleanup_registry.sh — для теста
REPOSITORY = "krikz/rob_box_resources"
ROBOT_REGISTRY = "http://10.1.1.249:5000"
PUSH_REGISTRY = "http://localhost:5000"
MEDIA_MANIFEST = "application/vnd.oci.image.manifest.v1+json"
MEDIA_CONFIG = "application/vnd.oci.image.config.v1+json"
MEDIA_LAYER = "application/vnd.oci.image.layer.v1.tar+gzip"


class BuildError(Exception):
    """Сборка не дала честной библиотеки — архив не пишется."""


def check_version(version: str) -> str:
    if not VERSION_RE.fullmatch(version) or CLEANUP_SHA_RE.match(tag_of(version)):
        raise BuildError(f"версия {version!r}: нужна ГГГГ.ММ.ДД.N (например 2026.10.07.1)")
    return version


def tag_of(version: str) -> str:
    return f"score-library-{version}"


def sha256_of(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


# ── импорт партитур (score_import.py, без своей копии разбора) ─────────────────────────────────────────────────

def _score_import():
    for p in (str(REPO / "src" / "rob_box_music"), str(HERE)):
        if p not in sys.path:
            sys.path.insert(0, p)
    import score_import  # noqa: E402 — music21 нужен только этому шагу

    return score_import


def pdmx_files(root: pathlib.Path, listing: pathlib.Path) -> List[pathlib.Path]:
    """Файлы PDMX по списку (строка — путь относительно ``root``); отсутствующий файл — ошибка, а не пропуск."""
    names = [ln.strip().lstrip("./") for ln in listing.read_text(encoding="utf-8").splitlines() if ln.strip()]
    files = [root / n for n in names]
    missing = [str(f) for f in files if not f.is_file()]
    if missing:
        raise BuildError(f"нет {len(missing)} файлов из {listing}: {missing[:5]}")
    return files


def import_scores(lib: pathlib.Path, *, pdmx: Sequence[pathlib.Path], pdmx_csv: Optional[str],
                  local: Optional[str], local_license: str, local_source: str, jobs: int) -> List[str]:
    """Два прохода ``score_import`` в один каталог: PDMX (метаданные из CSV) и локальные (лицензия из аргумента)."""
    si = _score_import()
    lines: List[str] = []
    passes = []
    if pdmx:
        passes.append(("PDMX", list(pdmx), si.load_pdmx_csv(pdmx_csv) if pdmx_csv else None, "", ""))
    if local:
        passes.append(("local", si.collect_files([local]), None, local_license, local_source))
    for label, files, csv_rows, lic, src in passes:
        t0 = time.time()
        results = si.run_batch(files, csv_rows, lic, src, max(1, jobs))
        written = si.store(results, lib, str(lib / INDEX_FILE))
        lines.append(f"── {label}: {len(files)} файлов\n" + si.report(results, time.time() - t0, max(1, jobs), written))
    return lines


# ── каталог → LIBRARY.json → архив ─────────────────────────────────────────────────────────────────────────────

def summarize(lib: pathlib.Path, version: str) -> Dict[str, Any]:
    """Сводка каталога по ``score_index``; каждая строка индекса обязана иметь свой JSON, лишних JSON нет."""
    index = lib / INDEX_FILE
    if not index.is_file():
        raise BuildError(f"нет {index}: ни одна партитура не принята")
    conn = sqlite3.connect(str(index))
    try:
        rows = conn.execute("SELECT material_id, license, file FROM score_index ORDER BY material_id").fetchall()
    finally:
        conn.close()
    if not rows:
        raise BuildError(f"{index}: score_index пуст")
    files = {r[2] for r in rows}
    missing = sorted(f for f in files if not (lib / f).is_file())
    extra = sorted(p.name for p in lib.glob("*.json") if p.name not in files and p.name != LIBRARY_FILE)
    if missing or extra:
        raise BuildError(f"каталог не сходится с индексом: нет JSON {missing[:5]}, JSON без строки индекса {extra[:5]}")
    return {"version": version, "scores": len(rows),
            "licenses": dict(sorted(collections.Counter(r[1] for r in rows).items())),
            "sources": dict(sorted(collections.Counter(r[0].split(":", 1)[0] for r in rows).items()))}


def pack(lib: pathlib.Path, archive: pathlib.Path) -> None:
    """Плоский детерминированный tar.gz: имена по алфавиту, mtime/uid/gid = 0, gzip без времени."""
    names = sorted(p.name for p in lib.iterdir() if p.is_file())
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w", format=tarfile.PAX_FORMAT) as tar:
        for name in names:
            data = (lib / name).read_bytes()
            info = tarfile.TarInfo(name)
            info.size, info.mode, info.mtime = len(data), 0o644, 0
            tar.addfile(info, io.BytesIO(data))
    with archive.open("wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as gz:
        gz.write(buf.getvalue())


def finish(lib: pathlib.Path, out: pathlib.Path, version: str) -> Dict[str, Any]:
    """``LIBRARY.json`` в каталог, архив и манифест рядом; возвращает манифест."""
    summary = summarize(lib, version)
    (lib / LIBRARY_FILE).write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    archive = out / f"scores-{version}.tar.gz"
    pack(lib, archive)
    manifest = {**summary, "archive": archive.name, "sha256": sha256_of(archive), "size": archive.stat().st_size}
    (out / f"scores-{version}.manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=1) + "\n",
                                                         encoding="utf-8")
    return manifest


def build(args: argparse.Namespace) -> int:
    version = check_version(args.version)
    out = pathlib.Path(args.out).expanduser()
    lib = out / f"scores-{version}"
    if lib.exists():
        raise BuildError(f"{lib} уже есть: версия собирается один раз, следующая — новым номером")
    if not (args.pdmx_list or args.local):
        raise BuildError("нет источников: --pdmx-list и/или --local")
    if args.local and not args.local_license.strip():
        raise BuildError("--local без --local-license: локальные партитуры без лицензии в индекс не попадут")
    lib.mkdir(parents=True)
    pdmx = pdmx_files(pathlib.Path(args.pdmx_root).expanduser(), pathlib.Path(args.pdmx_list).expanduser()) \
        if args.pdmx_list else []
    for line in import_scores(lib, pdmx=pdmx, pdmx_csv=args.pdmx_csv, local=args.local,
                              local_license=args.local_license, local_source=args.local_source, jobs=args.jobs):
        print(line, flush=True)
    manifest = finish(lib, out, version)
    print(json.dumps(manifest, ensure_ascii=False, indent=1))
    return 0


# ── публикация в registry katana ───────────────────────────────────────────────────────────────────────────────

Opener = Callable[..., Any]


def _request(opener: Opener, url: str, method: str, data: Any = None, headers: Optional[Mapping[str, str]] = None):
    return opener(Request(url, data=data, method=method, headers=dict(headers or {})), timeout=600)


def blob_exists(registry: str, repo: str, digest: str, opener: Opener = urlopen) -> bool:
    try:
        with _request(opener, f"{registry}/v2/{repo}/blobs/{digest}", "HEAD"):
            return True
    except Exception:  # noqa: BLE001 - 404 и любой отказ = «нет», загрузка решит сама
        return False


def push_blob(registry: str, repo: str, data: bytes, opener: Opener = urlopen) -> str:
    """Монолитная загрузка блоба (POST → PUT ?digest=); уже есть — без загрузки. Возвращает digest."""
    digest = "sha256:" + hashlib.sha256(data).hexdigest()
    if blob_exists(registry, repo, digest, opener):
        return digest
    with _request(opener, f"{registry}/v2/{repo}/blobs/uploads/", "POST", b"") as resp:
        location = resp.headers["Location"]
    location = urljoin(f"{registry}/", location)
    sep = "&" if "?" in location else "?"
    with _request(opener, f"{location}{sep}digest={digest}", "PUT", data,
                  {"Content-Type": "application/octet-stream", "Content-Length": str(len(data))}):
        pass
    return digest


def oci_manifest(config: bytes, layer_digest: str, layer_size: int, archive_name: str) -> bytes:
    return json.dumps({
        "schemaVersion": 2, "mediaType": MEDIA_MANIFEST,
        "config": {"mediaType": MEDIA_CONFIG, "digest": "sha256:" + hashlib.sha256(config).hexdigest(),
                   "size": len(config)},
        "layers": [{"mediaType": MEDIA_LAYER, "digest": layer_digest, "size": layer_size,
                    "annotations": {"org.opencontainers.image.title": archive_name}}],
    }, sort_keys=True).encode()


def lock_of(manifest: Mapping[str, Any], robot_registry: str, repo: str) -> Dict[str, Any]:
    """Lock-файл пака (в git): откуда взять архив, его sha256/размер и что внутри."""
    return {
        "_comment": [
            "Ресурсный пак score-library (ADR-0154 §8 В7): библиотека партитур на хост робота /opt/rob_box/scores.",
            "Архив собран на katana scripts/music/build_score_library.py build и положен publish в registry katana",
            "как OCI-артефакт (тег держит блоб от GC). Хук fetch_score_library.py берёт блоб по sha256 и сверяет его.",
            "Сам архив и партитуры в git НЕ лежат (ADR-0154 В1, ADR-0155 В1). Новая версия = новый lock-файл.",
        ],
        "title": "Библиотека партитур (ADR-0154)",
        "version": manifest["version"], "registry": robot_registry, "repository": repo,
        "tag": tag_of(manifest["version"]), "archive": manifest["archive"],
        "sha256": manifest["sha256"], "size": manifest["size"], "scores": manifest["scores"],
        "licenses": manifest["licenses"], "sources": manifest["sources"],
    }


def publish(args: argparse.Namespace, opener: Opener = urlopen) -> int:
    manifest_path = pathlib.Path(args.manifest).expanduser()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    archive = manifest_path.parent / manifest["archive"]
    data = archive.read_bytes()
    if hashlib.sha256(data).hexdigest() != manifest["sha256"] or len(data) != manifest["size"]:
        raise BuildError(f"{archive}: sha256/размер не сходятся с {manifest_path} — архив менялся после сборки")
    config = json.dumps({k: manifest[k] for k in ("version", "scores", "licenses", "sources")},
                        ensure_ascii=False, sort_keys=True).encode()
    reg, repo = args.registry.rstrip("/"), args.repository
    layer = push_blob(reg, repo, data, opener)
    push_blob(reg, repo, config, opener)
    body = oci_manifest(config, layer, len(data), manifest["archive"])
    with _request(opener, f"{reg}/v2/{repo}/manifests/{tag_of(manifest['version'])}", "PUT", body,
                  {"Content-Type": MEDIA_MANIFEST}):
        pass
    with _request(opener, f"{reg}/v2/{repo}/blobs/{layer}", "GET") as resp:  # обратная проверка: что отдаст роботу
        back = hashlib.sha256(resp.read()).hexdigest()
    if back != manifest["sha256"]:
        raise BuildError(f"registry вернул блоб с sha256 {back}, ждали {manifest['sha256']}")
    lock = lock_of(manifest, args.robot_registry.rstrip("/"), repo)
    text = json.dumps(lock, ensure_ascii=False, indent=1) + "\n"
    if args.lock_out:
        pathlib.Path(args.lock_out).write_text(text, encoding="utf-8")
    print(text)
    print(f"опубликовано: {reg}/v2/{repo}/blobs/{layer} ({len(data)} байт), тег {tag_of(manifest['version'])}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="источники → каталог библиотеки → архив + манифест")
    b.add_argument("--version", required=True, help="ГГГГ.ММ.ДД.N")
    b.add_argument("--out", required=True, help="каталог сборки (вне git)")
    b.add_argument("--pdmx-root", default=".", help="корень, от которого считаются пути --pdmx-list")
    b.add_argument("--pdmx-list", help="список .mxl PDMX (путь на строку)")
    b.add_argument("--pdmx-csv", help="PDMX.csv или его подмножество: лицензии/рейтинги, отбор license_conflict")
    b.add_argument("--local", help="каталог локальных партитур (не PDMX)")
    b.add_argument("--local-license", default="", help="лицензия локальных партитур (без неё — отказ)")
    b.add_argument("--local-source", default="", help="источник локальных партитур (атрибуция)")
    b.add_argument("--jobs", type=int, default=1)
    p = sub.add_parser("publish", help="архив → registry katana (OCI-артефакт) + lock-файл")
    p.add_argument("--manifest", required=True, help="scores-<версия>.manifest.json из build")
    p.add_argument("--registry", default=PUSH_REGISTRY, help="куда грузить (на katana — localhost:5000)")
    p.add_argument("--robot-registry", default=ROBOT_REGISTRY, help="адрес registry, видимый роботу (в lock)")
    p.add_argument("--repository", default=REPOSITORY)
    p.add_argument("--lock-out", help="записать lock-файл сюда (score_library.lock.json в git)")
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    args = build_parser().parse_args(argv)
    try:
        return build(args) if args.cmd == "build" else publish(args)
    except BuildError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
