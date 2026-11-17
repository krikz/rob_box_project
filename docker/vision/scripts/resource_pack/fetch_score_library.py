"""Хук-фетчер Ресурсного пака: библиотека партитур на ХОСТ (запись ``score-library``, ADR-0154 §8 В7).

Источник — katana: архив ``scores-<версия>.tar.gz`` собирает ``scripts/music/build_score_library.py build`` и кладёт
``publish`` в локальный docker registry katana (``10.1.1.249:5000``) как OCI-артефакт. Робот уже тянет образы из этого
registry, так что новый сервис, ключ или пароль не нужен: хук берёт блоб ``GET /v2/<repo>/blobs/sha256:<sha>``
обычным HTTP.

Эталон — ``score_library.lock.json`` рядом (в git): версия, registry, repository, sha256 и размер архива, число
партитур. Архив с чужой суммой НЕ распаковывается никогда (ADR-0018). В git сам архив и партитуры не лежат
(ADR-0154 В1, ADR-0155 В1).

Установка — в тот же каталог ``--target`` (``/opt/rob_box/scores``), без подмены самого каталога: он bind-mount'ом
смонтирован в ``voice-assistant``, и переименованный каталог контейнер бы уже не увидел. Файлы новой версии
распаковываются во временный подкаталог и встают на место ``os.replace`` по одному (индекс последним), лишние файлы
прошлой версии удаляются. Маркер ``.complete`` = sha256 архива + версия, пишется ТОЛЬКО после установки; по нему
``apply_resource_pack.sh`` отличает «стоит эта версия» от «стоит прошлая».

Зависимости: только stdlib.  Exit codes: 0 — библиотека этой версии на месте; 1 — нет.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import sys
import tarfile
import time
from typing import Any, Callable, Dict, List, Optional, Sequence
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

MARKER_FILENAME = ".complete"
INDEX_FILE = "score_index.db"
LIBRARY_FILE = "LIBRARY.json"
STAGING = ".staging"
DOWNLOAD = ".download.part"
USER_AGENT = "rob-box-score-library/1.0"
TIMEOUT_SECONDS = 120
RETRIES = 4

Lock = Dict[str, Any]
Opener = Callable[[str], Any]


class PackError(Exception):
    """Библиотека этой версии не встала: текст — причина для лога деплоя."""


def load_lock(path: pathlib.Path) -> Lock:
    lock = json.loads(path.read_text(encoding="utf-8"))
    for key in ("version", "registry", "repository", "sha256", "size", "scores"):
        if key not in lock:
            raise PackError(f"{path}: в lock-файле нет поля {key!r}")
    return lock


def blob_url(lock: Lock) -> str:
    return f"{str(lock['registry']).rstrip('/')}/v2/{lock['repository']}/blobs/sha256:{lock['sha256']}"


def marker_text(lock: Lock) -> str:
    """Первая строка — sha256 архива: её же сверяет ``apply_resource_pack.sh`` с lock-файлом."""
    return f"{lock['sha256']}\nversion {lock['version']}\nscores {lock['scores']}\n"


def is_installed(lock: Lock, target: pathlib.Path) -> bool:
    marker = target / MARKER_FILENAME
    if not marker.is_file() or not (target / INDEX_FILE).is_file():
        return False
    lines = marker.read_text(encoding="utf-8", errors="replace").splitlines()
    return bool(lines) and lines[0].strip() == lock["sha256"]


def sha256_of_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _default_open(url: str):
    return urlopen(Request(url, headers={"User-Agent": USER_AGENT}), timeout=TIMEOUT_SECONDS)


def download(lock: Lock, dest: pathlib.Path, opener: Opener = _default_open,
             sleep: Callable[[float], None] = time.sleep) -> None:
    """Скачать архив в ``dest`` и сверить размер и sha256; не сошлось — файла нет, ``PackError``."""
    url = blob_url(lock)
    reason = "не пробовали"
    for attempt in range(1, RETRIES + 1):
        try:
            with opener(url) as response, dest.open("wb") as handle:
                shutil.copyfileobj(response, handle)
        except (HTTPError, URLError, TimeoutError, ConnectionError, OSError) as exc:
            reason = f"{url}: {exc}"
            dest.unlink(missing_ok=True)
            if attempt < RETRIES:
                sleep(min(attempt, 5))
            continue
        size, actual = dest.stat().st_size, sha256_of_file(dest)
        if size != lock["size"] or actual != lock["sha256"]:
            # Не ретраим: блоб адресован своей суммой, другой контент — не сбой сети.
            dest.unlink(missing_ok=True)
            raise PackError(f"sha256/размер не совпали (ждали {lock['sha256']} {lock['size']} байт, "
                            f"пришло {actual} {size} байт) — архив НЕ установлен")
        return
    raise PackError(f"сеть: {reason}")


def extract(archive: pathlib.Path, staging: pathlib.Path, lock: Lock) -> List[str]:
    """Распаковать плоский архив в ``staging``; проверить состав против lock-файла. Возвращает имена файлов."""
    shutil.rmtree(staging, ignore_errors=True)
    staging.mkdir(parents=True)
    names: List[str] = []
    with tarfile.open(archive, mode="r:gz") as tar:
        for member in tar.getmembers():
            name = member.name
            if not member.isfile() or "/" in name or "\\" in name or name.startswith(".") or name in names:
                raise PackError(f"в архиве недопустимый элемент {name!r} (ждём плоский список файлов)")
            source = tar.extractfile(member)
            with source, (staging / name).open("wb") as out:
                shutil.copyfileobj(source, out)
            names.append(name)
    if INDEX_FILE not in names or LIBRARY_FILE not in names:
        raise PackError(f"в архиве нет {INDEX_FILE} или {LIBRARY_FILE}")
    summary = json.loads((staging / LIBRARY_FILE).read_text(encoding="utf-8"))
    if summary.get("version") != lock["version"] or summary.get("scores") != lock["scores"]:
        raise PackError(f"{LIBRARY_FILE} архива: версия {summary.get('version')} / партитур {summary.get('scores')}, "
                        f"lock: {lock['version']} / {lock['scores']}")
    return names


def install(staging: pathlib.Path, target: pathlib.Path, names: Sequence[str]) -> None:
    """Файлы новой версии — на место по одному (индекс последним), лишние файлы прошлой версии — удалить."""
    for name in sorted(names, key=lambda n: n == INDEX_FILE):
        os.replace(staging / name, target / name)
    keep = set(names)
    for old in target.iterdir():
        if old.is_file() and not old.name.startswith(".") and old.name not in keep:
            old.unlink()
    shutil.rmtree(staging, ignore_errors=True)


def apply(lock: Lock, target: pathlib.Path, opener: Opener = _default_open,
          sleep: Callable[[float], None] = time.sleep) -> str:
    """Довести ``target`` до версии lock-файла. Возвращает строку итога; провал — ``PackError``."""
    target.mkdir(parents=True, exist_ok=True)
    if is_installed(lock, target):
        return f"no-op: версия {lock['version']} уже стоит (маркер = sha256 архива), сети нет"
    marker = target / MARKER_FILENAME
    marker.unlink(missing_ok=True)  # неполный каталог не должен выглядеть готовым
    archive = target / DOWNLOAD
    try:
        download(lock, archive, opener, sleep)
        names = extract(archive, target / STAGING, lock)
        install(target / STAGING, target, names)
    finally:
        archive.unlink(missing_ok=True)
        shutil.rmtree(target / STAGING, ignore_errors=True)
    marker.write_text(marker_text(lock), encoding="utf-8")
    return f"установлена версия {lock['version']}: {lock['scores']} партитур, {len(names)} файлов, sha256 сошёлся"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="fetch_score_library.py",
                                     description="Библиотека партитур с registry katana на хост (score-library).")
    parser.add_argument("--target", required=True, metavar="DIR", help="каталог библиотеки на хосте")
    parser.add_argument("--lock", required=True, metavar="FILE", help="score_library.lock.json")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    for stream in (sys.stdout, sys.stderr):  # голая Pi под sudo может жить в C-локали
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    args = build_arg_parser().parse_args(argv)
    target = pathlib.Path(args.target).expanduser()
    try:
        lock = load_lock(pathlib.Path(args.lock))
        print(f"{lock.get('title', 'score library')} {lock['version']}: {blob_url(lock)} → {target}", flush=True)
        print(f"score-library: {apply(lock, target)}", flush=True)
    except (PackError, OSError, tarfile.TarError, ValueError) as exc:
        print(f"ERROR: score-library: {exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
