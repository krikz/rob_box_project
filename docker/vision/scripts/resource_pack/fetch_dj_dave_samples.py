"""Хук-фетчер Ресурсного пака: сэмплы DJ_Dave (issue #3219) на ХОСТ.

Что качаем (список и эталоны — ``dj_dave_samples.lock.json`` рядом):
  * ``algorave-dave/samples`` — вокальные/бит-стемы (cocaina, whatuneed, …)
    и ``spilltab`` (из HEAD удалён, берём по пиннутому коммиту);
  * ``lil-data/dj_dave-array_remix`` — стемы «Array (Lil Data Edit)» (mp3);
  * ``tidalcycles/Dirt-Samples`` — только ``tech``/``psr``/``hh``/``cp``;
  * ``ritchse/tidal-drum-machines`` — только hh/cp двух банков (TR808, DDM110).

ФАЙЛЫ В GIT НЕ КОММИТИМ: у репозиториев сэмплов лицензии нет, в них стемы
чужих песен, а наш репозиторий публичный. В git лежит только этот код и
lock-файл (путь → пиннутый коммит + sha256 + размер).

Почему lock-файл, а не поле ``sha256`` в manifest.yaml: у записи с
``fetch_hook`` ``apply_resource_pack.sh`` поле sha256 запрещает (одна сумма
на каталог — фикция). Пофайловые суммы поэтому живут здесь, а проверяет их
этот хук: скачанный файл с чужой суммой НЕ ставится никогда (ADR-0018).

Идемпотентность: файл на месте и sha256 совпал — сети нет. Маркер
``.complete`` пишется ТОЛЬКО после того, как все файлы сошлись.

Куда: ``--target <dir>`` (запись ``dj-dave-samples`` манифеста; по умолчанию
``/opt/rob_box/samples/dj_dave``). Раскладка внутри: ``algorave/``,
``array/``, ``dirt/``, ``machines/``.

Зависимости: только stdlib.  Exit codes: 0 — всё на месте; 1 — нет.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import shutil
import sys
import time
from typing import Callable, Dict, List, Optional, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

LOCK_FILE = pathlib.Path(__file__).resolve().with_name("dj_dave_samples.lock.json")
MARKER_FILENAME = ".complete"
DEFAULT_TARGET = pathlib.Path("/opt/rob_box/samples/dj_dave")
RAW_URL = "https://raw.githubusercontent.com/{repo}/{commit}/{path}"
USER_AGENT = "rob-box-sample-downloader/1.0"
TIMEOUT_SECONDS = 60
RETRIES = 4

Entry = Dict[str, object]
Opener = Callable[[str], "object"]


def load_lock(path: pathlib.Path = LOCK_FILE) -> List[Entry]:
    files = json.loads(path.read_text(encoding="utf-8"))["files"]
    if not files:
        raise ValueError(f"{path}: пустой список файлов")
    return files


def raw_url(entry: Entry) -> str:
    return RAW_URL.format(repo=entry["repo"], commit=entry["commit"], path=quote(str(entry["src"])))


def sha256_of_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _default_open(url: str):
    return urlopen(Request(url, headers={"User-Agent": USER_AGENT}), timeout=TIMEOUT_SECONDS)


def is_entry_valid(entry: Entry, target: pathlib.Path) -> bool:
    dest = target / str(entry["dest"])
    return dest.is_file() and dest.stat().st_size == entry["size"] and sha256_of_file(dest) == entry["sha256"]


def fetch_entry(
    entry: Entry,
    target: pathlib.Path,
    opener: Opener = _default_open,
    sleep: Callable[[float], None] = time.sleep,
) -> Optional[str]:
    """Скачать один файл. ``None`` — успех, иначе текст причины."""
    dest = target / str(entry["dest"])
    dest.parent.mkdir(parents=True, exist_ok=True)
    part = dest.with_name(dest.name + ".part")
    url = raw_url(entry)
    reason = "не пробовали"
    for attempt in range(1, RETRIES + 1):
        try:
            with opener(url) as response, part.open("wb") as handle:
                shutil.copyfileobj(response, handle)
        except (HTTPError, URLError, TimeoutError, ConnectionError, OSError) as exc:
            reason = f"сеть: {exc}"
            part.unlink(missing_ok=True)
            if attempt < RETRIES:
                sleep(min(attempt, 5))
            continue
        actual = sha256_of_file(part)
        if actual != entry["sha256"]:
            # Не ретраим: другой контент по пиннутому коммиту — не сбой сети.
            part.unlink(missing_ok=True)
            return f"sha256 не совпал (ждали {entry['sha256']}, пришло {actual})"
        part.replace(dest)
        return None
    return reason


def fetch_all(
    files: Sequence[Entry],
    target: pathlib.Path,
    opener: Opener = _default_open,
    sleep: Callable[[float], None] = time.sleep,
) -> List[str]:
    """Довести каталог до lock-файла. Возвращает список проблем (пусто = ок)."""
    problems: List[str] = []
    for entry in files:
        if is_entry_valid(entry, target):
            continue
        reason = fetch_entry(entry, target, opener, sleep)
        if reason is not None:
            problems.append(f"{entry['dest']}: {reason}")
            print(f"ERROR: {entry['dest']}: {reason}", file=sys.stderr, flush=True)
        else:
            print(f"OK {entry['dest']} ({entry['size']} байт, sha256 сошёлся)", flush=True)
    return problems


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fetch_dj_dave_samples.py",
        description="Скачать сэмплы DJ_Dave по lock-файлу (запись dj-dave-samples манифеста).",
    )
    parser.add_argument("--target", default=None, metavar="DIR", help=f"по умолчанию {DEFAULT_TARGET}")
    parser.add_argument("--lock", default=None, metavar="FILE", help="lock-файл (для тестов)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    # Голая Pi под sudo может жить в C-локали: русский текст лога не должен
    # ронять скачивание (UnicodeEncodeError), только потерять читаемость.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    args = build_arg_parser().parse_args(argv)
    target = pathlib.Path(args.target).expanduser() if args.target else DEFAULT_TARGET
    files = load_lock(pathlib.Path(args.lock) if args.lock else LOCK_FILE)
    print(f"DJ_Dave samples target: {target} ({len(files)} файлов)", flush=True)
    target.mkdir(parents=True, exist_ok=True)
    marker = target / MARKER_FILENAME
    marker.unlink(missing_ok=True)  # неполный каталог не должен выглядеть готовым
    problems = fetch_all(files, target)
    if problems:
        print(f"ERROR: {len(problems)} из {len(files)} файлов не встали", file=sys.stderr, flush=True)
        return 1
    marker.write_text(f"{len(files)} files\n", encoding="utf-8")
    print(f"DJ_Dave samples: done ({len(files)} файлов, sha256 всех сошёлся)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
