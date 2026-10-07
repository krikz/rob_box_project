"""Хук-фетчер Ресурсного пака: сэмпл-паки на ХОСТ по lock-файлу (один скрипт на все паки).

Паки (lock-файл ``--lock`` на пак, рядом с этим скриптом):
  * ``dj_dave_samples.lock.json`` — сэмплы DJ_Dave (issue #3219): ``algorave-dave/samples``
    (вокальные/бит-стемы, ``spilltab`` по пиннутому коммиту), ``lil-data/dj_dave-array_remix``
    (mp3), ``tidalcycles/Dirt-Samples`` (``tech``/``psr``/``hh``/``cp``),
    ``ritchse/tidal-drum-machines`` (hh/cp двух банков). У источников лицензии нет;
  * ``sonicpi_samples.lock.json`` — Sonic Pi samples (CC0, 206 flac + README);
  * ``muldjord_kit.lock.json`` — DrumGizmo MuldjordKit (CC BY 4.0, 32 flac выборкой).

ФАЙЛЫ В GIT НЕ КОММИТИМ: у DJ_Dave нет лицензии (стемы чужих песен, репозиторий
публичный), у остальных паков сэмплы тоже живут только на хосте. В git лежит
этот код и lock-файлы (путь → пиннутый коммит + sha256 + размер).

Почему lock-файл, а не поле ``sha256`` в manifest.yaml: у записи с
``fetch_hook`` ``apply_resource_pack.sh`` поле sha256 запрещает (одна сумма
на каталог — фикция). Пофайловые суммы поэтому живут здесь, а проверяет их
этот хук: скачанный файл с чужой суммой НЕ ставится никогда (ADR-0018).

Идемпотентность: файл на месте и sha256 совпал — сети нет. Маркер
``.complete`` пишется ТОЛЬКО после того, как все файлы сошлись.

Куда: ``--target <dir>`` (запись манифеста; для DJ_Dave
``/opt/rob_box/samples/dj_dave``). Раскладка внутри — поле ``dest`` lock-файла.

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
from typing import Callable, Dict, List, Optional, Sequence, Tuple
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

MARKER_FILENAME = ".complete"
RAW_URL = "https://raw.githubusercontent.com/{repo}/{commit}/{path}"
USER_AGENT = "rob-box-sample-downloader/1.0"
TIMEOUT_SECONDS = 60
RETRIES = 4

Entry = Dict[str, object]
Opener = Callable[[str], "object"]


def load_lock(path: pathlib.Path) -> Tuple[str, List[Entry]]:
    """Прочитать lock-файл: (заголовок для лога, список файлов)."""
    data = json.loads(path.read_text(encoding="utf-8"))
    files = data["files"]
    if not files:
        raise ValueError(f"{path}: пустой список файлов")
    return str(data.get("title", "sample pack")), files


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
        prog="fetch_sample_pack.py",
        description="Скачать сэмпл-пак по lock-файлу (записи dj-dave-samples, sonicpi-samples, muldjord-kit).",
    )
    parser.add_argument("--target", required=True, metavar="DIR", help="каталог пака на хосте")
    parser.add_argument("--lock", required=True, metavar="FILE", help="lock-файл пака")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    # Голая Pi под sudo может жить в C-локали: русский текст лога не должен
    # ронять скачивание (UnicodeEncodeError), только потерять читаемость.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    args = build_arg_parser().parse_args(argv)
    target = pathlib.Path(args.target).expanduser()
    title, files = load_lock(pathlib.Path(args.lock))
    print(f"{title} target: {target} ({len(files)} файлов)", flush=True)
    target.mkdir(parents=True, exist_ok=True)
    marker = target / MARKER_FILENAME
    marker.unlink(missing_ok=True)  # неполный каталог не должен выглядеть готовым
    problems = fetch_all(files, target)
    if problems:
        print(f"ERROR: {len(problems)} из {len(files)} файлов не встали", file=sys.stderr, flush=True)
        return 1
    marker.write_text(f"{len(files)} files\n", encoding="utf-8")
    print(f"{title}: done ({len(files)} файлов, sha256 всех сошёлся)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
