"""Хук-фетчер Ресурсного пака: Renardo/FoxDot сэмплы (~600 МБ) на ХОСТ.

Паки: ``0_foxdot_default`` (drums/perc), ``1_pitchglitch_samples``
(расширенные тембры). Источник — ``collections.renardo.org``, дерево
файлов описано ``collection_index.json`` (не один архив, а несколько
тысяч отдельных .wav) — поэтому это НЕ запись ``url:`` в манифесте, а
именованный хук: манифест хранит ФАКТ наличия хука, логика остаётся
кодом (тот же приём, что ``pre_build: fetch_hailort_wheel`` в
docker/build-manifest.yaml, см. docs/plans/2026-09-15-service-manifest.md §2.5).

Вызывающий — ``apply_resource_pack.sh`` по записи ``renardo-samples``
манифеста ``manifest.yaml``. Раньше этот же файл жил в
``docker/vision/voice_assistant/download_samples.py`` и запускался на
этапе СБОРКИ образа ``voice-resources``; образ удалён, сэмплы приезжают
на хост рядом с моделями (``/opt/rob_box/samples``) и монтируются в
контейнеры bind-mount'ом.

Куда качать (по убыванию приоритета):
  1. ``--target <dir>``
  2. env ``RENARDO_SAMPLES_DIR``
  3. ``SAMPLES_DIR_PATH`` — прежнее поведение (``~/.config/renardo/samples``
     либо то, что подставит установленный ``renardo_gatherer``).

Идемпотентность: маркер ``<dir>/0_foxdot_default/downloaded_at.txt`` +
пофайловая проверка «уже скачан и непустой» внутри ``download_collection``.

Зависимости: только stdlib. ``renardo_gatherer`` используется, если он
есть (тогда берём его константы), но на Vision Pi его НЕТ — фолбэк ниже
обязан оставаться рабочим.

Exit codes: 0 — сэмплы на месте; 1 — что-то не скачалось.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from http.client import RemoteDisconnected
from typing import Iterator, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

try:
    from renardo_gatherer.collections import (
        DEFAULT_SAMPLES_PACK_NAME,
        SAMPLES_DIR_PATH,
        SAMPLES_DOWNLOAD_SERVER,
    )
except ImportError:
    DEFAULT_SAMPLES_PACK_NAME = "0_foxdot_default"
    SAMPLES_DIR_PATH = pathlib.Path.home() / ".config" / "renardo" / "samples"
    SAMPLES_DOWNLOAD_SERVER = "https://collections.renardo.org/samples"

# NB: ``renardo_gatherer.collections.is_default_spack_initialized`` больше не
# импортируется. Он проверяет маркер в СВОЁМ ``SAMPLES_DIR_PATH`` и про
# ``--target``/``RENARDO_SAMPLES_DIR`` ничего не знает: с переездом на
# /opt/rob_box/samples он отвечал бы про другой каталог и молча разрешал
# «уже скачано» там, где пусто. Проверка маркера — три строки, и она обязана
# смотреть ровно в тот каталог, куда мы качаем.
MARKER_FILENAME = "downloaded_at.txt"

# Env-переменная целевого каталога. То же имя читает docker-compose и шаг
# деплоя, чтобы «куда легли сэмплы» было ровно одним словом во всех местах.
SAMPLES_DIR_ENV = "RENARDO_SAMPLES_DIR"


PACKS_TO_DOWNLOAD = (
    DEFAULT_SAMPLES_PACK_NAME,
    "1_pitchglitch_samples",
)
SAMPLES_PATH_PREFIX = "./samples/"
DEFAULT_TIMEOUT_SECONDS = 45
CONCURRENT_RETRIES = 6
SEQUENTIAL_RETRIES = 10
MAX_WORKERS = 4
USER_AGENT = "rob-box-sample-downloader/1.0"


class Logger:
    def write_line(self, msg: str) -> None:
        print(msg, flush=True)


def build_request(url: str) -> Request:
    return Request(url, headers={"User-Agent": USER_AGENT})


def normalize_relative_path(sample_path: str) -> pathlib.Path:
    normalized = sample_path.strip()
    if normalized.startswith(SAMPLES_PATH_PREFIX):
        normalized = normalized[len(SAMPLES_PATH_PREFIX) :]
    elif normalized.startswith("samples/"):
        normalized = normalized[len("samples/") :]
    elif normalized.startswith("./"):
        normalized = normalized[2:]
    return pathlib.Path(normalized)


def resolve_file_url(file_url: str, base_url: str = f"{SAMPLES_DOWNLOAD_SERVER}/") -> str:
    stripped = file_url.strip()
    if urlparse(stripped).scheme:
        return stripped
    return urljoin(base_url, stripped)


def iter_collection_files(node: dict) -> Iterator[tuple[str, pathlib.Path]]:
    if not isinstance(node, dict):
        return

    file_url = node.get("url")
    file_path = node.get("path")
    if isinstance(file_url, str) and isinstance(file_path, str):
        yield resolve_file_url(file_url), normalize_relative_path(file_path)

    children = node.get("children", [])
    if isinstance(children, list):
        for child in children:
            yield from iter_collection_files(child)


def load_json_index(json_url: str, logger: Logger) -> dict:
    try:
        with urlopen(build_request(json_url), timeout=DEFAULT_TIMEOUT_SECONDS) as response:
            return json.load(response)
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        logger.write_line(f"Error downloading collection JSON index {json_url} ({exc})")
        raise RuntimeError(f"Failed to download JSON index: {json_url}") from exc


def download_file(
    *,
    url: str,
    destination: pathlib.Path,
    logger: Logger,
    timeout_seconds: int = DEFAULT_TIMEOUT_SECONDS,
    retries: int = CONCURRENT_RETRIES,
) -> bool:
    filename = pathlib.Path(urlparse(url).path).name or destination.name
    destination.parent.mkdir(parents=True, exist_ok=True)
    temp_path = destination.with_name(f"{destination.name}.part")

    for attempt in range(1, retries + 1):
        try:
            with urlopen(build_request(url), timeout=timeout_seconds) as response, temp_path.open("wb") as handle:
                shutil.copyfileobj(response, handle)
            temp_path.replace(destination)
            logger.write_line(f"Downloaded {filename} to {destination}")
            return True
        except (HTTPError, URLError, RemoteDisconnected, TimeoutError, ConnectionError, OSError) as exc:
            temp_path.unlink(missing_ok=True)
            logger.write_line(f"Error downloading URL {url} ({exc})")
            if attempt < retries:
                logger.write_line(f"Retrying {filename} ({attempt}/{retries})...")
                time.sleep(min(attempt, 5))

    logger.write_line(f"Failed to download URL {url} after {retries} attempts")
    return False


def download_collection(
    *,
    json_url: str,
    download_dir: pathlib.Path,
    logger: Logger,
    max_workers: int = MAX_WORKERS,
) -> list[tuple[str, pathlib.Path]]:
    file_tree = load_json_index(json_url, logger)
    entries = [(url, pathlib.Path(download_dir) / relative_path) for url, relative_path in iter_collection_files(file_tree)]

    pending_entries = [(url, path) for url, path in entries if not path.exists() or path.stat().st_size == 0]
    if not pending_entries:
        return []

    failures: list[tuple[str, pathlib.Path]] = []
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_entry = {
            executor.submit(
                download_file,
                url=url,
                destination=path,
                logger=logger,
                timeout_seconds=DEFAULT_TIMEOUT_SECONDS,
                retries=CONCURRENT_RETRIES,
            ): (url, path)
            for url, path in pending_entries
        }

        for future in as_completed(future_to_entry):
            entry = future_to_entry[future]
            if not future.result():
                failures.append(entry)

    if not failures:
        return []

    logger.write_line(f"Retrying {len(failures)} failed downloads sequentially...")
    remaining_failures: list[tuple[str, pathlib.Path]] = []
    for url, path in failures:
        if path.exists() and path.stat().st_size > 0:
            continue
        if not download_file(
            url=url,
            destination=path,
            logger=logger,
            timeout_seconds=DEFAULT_TIMEOUT_SECONDS,
            retries=SEQUENTIAL_RETRIES,
        ):
            remaining_failures.append((url, path))

    return remaining_failures


def resolve_samples_dir(target: str | None = None) -> pathlib.Path:
    """--target > env RENARDO_SAMPLES_DIR > SAMPLES_DIR_PATH (прежнее поведение).

    Пустая строка в env трактуется как «не задано»: ``RENARDO_SAMPLES_DIR=``
    в .env-файле не должен молча уронить сэмплы в текущий каталог.
    """
    if target:
        return pathlib.Path(target).expanduser()
    env_value = os.environ.get(SAMPLES_DIR_ENV, "").strip()
    if env_value:
        return pathlib.Path(env_value).expanduser()
    return pathlib.Path(SAMPLES_DIR_PATH)


def is_default_spack_initialized(samples_dir: pathlib.Path) -> bool:
    return (pathlib.Path(samples_dir) / DEFAULT_SAMPLES_PACK_NAME / MARKER_FILENAME).exists()


def is_pack_present(pack_name: str, samples_dir: pathlib.Path) -> bool:
    pack_dir = pathlib.Path(samples_dir) / pack_name
    return pack_dir.exists() and any(pack_dir.iterdir())


def write_default_pack_marker(logger: Logger, samples_dir: pathlib.Path) -> None:
    marker_file = pathlib.Path(samples_dir) / DEFAULT_SAMPLES_PACK_NAME / MARKER_FILENAME
    marker_file.parent.mkdir(parents=True, exist_ok=True)
    marker_file.write_text(str(datetime.now()), encoding="utf-8")
    logger.write_line(f"Updated {marker_file.name} for {DEFAULT_SAMPLES_PACK_NAME}")


def download_pack(
    pack_name: str,
    logger: Logger,
    samples_dir: pathlib.Path,
) -> list[tuple[str, pathlib.Path]]:
    json_url = f"{SAMPLES_DOWNLOAD_SERVER}/{pack_name}/collection_index.json"
    return download_collection(
        json_url=json_url,
        download_dir=pathlib.Path(samples_dir),
        logger=logger,
        max_workers=MAX_WORKERS,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fetch_renardo_samples.py",
        description=(
            "Скачать Renardo/FoxDot сэмпл-паки в целевой каталог. "
            "Хук Ресурсного пака (запись renardo-samples в manifest.yaml)."
        ),
    )
    parser.add_argument(
        "--target",
        default=None,
        metavar="DIR",
        help=(
            "куда класть паки. По умолчанию — "
            f"${SAMPLES_DIR_ENV}, а если и он не задан — {SAMPLES_DIR_PATH}"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logger = Logger()
    samples_dir = resolve_samples_dir(args.target)

    print(f"Renardo samples target: {samples_dir}", flush=True)

    if is_default_spack_initialized(samples_dir):
        print(f"{DEFAULT_SAMPLES_PACK_NAME}: already present, skipping", flush=True)
    else:
        print(f"Downloading {DEFAULT_SAMPLES_PACK_NAME}...", flush=True)
        default_failures = download_pack(DEFAULT_SAMPLES_PACK_NAME, logger, samples_dir)
        if default_failures:
            print(
                f"ERROR: {DEFAULT_SAMPLES_PACK_NAME}: "
                f"{len(default_failures)} files failed to download",
                file=sys.stderr,
                flush=True,
            )
            return 1
        # Маркер пишем ТОЛЬКО после успеха: полускачанный пак, помеченный как
        # готовый, — это молчаливо сломанная музыка при следующем прогоне.
        write_default_pack_marker(logger, samples_dir)
        print(f"{DEFAULT_SAMPLES_PACK_NAME}: done", flush=True)

    for pack_name in PACKS_TO_DOWNLOAD:
        if pack_name == DEFAULT_SAMPLES_PACK_NAME:
            continue
        if is_pack_present(pack_name, samples_dir):
            print(f"{pack_name}: already present, skipping", flush=True)
            continue

        print(f"Downloading {pack_name}...", flush=True)
        failures = download_pack(pack_name, logger, samples_dir)
        if failures:
            print(
                f"ERROR: {pack_name}: {len(failures)} files failed to download",
                file=sys.stderr,
                flush=True,
            )
            return 1
        print(f"{pack_name}: done", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
