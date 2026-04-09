"""Download Renardo/FoxDot sample packs during Docker build.

Run during Docker image build to bake samples into the layer.
Packages: 0_foxdot_default (drums/perc), 1_pitchglitch_samples (extended timbres).
On failure — just warns, build continues (synth-only fallback).
"""

from __future__ import annotations

import json
import pathlib
import shutil
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from http.client import RemoteDisconnected
from typing import Iterator
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

try:
    from renardo_gatherer.collections import (
        DEFAULT_SAMPLES_PACK_NAME,
        SAMPLES_DIR_PATH,
        SAMPLES_DOWNLOAD_SERVER,
        is_default_spack_initialized,
    )
except ImportError:
    DEFAULT_SAMPLES_PACK_NAME = "0_foxdot_default"
    SAMPLES_DIR_PATH = pathlib.Path.home() / ".config" / "renardo" / "samples"
    SAMPLES_DOWNLOAD_SERVER = "https://collections.renardo.org/samples"

    def is_default_spack_initialized() -> bool:
        return (SAMPLES_DIR_PATH / DEFAULT_SAMPLES_PACK_NAME / "downloaded_at.txt").exists()


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


def is_pack_present(pack_name: str) -> bool:
    pack_dir = pathlib.Path(SAMPLES_DIR_PATH) / pack_name
    return pack_dir.exists() and any(pack_dir.iterdir())


def write_default_pack_marker(logger: Logger) -> None:
    marker_file = pathlib.Path(SAMPLES_DIR_PATH) / DEFAULT_SAMPLES_PACK_NAME / "downloaded_at.txt"
    marker_file.parent.mkdir(parents=True, exist_ok=True)
    marker_file.write_text(str(datetime.now()), encoding="utf-8")
    logger.write_line(f"Updated {marker_file.name} for {DEFAULT_SAMPLES_PACK_NAME}")


def download_pack(pack_name: str, logger: Logger) -> list[tuple[str, pathlib.Path]]:
    json_url = f"{SAMPLES_DOWNLOAD_SERVER}/{pack_name}/collection_index.json"
    return download_collection(
        json_url=json_url,
        download_dir=pathlib.Path(SAMPLES_DIR_PATH),
        logger=logger,
        max_workers=MAX_WORKERS,
    )


def main() -> None:
    logger = Logger()

    if is_default_spack_initialized():
        print(f"{DEFAULT_SAMPLES_PACK_NAME}: already present, skipping", flush=True)
    else:
        print(f"Downloading {DEFAULT_SAMPLES_PACK_NAME}...", flush=True)
        default_failures = download_pack(DEFAULT_SAMPLES_PACK_NAME, logger)
        if default_failures:
            raise RuntimeError(f"{DEFAULT_SAMPLES_PACK_NAME}: {len(default_failures)} files failed to download")
        write_default_pack_marker(logger)
        print(f"{DEFAULT_SAMPLES_PACK_NAME}: done", flush=True)

    for pack_name in PACKS_TO_DOWNLOAD:
        if pack_name == DEFAULT_SAMPLES_PACK_NAME:
            continue
        if is_pack_present(pack_name):
            print(f"{pack_name}: already present, skipping", flush=True)
            continue

        print(f"Downloading {pack_name}...", flush=True)
        failures = download_pack(pack_name, logger)
        if failures:
            raise RuntimeError(f"{pack_name}: {len(failures)} files failed to download")
        print(f"{pack_name}: done", flush=True)


if __name__ == "__main__":
    main()
