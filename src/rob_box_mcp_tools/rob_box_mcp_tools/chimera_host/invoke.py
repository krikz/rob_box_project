"""Команда рендера нот закреплённым CLI Химеры (ADR-0175 §2.1, #5205, #5152).

Что поёт — решает вызывающий и кладёт в JSON сам. Этот модуль JSON не читает и не пишет:
чужой текст песни в код и в лог не попадает. Нет бинаря, не сошёлся SHA или тембр не из
списка — честный отказ, не «спел».
"""

from __future__ import annotations

import hashlib
import os
from typing import Optional, Sequence, Tuple

PINNED_SHA = "caf270abf893bdc63f6342059d4468f754edf5c6"  # jamesondh/chimera-vocal-synth, проба #5014
TIMBRES: Tuple[str, ...] = ("male", "female", "child")
BINARY_NAME = "chimera"
DATA_DIR = "data"  # public/data закреплённого SHA: cmudict и лексикон, без них CLI не стартует
HOST_DIR = "/opt/chimera"  # дефолт образа; тест перекрывает переменной

#: Куда CLI ищет словарь. Путь зашит при сборке от CARGO_MANIFEST_DIR, поэтому данные
#: монтируются ровно туда, а не «куда удобно».
DATA_MOUNT = "/src/public/data"


def host_dir() -> str:
    """Каталог хоста: бинарь, SHA-пин и данные. Переменная перекрывает дефолт, чтобы тест не писал в /opt."""
    return os.environ.get("CHIMERA_HOST_DIR", HOST_DIR)


class ChimeraHost:
    """Проверяет пин и собирает команду. Сам процесс не запускает: это делает владелец звука."""

    def __init__(self, directory: Optional[str] = None) -> None:
        self.directory = directory if directory is not None else host_dir()

    def binary_path(self) -> str:
        return os.path.join(self.directory, BINARY_NAME)

    def pin_path(self) -> str:
        return os.path.join(self.directory, "SOURCE_SHA")

    def data_dir(self) -> str:
        return os.path.join(self.directory, DATA_DIR)

    def refusal(self) -> str:
        """Почему рендерить нельзя. Пустая строка — можно."""
        if not os.path.isfile(self.binary_path()):
            return "missing_binary"
        pinned = _read_pin(self.pin_path())
        if pinned != PINNED_SHA:
            return "sha_mismatch"
        if not os.path.isfile(os.path.join(self.data_dir(), "cmudict.txt.gz")):
            return "missing_data"
        return ""

    def notes_command(self, notes_json: str, out_stem: str, timbre: str = "male") -> Tuple[str, ...]:
        """Команда ``render notes``. Отказ — исключение с причиной, не команда в никуда."""
        why = self.refusal()
        if why:
            raise ChimeraUnavailable(why)
        if timbre not in TIMBRES:
            raise ChimeraUnavailable("unsupported_timbre")
        if not notes_json:
            raise ChimeraUnavailable("bad_input")
        return (
            self.binary_path(),
            "render",
            "notes",
            notes_json,
            out_stem,
            timbre,
        )

    def data_mount(self) -> Tuple[str, str]:
        """Пара ``(источник, куда)`` для монтирования словаря в путь, зашитый сборкой."""
        return self.data_dir(), DATA_MOUNT


class ChimeraUnavailable(Exception):
    """Хост не может рендерить. ``reason`` — одна из причин, не текст песни."""

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


def _read_pin(path: str) -> str:
    try:
        return open(path, encoding="utf-8").read().strip()
    except OSError:
        return ""


def sha256_file(path: str) -> str:
    """SHA256 файла бинаря. Нет файла — пусто, не исключение: сверку делает вызывающий."""
    try:
        digest = hashlib.sha256()
        with open(path, "rb") as handle:
            for block in iter(lambda: handle.read(1 << 16), b""):
                digest.update(block)
        return digest.hexdigest()
    except OSError:
        return ""


def pinned_inputs(directory: str) -> Sequence[Tuple[str, str]]:
    """Имена и SHA256 входов хоста (бинарь и словарь). Для манифеста поставки, не для пения."""
    names = (BINARY_NAME, os.path.join(DATA_DIR, "cmudict.txt.gz"))
    return tuple((name, sha256_file(os.path.join(directory, name))) for name in names)


__all__ = [
    "PINNED_SHA",
    "TIMBRES",
    "DATA_DIR",
    "DATA_MOUNT",
    "ChimeraHost",
    "ChimeraUnavailable",
    "host_dir",
    "sha256_file",
    "pinned_inputs",
]
