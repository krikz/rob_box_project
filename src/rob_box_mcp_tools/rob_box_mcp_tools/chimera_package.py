"""Чтение поставки Химеры с диска (ADR-0175 шаг 1.1, #5152).

Без ROS и без ``rob_box_music``: каталог инструментов и нода звука импортируют этот модуль напрямую и берут
тот же каталог, что тул проверяет на старте. Вторая таблица имён здесь была бы враньём (ADR-0148).
Лежит в корне пакета, не в ``tools``: импорт ``tools`` тянет ROS и музыку, а звуковой ноде они не нужны.
"""

from __future__ import annotations

import hashlib
import json
import os
import wave
from typing import Mapping, Optional, Tuple

PACKAGE_DIR = "/ws/sound_pack/chimera"  # дефолт: монтирование чекаута, docker/sound/docker-compose.yaml
PROFILE_NAME = "profile.json"
RATE_HZ = 16000  # формат Лайт и ReSpeaker: моно PCM16, ADR-0175
CHANNELS = 1
SAMPLE_WIDTH = 2

VerifiedCall = Tuple[str, str, str]  # (существо, вызов, имя файла)


def package_dir() -> str:
    """Каталог поставки. Переменная перекрывает дефолт, чтобы тест не писал в /ws."""
    return os.environ.get("CHIMERA_PACKAGE_DIR", PACKAGE_DIR)


def read_calls(directory: Optional[str] = None) -> Tuple[VerifiedCall, ...]:
    """Вызовы, у которых файл на месте, sha256 совпал и заголовок — моно 16 кГц PCM16.

    Каталога, профиля или ни одной сверки нет — пусто. Пусто и есть честный ответ, не заглушка с именами.
    """
    root = directory if directory is not None else package_dir()
    path = os.path.join(root, PROFILE_NAME)
    if not os.path.isfile(path):
        return ()
    try:
        rows = json.loads(open(path, encoding="utf-8").read()).get("calls")
    except (OSError, ValueError):
        return ()
    if not isinstance(rows, list):
        return ()
    good = [item for item in (_one(root, row) for row in rows) if item is not None]
    return tuple(good)


def verified_clip(directory: str, filename: str) -> bool:
    """Файл объявлен этой поставкой и прошёл ту же сверку. Чужой WAV рядом в звук не попадает."""
    if os.path.basename(filename) != filename:
        return False
    return any(item[2] == filename for item in read_calls(directory))


def enum_values(directory: Optional[str] = None) -> Tuple[Tuple[str, ...], Tuple[str, ...], Tuple[str, ...]]:
    """Действия, существа, вызовы для схемы тула. Нет сверенных файлов — все три пусты."""
    calls = read_calls(directory)
    if not calls:
        return (), (), ()
    creatures: dict = {}
    for creature, name, _filename in calls:
        creatures.setdefault(creature, []).append(name)
    names = tuple(dict.fromkeys(name for _creature, name, _file in calls))
    return ("call",), tuple(creatures), names


def _one(root: str, row: object) -> Optional[VerifiedCall]:
    if not isinstance(row, Mapping):
        return None
    creature, name, filename = str(row.get("creature") or ""), str(row.get("call") or ""), str(row.get("file") or "")
    expect = str(row.get("sha256") or "")
    if not creature or not name or not filename or not expect or os.path.basename(filename) != filename:
        return None
    path = os.path.join(root, filename)
    if not _hash_matches(path, expect) or not _wav_ok(path):
        return None
    return creature, name, filename


def _hash_matches(path: str, expect: str) -> bool:
    try:
        digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
    except OSError:
        return False
    return digest == expect


def _wav_ok(path: str) -> bool:
    try:
        with wave.open(path) as handle:
            return (
                handle.getnchannels() == CHANNELS
                and handle.getframerate() == RATE_HZ
                and handle.getsampwidth() == SAMPLE_WIDTH
                and handle.getnframes() > 0
            )
    except (OSError, wave.Error, EOFError):
        return False


__all__ = [
    "PACKAGE_DIR",
    "PROFILE_NAME",
    "VerifiedCall",
    "package_dir",
    "read_calls",
    "verified_clip",
    "enum_values",
]
