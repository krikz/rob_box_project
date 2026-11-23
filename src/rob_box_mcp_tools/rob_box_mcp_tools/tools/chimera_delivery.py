"""Поставка готовых клипов Химеры (ADR-0175 шаг 1.1, карточка #5152).

Профиль читается с диска и сверяется с файлами: вызов объявляется, только если его WAV лежит рядом и
sha256 совпал с записью. Битый или чужой файл в список не попадает — объявить его было бы ложью.
Синтеза здесь нет: речь, пение и игра по названию этот пакет не обещает.
"""

from __future__ import annotations

import hashlib
import json
import os
import wave
from typing import Mapping, Optional, Sequence, Tuple

from . import chimera_decide as decide

PACKAGE_DIR = "/ws/sound_pack/chimera"  # каталог поставки в образе voice-assistant
PROFILE_NAME = "profile.json"
RATE_HZ = 16000  # формат Лайт и ReSpeaker: моно PCM16, ADR-0175
CHANNELS = 1
SAMPLE_WIDTH = 2

DeliveryCalls = Tuple[Tuple[str, str, str], ...]  # (существо, вызов, имя файла)


def package_dir() -> str:
    """Каталог поставки: переменная окружения перекрывает путь образа, чтобы тест не писал в /ws."""
    return os.environ.get("CHIMERA_PACKAGE_DIR", PACKAGE_DIR)


def load_delivery(directory: Optional[str] = None) -> decide.ChimeraProfile:
    """Профиль из поставки. Каталога или профиля нет — заглушка, ни одного настоящего вызова."""
    root = directory if directory is not None else package_dir()
    path = os.path.join(root, PROFILE_NAME)
    if not os.path.isfile(path):
        return decide.stub_profile()
    try:
        body = json.loads(open(path, encoding="utf-8").read())
    except (OSError, ValueError):
        return decide.stub_profile()
    calls = _verified(root, body.get("calls"))
    if not calls:
        return decide.stub_profile()
    return _profile(calls, body)


def advertised_calls(profile: decide.ChimeraProfile) -> DeliveryCalls:
    """Вызовы, которые профиль реально отдаёт: существо, имя вызова, имя файла. Заглушка — пусто."""
    if profile.stub:
        return ()
    found: list = []
    for creature, names in profile.creatures.items():
        for name in names:
            clip = profile.calls.get(f"{creature}:{name}") or ""
            if clip and not clip.startswith("clip:"):
                found.append((creature, name, clip))
    return tuple(sorted(found))


def _profile(calls: DeliveryCalls, body: Mapping) -> decide.ChimeraProfile:
    creatures: dict = {}
    clips: dict = {}
    for creature, name, filename in calls:
        creatures.setdefault(creature, []).append(name)
        clips[f"{creature}:{name}"] = filename
    note = (
        f"поставка {body.get('package')} {body.get('version')}, "
        f"исходник {body.get('source_sha')}, карточка #{body.get('source_issue')}"
    )
    return decide.ChimeraProfile(
        actions=("call",),  # готовые WAV; речь, пение и игра этот пакет не умеет и не объявляет
        creatures={key: tuple(names) for key, names in creatures.items()},
        calls=clips,
        stub=False,
        note=note,
    )


def _verified(root: str, rows: object) -> DeliveryCalls:
    """Строки профиля, у которых файл на месте, хеш совпал и заголовок — моно 16 кГц PCM16."""
    if not isinstance(rows, list):
        return ()
    good = []
    for row in rows:
        item = _one(root, row)
        if item is not None:
            good.append(item)
    return tuple(good)


def _one(root: str, row: object) -> Optional[Tuple[str, str, str]]:
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


def call_names(profile: decide.ChimeraProfile) -> Sequence[str]:
    """Имена вызовов для схемы инструмента. Повтор имени у разных существ — один раз."""
    return tuple(dict.fromkeys(name for _creature, name, _file in advertised_calls(profile)))


__all__ = [
    "PACKAGE_DIR",
    "PROFILE_NAME",
    "package_dir",
    "load_delivery",
    "advertised_calls",
    "call_names",
]
