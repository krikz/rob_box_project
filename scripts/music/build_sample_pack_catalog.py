#!/usr/bin/env python3
"""Собрать JSON-инвентарь сэмпл-пака (Sonic Pi / MuldjordKit) из lock-файла и скачанных файлов.

Каталог — данные рядом с ``sample_dave.json``: ``src/rob_box_music/rob_box_music/data/sample_<пак>.json``.
Состав берётся из lock-файла Ресурсного пака (``docker/vision/scripts/resource_pack/<пак>.lock.json``),
длительность/каналы/пик/RMS — со скачанных файлов (``--dir``: каталог, куда фетчер положил пак, либо
ручная выгрузка по ``dest`` из lock). Роль (kick/snare/tom/hat/ride/crash/break/loop/…) — таблица правил
в этом файле, а не LLM и не ручная правка JSON.

Запуск (dev-инструмент на хосте разработчика, НЕ рантайм; нужны numpy и soundfile)::

    python scripts/music/build_sample_pack_catalog.py sonicpi --dir <каталог пака>
    python scripts/music/build_sample_pack_catalog.py muldjord --dir <каталог пака>

Пока ни один пак в генератор (ADR-0153 S3/S5) не подключён: только данные. Тест
``src/rob_box_music/test/test_sample_pack_catalogs.py`` держит каталог равным lock-файлу.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import sys
from typing import Dict, Sequence, Tuple

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
PACK_DIR = REPO_ROOT / "docker" / "vision" / "scripts" / "resource_pack"
DATA_DIR = REPO_ROOT / "src" / "rob_box_music" / "rob_box_music" / "data"

#: Роли каталога. Первые восемь — барабанные, остальные — всё прочее из пака (генератор их не берёт).
ROLES: Tuple[str, ...] = (
    "kick", "snare", "tom", "hat", "ride", "crash", "break", "loop", "perc", "bass", "tonal", "fx")

#: Sonic Pi: первое совпавшее правило (по имени файла без расширения) задаёт роль.
SONICPI_RULES: Sequence[Tuple[str, str]] = (
    (r"^loop_amen(_full)?$|^loop_breakbeat$", "break"),
    (r"^loop_", "loop"),
    (r"^arovane_beat_", "loop"),
    (r"^bd_|^drum_bass_|^drum_heavy_kick$|^elec_(hollow|soft)_kick$", "kick"),
    (r"^sn_|^drum_snare_|^elec_(snare|lo_snare|mid_snare|hi_snare|filt_snare)$", "snare"),
    (r"^drum_tom_|^elec_fuzz_tom$", "tom"),
    (r"^hat_|^drum_cymbal_(closed|open|pedal)$|^tbd_perc_hat$", "hat"),
    (r"^ride_", "ride"),
    (r"^drum_cymbal_(hard|soft)$|^drum_splash_|^elec_cymbal$", "crash"),
    (r"^bass_|^glitch_bass_", "bass"),
    (r"^ambi_|^tbd_(pad|highkey|voctone|fxbed)|^guit_|^elec_(bell|chime|triangle)$", "tonal"),
    (r"^drum_(roll|cowbell)$|^perc_|^glitch_perc|^tbd_perc|^tabla_|^elec_|^mehackit_", "perc"),
)
SONICPI_FALLBACK = "fx"

#: MuldjordKit: каталог инструмента → роль.
MULDJORD_ROLES: Dict[str, str] = {
    "KdrumL": "kick", "KdrumR": "kick", "Snare": "snare",
    "Tom1": "tom", "Tom2": "tom", "Tom3": "tom", "Tom4": "tom",
    "HihatClosed": "hat", "HihatOpen": "hat",
    "RideL": "ride", "RideR": "ride", "RideLBell": "ride", "RideRBell": "ride",
    "CrashL": "crash", "CrashR": "crash", "China": "crash",
}

PACKS = {
    "sonicpi": {
        "lock": "sonicpi_samples.lock.json", "out": "sample_sonicpi.json", "pack_dir": "sonicpi", "prefix": "sonicpi",
        "title": "Sonic Pi samples (CC0)",
    },
    "muldjord": {
        "lock": "muldjord_kit.lock.json", "out": "sample_muldjord.json", "pack_dir": "muldjord", "prefix": "muldjord",
        "title": "DrumGizmo MuldjordKit (CC BY 4.0, Lars Muldjord)",
    },
}


def is_audio(dest: str) -> bool:
    return dest.lower().endswith((".flac", ".wav", ".mp3"))


def sonicpi_role(stem: str) -> str:
    for pattern, role in SONICPI_RULES:
        if re.search(pattern, stem):
            return role
    return SONICPI_FALLBACK


def muldjord_role(dest: str) -> str:
    return MULDJORD_ROLES[dest.split("/")[0]]


def entry_name(pack: str, dest: str) -> str:
    stem = pathlib.PurePosixPath(dest).stem
    if pack == "muldjord":
        inst, _, _ = dest.partition("/")
        index = re.match(r"(\d+)-", stem)
        return f"muldjord_{inst.lower()}_{int(index.group(1)):02d}" if index else f"muldjord_{stem.lower()}"
    return f"sonicpi_{stem}"


def entry_role(pack: str, dest: str) -> str:
    if pack == "muldjord":
        return muldjord_role(dest)
    return sonicpi_role(pathlib.PurePosixPath(dest).stem)


def entry_group(pack: str, dest: str, role: str) -> str:
    return f"muldjord_{pathlib.PurePosixPath(dest).parts[0].lower()}" if pack == "muldjord" else f"sonicpi_{role}"


def db(value: float) -> float:
    return round(20.0 * math.log10(value), 1) if value > 1e-9 else -120.0


def measure(path: pathlib.Path) -> Dict[str, object]:
    import numpy as np  # dev-only зависимости, в рантайм не идут
    import soundfile as sf

    data, rate = sf.read(str(path), dtype="float64", always_2d=True)
    return {
        "seconds": round(data.shape[0] / rate, 3),
        "channels": int(data.shape[1]),
        "samplerate": int(rate),
        "peak_db": db(float(np.abs(data).max())),
        "rms_db": db(float(np.sqrt(np.mean(np.square(data))))),
    }


def build(pack: str, files_dir: pathlib.Path) -> Dict[str, object]:
    cfg = PACKS[pack]
    lock = json.loads((PACK_DIR / cfg["lock"]).read_text(encoding="utf-8"))
    samples: Dict[str, Dict[str, object]] = {}
    groups: Dict[str, str] = {}
    for item in sorted(lock["files"], key=lambda e: str(e["dest"])):
        dest = str(item["dest"])
        if not is_audio(dest):
            continue
        role = entry_role(pack, dest)
        group = entry_group(pack, dest, role)
        groups.setdefault(group, f"{role}: {group}")
        name = entry_name(pack, dest)
        assert name not in samples, f"дубль имени {name}"
        samples[name] = {"path": dest, "role": role, **measure(files_dir / dest), "group": group}
    return {
        "_comment": [
            f"Каталог пака «{cfg['title']}». Файлы в репозитории не лежат: их кладёт на хост Ресурсный пак",
            f"(запись манифеста по хуку {cfg['lock'].replace('.lock.json', '')}), пути = dest из",
            f"docker/vision/scripts/resource_pack/{cfg['lock']} (сверяет тест).",
            "seconds/channels/samplerate/peak_db/rms_db посчитаны python soundfile+numpy по реально скачанным файлам",
            "07.10.2026 (scripts/music/build_sample_pack_catalog.py); роль — таблица правил в этом скрипте.",
            "peak_db/rms_db в дБFS по всем каналам; на 16 кГц DAC робота НЕ прослушано.",
            "В генератор пак НЕ подключён (ADR-0153 S3/S5): только данные.",
        ],
        "pack_dir": cfg["pack_dir"],
        "roles": list(ROLES),
        "groups": dict(sorted(groups.items())),
        "samples": samples,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pack", choices=sorted(PACKS))
    parser.add_argument("--dir", required=True, type=pathlib.Path, help="каталог с файлами пака (по dest из lock)")
    args = parser.parse_args(argv)
    catalog = build(args.pack, args.dir)
    out = DATA_DIR / str(PACKS[args.pack]["out"])
    out.write_text(json.dumps(catalog, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    roles: Dict[str, int] = {}
    for info in catalog["samples"].values():  # type: ignore[union-attr]
        roles[str(info["role"])] = roles.get(str(info["role"]), 0) + 1
    print(f"{out}: {len(catalog['samples'])} сэмплов, роли {roles}")  # type: ignore[arg-type]
    return 0


if __name__ == "__main__":
    sys.exit(main())
