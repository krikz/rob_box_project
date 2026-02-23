"""Download Renardo/FoxDot sample packs during Docker build.

Run during Docker image build to bake samples into the layer.
Packages: 0_foxdot_default (drums/perc), 1_pitchglitch_samples (extended timbres).
On failure — just warns, build continues (synth-only fallback).
"""
import pathlib

from renardo_gatherer.collections import (
    SAMPLES_DIR_PATH,
    SAMPLES_DOWNLOAD_SERVER,
    download_default_sample_pack,
    download_files_from_json_index_concurrent,
    is_default_spack_initialized,
)


class Logger:
    def write_line(self, msg: str) -> None:
        print(msg, flush=True)


logger = Logger()

# 1. Базовый пак FoxDot (bd, sd, hh, cp, bass...)
if is_default_spack_initialized():
    print("0_foxdot_default: already present, skipping", flush=True)
else:
    print("Downloading 0_foxdot_default...", flush=True)
    download_default_sample_pack(logger=logger)
    print("0_foxdot_default: done", flush=True)

# 2. Расширенный пак pitchglitch
pitchglitch_dir = pathlib.Path(SAMPLES_DIR_PATH) / "1_pitchglitch_samples"
if pitchglitch_dir.exists() and any(pitchglitch_dir.iterdir()):
    print("1_pitchglitch_samples: already present, skipping", flush=True)
else:
    print("Downloading 1_pitchglitch_samples...", flush=True)
    download_files_from_json_index_concurrent(
        json_url=f"{SAMPLES_DOWNLOAD_SERVER}/1_pitchglitch_samples/collection_index.json",
        download_dir=SAMPLES_DIR_PATH,
        max_workers=8,
        logger=logger,
    )
    print("1_pitchglitch_samples: done", flush=True)
