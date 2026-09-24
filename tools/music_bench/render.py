"""Один прогон стенда: ``CompositionSpec`` -> :class:`~tools.music_bench.
metrics.TrackMetrics` (issue #2977).

Склеивает :mod:`.note_extract` (спека -> события), :mod:`.synth` (события
-> буферы по ролям), :mod:`.master_chain` (мастер-шина) и :mod:`.metrics`
(числа). Ничего не проигрывает и не пишет на диск по умолчанию —
:func:`render_bench_track` умеет опционально сохранить .wav для ручного
прослушивания (``--save-wav`` в ``run_bench.py``).
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from . import _repo_paths  # noqa: F401
from .master_chain import apply_master_chain
from .metrics import (
    TrackMetrics,
    crest_factor_db,
    pct_time_over,
    peak,
    simultaneous_layers_per_section,
    spectral_centroid_hz,
    true_peak_approx,
)
from .note_extract import extract_events
from .synth import render_events, sum_buffers

from rob_box_mcp_tools.core.arranger import (  # noqa: E402
    BEATS_PER_BAR,
    CompositionSpec,
    resolve_form,
)

SAMPLE_RATE = 16000  # docker/vision/scripts/supercollider/start_supercollider.sh -S 16000


def render_bench_track(
    spec: CompositionSpec,
    label: str,
    sample_rate: int = SAMPLE_RATE,
    seed: int = 0,
    save_wav_path: Optional[str] = None,
) -> TrackMetrics:
    events, total_beats = extract_events(spec)
    buffers = render_events(events, total_beats, spec.bpm, sample_rate=sample_rate, seed=seed)
    mix = sum_buffers(buffers)

    chain = apply_master_chain(mix, sample_rate)

    role_centroid = {
        role: spectral_centroid_hz(buf, sample_rate)
        for role, buf in buffers.items()
    }

    plan = resolve_form(spec.form, getattr(spec, "theme_bars", 0))
    layers_per_section = simultaneous_layers_per_section(events, plan, BEATS_PER_BAR)
    max_layers = max((n for _n, _b, n in layers_per_section), default=0)

    if save_wav_path:
        _save_wav(save_wav_path, chain.output, sample_rate)

    return TrackMetrics(
        label=label,
        pre_tanh_peak=peak(chain.pre_tanh),
        pct_time_over_1=pct_time_over(chain.pre_tanh, 1.0),
        true_peak=true_peak_approx(chain.output),
        crest_factor_db=crest_factor_db(chain.output),
        role_centroid_hz=role_centroid,
        layers_per_section=layers_per_section,
        max_simultaneous_layers=max_layers,
    )


def _save_wav(path: str, sig: np.ndarray, sample_rate: int) -> None:
    import wave

    clipped = np.clip(sig, -1.0, 1.0)
    pcm = (clipped * 32767.0).astype(np.int16)
    with wave.open(path, "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(pcm.tobytes())
