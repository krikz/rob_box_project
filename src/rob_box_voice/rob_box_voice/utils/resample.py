"""
Lightweight audio resampling helper — numpy-only, no ``pyaudio``.

ADR-0145 §4 TTSNode step 1 — moved verbatim from ``tts_node.py`` (no
behaviour change); ``tts_node`` re-exports :func:`resample_audio` for
backward compatibility.

Deliberately a separate module from ``utils/audio_utils.py``: that module
does ``import pyaudio`` at top level (used by its ReSpeaker/PyAudio device
helpers), and ``tts_node.py`` never imported ``audio_utils`` before this
refactor. Putting ``resample_audio`` there would make importing
``rob_box_voice.tts_node`` transitively pull in ``pyaudio`` — a new
import-time dependency the original module never had, i.e. a behaviour
change this "clean transfer" step must not introduce.
"""

import numpy as np


def resample_audio(audio: np.ndarray, orig_sr: float, target_sr: float) -> np.ndarray:
    """
    Resample audio from original sample rate to target sample rate using linear interpolation.

    This is a lightweight resampling implementation suitable for TTS audio where:
    - Low latency is important (no heavy dependencies like scipy/librosa)
    - Audio quality is acceptable for voice synthesis
    - Minimal artifacts for pitch shifting within reasonable range (1.0-3.0x)

    For higher quality resampling, consider using scipy.signal.resample or librosa.resample.

    Args:
        audio: Audio data as numpy array (mono, float32, range -1.0 to 1.0)
        orig_sr: Original sample rate (e.g., 22050 or 10022.7 for fractional rates)
        target_sr: Target sample rate (e.g., 16000)

    Returns:
        Resampled audio at target sample rate
    """
    if abs(orig_sr - target_sr) < 0.01:  # Use epsilon comparison for floats
        return audio

    # Calculate the resampling ratio
    duration = len(audio) / orig_sr
    target_length = int(duration * target_sr)

    # Create new time indices for interpolation
    orig_indices = np.linspace(0, len(audio) - 1, len(audio))
    target_indices = np.linspace(0, len(audio) - 1, target_length)

    # Linear interpolation
    resampled = np.interp(target_indices, orig_indices, audio)

    return resampled
