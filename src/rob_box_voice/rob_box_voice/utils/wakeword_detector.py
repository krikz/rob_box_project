#!/usr/bin/env python3
"""
wakeword_detector.py - Always-on wake word detection using openWakeWord

Purpose:
    Provides lightweight, always-on wake word detection that runs in-process
    on every audio chunk BEFORE speech is sent to STT. This eliminates the
    1-3s STT round-trip that the text-based wake word check requires.

    Designed to be ROS-agnostic (pure Python) for easy unit testing.

Public Interface:
    - WakeWordDetector: Main class
    - WakeWordResult: Named tuple for detection events

Usage:
    detector = WakeWordDetector(model_paths=['/models/ww/robbot.tflite'], threshold=0.5)
    detector.start()
    for chunk in pcm_stream:                    # 512 samples, 16kHz int16
        result = detector.process_chunk(chunk)
        if result:
            print(f"Wake word! model={result.model_name} score={result.score:.2f}")
    detector.stop()

Tests:
    See: test/unit/utils/test_wakeword_detector.py

Dependencies:
    - openwakeword (optional, falls back to no-op if not installed)
    - numpy
"""

import logging
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

logger = logging.getLogger(__name__)

# --- openWakeWord import (optional) -----------------------------------------
try:
    from openwakeword.model import Model as OWWModel

    _OWW_AVAILABLE = True
except ImportError:  # pragma: no cover
    _OWW_AVAILABLE = False
    logger.warning(
        "openWakeWord not installed. WakeWordDetector will be inactive. "
        "Install with: pip install openwakeword"
    )


@dataclass
class WakeWordResult:
    """Result of a wake word detection event."""

    model_name: str
    """Name of the model that fired (filename without extension)."""

    score: float
    """Detection confidence score (0.0 – 1.0)."""

    timestamp: float
    """Unix timestamp of detection (time.time())."""


class WakeWordDetector:
    """
    Always-on wake word detector backed by openWakeWord.

    Each call to process_chunk() feeds 512-sample PCM-16 audio into the
    openWakeWord inference pipeline. Returns a WakeWordResult on detection
    or None otherwise.

    Attributes:
        model_paths: Paths to .tflite / .onnx wake word model files.
        threshold: Minimum score to consider a detection (0.0 – 1.0).
        callback: Optional callable invoked on every detection.
        enabled: Whether detection is active.

    Notes:
        - Thread-safe for single-producer; process_chunk() is NOT re-entrant
          but is safe to call from a PyAudio callback thread.
        - If openWakeWord is not installed, all calls are no-ops and
          ``available`` is False.
        - Chunk size must be 512 samples at 16000 Hz (openWakeWord default).
    """

    CHUNK_SAMPLES: int = 512  # openWakeWord expects 512-sample chunks at 16kHz

    def __init__(
        self,
        model_paths: Optional[List[str]] = None,
        threshold: float = 0.5,
        callback: Optional[Callable[[WakeWordResult], None]] = None,
    ):
        """
        Initialise WakeWordDetector.

        Args:
            model_paths: List of .tflite/.onnx model paths. If empty or None
                and openWakeWord is available, pre-packaged models are loaded.
            threshold: Detection threshold (default: 0.5).
            callback: Optional callable called on every detection.
        """
        self.model_paths: List[str] = model_paths or []
        self.threshold = threshold
        self.callback = callback
        self.enabled = True

        self._model: Optional[object] = None  # OWWModel instance
        self._last_detection_time: float = 0.0
        self._cooldown_sec: float = 1.0  # Ignore repeated detections within 1s

        if _OWW_AVAILABLE:
            self._load_model()
        else:
            logger.warning("WakeWordDetector: openWakeWord unavailable — running in no-op mode")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def available(self) -> bool:
        """True if openWakeWord is installed and model loaded successfully."""
        return _OWW_AVAILABLE and self._model is not None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _load_model(self) -> None:
        """Load openWakeWord model. Called once at init."""
        try:
            if self.model_paths:
                self._model = OWWModel(
                    wakeword_models=self.model_paths,
                    inference_framework="tflite",
                )
                logger.info(f"WakeWordDetector: loaded {len(self.model_paths)} model(s): {self.model_paths}")
            else:
                # Load bundled pre-trained models (hey_jarvis etc.)
                self._model = OWWModel(inference_framework="tflite")
                logger.info("WakeWordDetector: loaded pre-packaged openWakeWord models")
        except Exception as exc:  # pragma: no cover
            logger.error(f"WakeWordDetector: failed to load model — {exc}")
            self._model = None

    def stop(self) -> None:
        """Disable detection (does not unload model)."""
        self.enabled = False

    def start(self) -> None:
        """Enable detection."""
        self.enabled = True

    # ------------------------------------------------------------------
    # Core API
    # ------------------------------------------------------------------

    def process_chunk(self, pcm_bytes: bytes) -> Optional[WakeWordResult]:
        """
        Feed one audio chunk into the wake word engine.

        Args:
            pcm_bytes: Raw PCM-16 mono bytes. Should be CHUNK_SAMPLES * 2
                bytes (1024 bytes at 16kHz). Shorter chunks are zero-padded;
                longer chunks are truncated to first CHUNK_SAMPLES samples.

        Returns:
            WakeWordResult if a wake word was detected, None otherwise.
        """
        if not self.enabled or not self.available:
            return None

        import numpy as np

        # Convert to int16 numpy array
        audio_np = np.frombuffer(pcm_bytes, dtype=np.int16)

        # Pad or truncate to CHUNK_SAMPLES
        if len(audio_np) < self.CHUNK_SAMPLES:
            audio_np = np.pad(audio_np, (0, self.CHUNK_SAMPLES - len(audio_np)))
        elif len(audio_np) > self.CHUNK_SAMPLES:
            audio_np = audio_np[: self.CHUNK_SAMPLES]

        # Run inference
        try:
            prediction = self._model.predict(audio_np)
        except Exception as exc:  # pragma: no cover
            logger.debug(f"WakeWordDetector: inference error — {exc}")
            return None

        # Check all models for threshold crossing
        for model_name, score in prediction.items():
            if float(score) >= self.threshold:
                now = time.time()
                # Cooldown: suppress repeated fires within cooldown window
                if now - self._last_detection_time < self._cooldown_sec:
                    continue
                self._last_detection_time = now
                result = WakeWordResult(
                    model_name=str(model_name),
                    score=float(score),
                    timestamp=now,
                )
                logger.info(
                    f"🔔 Wake word detected: model={result.model_name} score={result.score:.3f}"
                )
                if self.callback:
                    try:
                        self.callback(result)
                    except Exception as cb_exc:  # pragma: no cover
                        logger.error(f"WakeWordDetector callback error: {cb_exc}")
                return result

        return None

    def reset(self) -> None:
        """Reset detection state (cooldown timer). Useful between tests."""
        self._last_detection_time = 0.0
