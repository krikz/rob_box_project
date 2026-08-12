"""
test_audio_playback_manager.py — unit-тесты защиты от зависания sd.wait()
(issue #1133).

Воспроизводит сценарий: tts_node отправил audio_chunk → sd.play() → sd.wait()
→ ALSA-devaйс занят / PaErrorCode -9985 / dmix race → sd.wait() висит вечно →
worker в ThreadPoolExecutor залипает → _synthesis_slots не освобождается →
через max_workers+max_queue запросов tts_node перестаёт реагировать на
callback. Без polling-цикла с таймаутом (фикс в PR #1133) — безнадёжный
hang, выручал только watchdog (kill -KILL PID).

Verify:
1. play_audio() возвращается в течение ~8с при зависшем sd.wait() (НЕ вечно).
2. lock освобождается даже при timeout — следующий play_audio() не зависает.
3. Нормальный путь (active=False) — polling завершается < 1с.
4. Non-blocking режим — возвращается мгновенно.
"""
from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from rob_box_voice.audio_playback_manager import AudioPlaybackManager


@pytest.fixture
def fresh_manager():
    """Чистый экземпляр для теста (singleton обходим)."""
    mgr = AudioPlaybackManager.__new__(AudioPlaybackManager)
    mgr._playback_lock = threading.Lock()
    mgr._current_stream = None
    return mgr


def _silent_silence(sample_rate: int, seconds: float = 1.0) -> np.ndarray:
    return np.zeros(int(sample_rate * seconds), dtype=np.float32)


def test_normal_completes_quickly(fresh_manager):
    """Нормальный путь: stream сразу active=False → polling выходит за 1-2 итерации."""
    with patch("rob_box_voice.audio_playback_manager.sd") as sd:
        sd.get_stream.return_value.active = False
        sd.stop = MagicMock()
        start = time.monotonic()
        ok = fresh_manager.play_audio(
            _silent_silence(16000), 16000, blocking=True, timeout=5.0
        )
        elapsed = time.monotonic() - start
    assert ok is True
    assert elapsed < 1.0, f"normal path занял {elapsed:.1f}с — polling неэффективен"
    assert fresh_manager._current_stream is None


def test_hang_recovers_within_timeout(fresh_manager):
    """sd.wait() зависает → polling должен вернуться через ~8с, не вечно."""
    with patch("rob_box_voice.audio_playback_manager.sd") as sd:
        sd.get_stream.return_value.active = True  # всегда "играет"
        sd.stop = MagicMock()
        start = time.monotonic()
        ok = fresh_manager.play_audio(
            _silent_silence(16000), 16000, blocking=True, timeout=5.0
        )
        elapsed = time.monotonic() - start
    assert elapsed < 12.0, f"hang не восстановлен ({elapsed:.1f}с) — polling timeout не работает"
    assert sd.stop.called, "принудительный sd.stop() не вызван"
    assert fresh_manager._current_stream is None


def test_lock_released_after_hang(fresh_manager):
    """После hang — следующий play_audio() должен пройти (не зависнуть на lock.acquire)."""
    with patch("rob_box_voice.audio_playback_manager.sd") as sd:
        sd.get_stream.return_value.active = True
        sd.stop = MagicMock()
        fresh_manager.play_audio(
            _silent_silence(16000), 16000, blocking=True, timeout=5.0
        )
    # второй вызов с активным stream — должен пройти
    with patch("rob_box_voice.audio_playback_manager.sd") as sd:
        sd.get_stream.return_value.active = False
        start = time.monotonic()
        ok = fresh_manager.play_audio(
            _silent_silence(16000), 16000, blocking=True, timeout=2.0
        )
        elapsed = time.monotonic() - start
    assert ok is True
    assert elapsed < 1.0, "lock не освободился после hang — второй вызов завис"
    assert fresh_manager._playback_lock.locked() is False


def test_nonblocking_returns_immediately(fresh_manager):
    """blocking=False — НЕ должен крутить polling-цикл."""
    with patch("rob_box_voice.audio_playback_manager.sd") as sd:
        sd.get_stream.return_value.active = True
        sd.stop = MagicMock()
        start = time.monotonic()
        ok = fresh_manager.play_audio(
            _silent_silence(16000), 16000, blocking=False, timeout=5.0
        )
        elapsed = time.monotonic() - start
    assert ok is True
    # non-blocking: sd.wait() не вызывается, но sleep(0.05) после play есть
    assert elapsed < 0.5, f"non-blocking path занял {elapsed:.1f}с"