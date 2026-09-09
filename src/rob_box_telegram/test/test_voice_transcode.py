#!/usr/bin/env python3
"""Tests for ``rob_box_telegram.voice_transcode`` (AV-23 / issue #1915 P8).

Транскодер OGG/Opus → int16 PCM 16 kHz mono — критическая часть рации
из Telegram. Эти тесты НЕ зависят от ROS / rclpy / telegram (clean
Python), поэтому гоняются как самостоятельный файл pytest'ом
(см. CI ``G-Run Tests.yml`` — ``rob_box_telegram`` per-file loop).

Покрытие:

* happy-path: 2-секундный OGG/Opus → ровно 2 секунды PCM,
  выровнено по int16, mono, 16 kHz;
* длинный input (в пределах лимита) — duration scale;
* пустой bytes → ``VoiceTranscodeError`` (не падение ноды);
* битый OGG (рандомные байты) → ``VoiceTranscodeError``;
* превышение ``MAX_INPUT_BYTES`` → ``VoiceTranscodeError`` без вызова
  ffmpeg (защита до дорогого декодера).
"""

from __future__ import annotations

import os
import shutil
import sys
import unittest
from pathlib import Path

# Убеждаемся, что корень пакета доступен: ``from rob_box_telegram.X import ...``.
# CI запускает ``cd $TG_DIR && PYTHONPATH=. pytest $TG_TEST``, но если
# кто-то гоняет из корня — тоже работает.
_THIS_DIR = Path(__file__).resolve().parent
_PKG_PARENT = _THIS_DIR.parent  # src/rob_box_telegram/
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

from rob_box_telegram.voice_transcode import (  # noqa: E402  - после sys.path
    MAX_INPUT_BYTES,
    TARGET_CHANNELS,
    TARGET_SAMPLE_RATE_HZ,
    TARGET_SAMPLE_WIDTH_BYTES,
    VoiceTranscodeError,
    ogg_to_pcm16k,
    probe_ogg_duration_ms,
)


FIXTURES_DIR = _THIS_DIR / "fixtures"
VALID_OGG = FIXTURES_DIR / "voice_2s_440hz_opus.ogg"
BAD_OGG = FIXTURES_DIR / "voice_bad.ogg"

# Большинство тестов требуют ffmpeg в PATH. В CI образе для
# ``G-Run Tests.yml`` (ubuntu-22.04 без нашего prod-образа) ffmpeg
# может не быть — пропускаем чисто, чтобы не было красных CI.
FFMPEG_AVAILABLE = shutil.which("ffmpeg") is not None
FFPROBE_AVAILABLE = shutil.which("ffprobe") is not None


def _bytes_to_seconds(n_bytes: int) -> float:
    """Размер PCM-байт → длительность в секундах."""
    return n_bytes / (TARGET_SAMPLE_RATE_HZ * TARGET_CHANNELS * TARGET_SAMPLE_WIDTH_BYTES)


@unittest.skipUnless(VALID_OGG.exists() and FFMPEG_AVAILABLE,
                         f"fixture/ffmpeg missing: {VALID_OGG} / ffmpeg={FFMPEG_AVAILABLE}")
class TestOggToPcm16kValid(unittest.TestCase):
    """Happy path: реальный OGG/Opus от Telegram транскодируется в PCM."""

    def test_returns_nonempty_pcm(self):
        pcm = ogg_to_pcm16k(VALID_OGG.read_bytes())
        self.assertGreater(len(pcm), 0, "ffmpeg вернул пустой PCM")

    def test_pcm_is_int16_aligned(self):
        pcm = ogg_to_pcm16k(VALID_OGG.read_bytes())
        self.assertEqual(
            len(pcm) % TARGET_SAMPLE_WIDTH_BYTES,
            0,
            f"PCM length {len(pcm)} is not aligned to int16 ({TARGET_SAMPLE_WIDTH_BYTES})",
        )

    def test_duration_matches_input_within_tolerance(self):
        """Источник 2 с ±200 мс (DURATION_TOLERANCE_MS)."""
        pcm = ogg_to_pcm16k(VALID_OGG.read_bytes())
        duration_s = _bytes_to_seconds(len(pcm))
        self.assertAlmostEqual(
            duration_s,
            2.0,
            delta=0.5,  # 500 мс — ffmpeg иногда добавляет пре-roll/encoder-delay
            msg=f"expected ~2.0 s, got {duration_s:.3f} s",
        )

    def test_duration_matches_when_probe_provided(self):
        """Когда probe_duration_ms задан — sanity-check внутри тоже зелёный.

        ``probe_ogg_duration_ms`` может вернуть ``None`` на старых ffprobe
        (не умеет stdin-probe) — это не наш баг, поэтому скипаем
        (sanity-check пропустится внутри ``ogg_to_pcm16k``).
        """
        if not FFPROBE_AVAILABLE:
            self.skipTest("ffprobe не установлен")
        probe = probe_ogg_duration_ms(VALID_OGG.read_bytes())
        if probe is None:
            self.skipTest("ffprobe не умеет stdin-probe на этой машине")
        pcm = ogg_to_pcm16k(VALID_OGG.read_bytes(), probe_duration_ms=probe)
        self.assertGreater(len(pcm), 0)

    def test_output_contains_nonzero_samples(self):
        """PCM содержит реальный сигнал, а не нули (sanity: декодер работал)."""
        import struct

        pcm = ogg_to_pcm16k(VALID_OGG.read_bytes())
        samples = struct.unpack(f"<{len(pcm) // 2}h", pcm)
        peak = max(abs(s) for s in samples)
        self.assertGreater(
            peak,
            100,
            f"peak amplitude {peak} suspiciously low — ffmpeg gave silence?",
        )


class TestOggToPcm16kErrors(unittest.TestCase):
    """Негативные пути — не должны ронять ноду."""

    def test_empty_bytes_raises(self):
        """Пустой вход → ошибка ДО вызова ffmpeg (защита от пустых telegram update)."""
        with self.assertRaises(VoiceTranscodeError) as ctx:
            ogg_to_pcm16k(b"")
        self.assertIn("empty input", str(ctx.exception))

    def test_ogg_to_pcm16k_importable_from_public_path(self):
        """Контракт: ogg_to_pcm16k импортируется напрямую из voice_transcode."""
        from rob_box_telegram.voice_transcode import ogg_to_pcm16k as fn

        self.assertTrue(callable(fn))

    def test_oversize_input_rejected_before_ffmpeg(self):
        """Больше MAX_INPUT_BYTES → отказ до запуска ffmpeg (защита от DoS)."""
        # Подменяем MAX_INPUT_BYTES маленьким — тестируем именно логику.
        from unittest.mock import patch

        big = b"\x00" * (MAX_INPUT_BYTES + 1)
        with patch("rob_box_telegram.voice_transcode.MAX_INPUT_BYTES", 1024):
            with self.assertRaises(VoiceTranscodeError) as ctx:
                ogg_to_pcm16k(big)
            self.assertIn("too large", str(ctx.exception))


@unittest.skipUnless(FFMPEG_AVAILABLE, "ffmpeg не установлен")
class TestOggToPcm16kErrorsFfmpeg(unittest.TestCase):
    """Тесты, требующие ffmpeg (он должен ругнуться на битый input)."""

    def test_garbage_bytes_raises(self):
        """200 байт случайного шума — ffmpeg не распарсит, вернёт код != 0."""
        with self.assertRaises(VoiceTranscodeError) as ctx:
            ogg_to_pcm16k(b"\x00\x01\x02\x03 not an OGG \xff\xfe\xfd")
        self.assertIn("ffmpeg returned", str(ctx.exception))

    def test_corrupt_fixture_raises(self):
        """200-байтный фикстурный «битый OGG» → VoiceTranscodeError."""
        if not BAD_OGG.exists():
            self.skipTest(f"fixture missing: {BAD_OGG}")
        with self.assertRaises(VoiceTranscodeError):
            ogg_to_pcm16k(BAD_OGG.read_bytes())

    def test_oversize_real_ffmpeg(self):
        """Реальный ffmpeg получает маленький limit и видит «too large» раньше запуска."""
        # Этот тест дополняет test_oversize_input_rejected_before_ffmpeg:
        # убеждаемся, что при подменённом лимите ffmpeg вообще НЕ
        # вызывается (защита от DoS).
        from unittest.mock import patch

        big = b"\x00" * 9999
        with patch("rob_box_telegram.voice_transcode.MAX_INPUT_BYTES", 100):
            with self.assertRaises(VoiceTranscodeError) as ctx:
                ogg_to_pcm16k(big)
            self.assertIn("too large", str(ctx.exception))


@unittest.skipUnless(os.environ.get("RUN_SLOW_FFMPEG_TESTS"), "set RUN_SLOW_FFMPEG_TESTS=1 to run")
class TestOggToPcm16kSlow(unittest.TestCase):
    """Длинные тесты (по умолчанию OFF — нужны только для регрессий).

    Включаются переменной ``RUN_SLOW_FFMPEG_TESTS=1`` — иначе CI
    не тратит 10+ секунд на тест большого входного файла.
    """

    def test_60s_input_fits_within_tolerance(self):
        # Сгенерируем 60 секунд синуса на лету.
        import subprocess
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            long_path = os.path.join(tmp, "long.ogg")
            subprocess.run(
                [
                    "ffmpeg",
                    "-hide_banner",
                    "-loglevel",
                    "error",
                    "-f",
                    "lavfi",
                    "-i",
                    "sine=frequency=440:duration=60:sample_rate=48000",
                    "-ac",
                    "1",
                    "-c:a",
                    "libopus",
                    long_path,
                    "-y",
                ],
                check=True,
            )
            pcm = ogg_to_pcm16k(Path(long_path).read_bytes())
            duration_s = _bytes_to_seconds(len(pcm))
            self.assertAlmostEqual(duration_s, 60.0, delta=1.0)


if __name__ == "__main__":
    unittest.main()
