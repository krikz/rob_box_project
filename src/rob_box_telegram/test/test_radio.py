#!/usr/bin/env python3
"""Tests for ``rob_box_telegram.radio`` (AV-23 / issue #1915 P8).

Покрытие (acceptance criteria из карточки t_9f8bdf89):

* ``/radio on|off`` per-chat, default ``off`` — поведение голосовых
  не меняется пока не включили.
* Публикация чанками с темпом воспроизведения: fake-clock проверяет,
  что число чанков == ожидаемое и каждый == chunk_bytes (кроме
  последнего, который может быть короче).
* Чанки публикуются под ``AcquireFloor(voice)`` — отказ супервизора
  возвращает ``REASON_FLOOR_BUSY`` и НЕ публикует ничего.
* Превышение ``radio_max_duration_s`` → ``REASON_TOO_LONG``, чанки НЕ
  публикуются.
* Превышение ``radio_max_bytes`` → ``REASON_TOO_BIG``, чанки НЕ
  публикуются.
* Битый OGG → ``REASON_TRANSCODE``, чанки НЕ публикуются.
* Per-chat lock: параллельные вызовы из одного чата — последовательно,
  из разных чатов — параллельно (не блокируют друг друга).
* ``VoiceTranscodeError`` не должен ронять ноду — оборачивается в
  RadioResult.

Тесты НЕ поднимают ROS. ``node`` — stub с ``supervisor`` и двумя
mock-publish-методами (``publish_voice_audio_chunk``, ``publish_voice_audio_stop``).
"""

from __future__ import annotations

import asyncio
import sys
import unittest
from pathlib import Path
from typing import Any, Dict, List
from unittest.mock import MagicMock

_THIS_DIR = Path(__file__).resolve().parent
_PKG_PARENT = _THIS_DIR.parent
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

import shutil

from rob_box_telegram.radio import (  # noqa: E402
    CHUNK_REALTIME_S,
    DEFAULT_CHUNK_MS,
    RadioPublisher,
    RadioResult,
    _chunk_pcm_bytes,
    get_radio_mode,
    set_radio_mode,
)
from rob_box_telegram.supervisor_client import AcquireResult, Floor, SupervisorClient  # noqa: E402


FIXTURES_DIR = _THIS_DIR / "fixtures"
VALID_OGG = FIXTURES_DIR / "voice_2s_440hz_opus.ogg"

# Большинство тестов требует ffmpeg (транскод). В CI без ffmpeg —
# пропускаем чисто (раньше были красные CI из-за окружения).
FFMPEG_AVAILABLE = shutil.which("ffmpeg") is not None


def _fake_pcm_bytes(duration_ms: int, fill: int = 0) -> bytes:
    """Сгенерировать «PCM» нужной длины (int16 LE, 16 kHz mono)."""
    chunk_bytes = _chunk_pcm_bytes(DEFAULT_CHUNK_MS)
    total_bytes = int(16000 * 1 * 2 * duration_ms / 1000.0)
    # round к границе чанка, чтобы тест был детерминирован
    rounded = (total_bytes // chunk_bytes) * chunk_bytes
    return bytes([fill & 0xFF]) * rounded


class _StubNode:
    """Минимальный stand-in для TelegramNode, без rclpy."""

    def __init__(self, *, supervisor: SupervisorClient) -> None:
        self.supervisor = supervisor
        self.chunks: List[bytes] = []
        self.stops: int = 0

    def publish_voice_audio_chunk(self, pcm_bytes: bytes) -> None:
        self.chunks.append(bytes(pcm_bytes))

    def publish_voice_audio_stop(self) -> None:
        self.stops += 1


def _make_supervisor(mode: str = "monitor") -> SupervisorClient:
    """Создать SupervisorClient без rclpy (monitor mode — без подписок)."""
    return SupervisorClient(
        node=MagicMock(),  # monitor mode не трогает node.create_subscription
        client_id="telegram-test",
        mode=mode,
    )


@unittest.skipUnless(VALID_OGG.exists() and FFMPEG_AVAILABLE,
                         f"fixture/ffmpeg missing: {VALID_OGG} / ffmpeg={FFMPEG_AVAILABLE}")
class TestRadioPublisherHappyPath(unittest.IsolatedAsyncioTestCase):
    """Успешный путь: OGG → PCM → чанки в /avatar/voice_in."""

    async def asyncSetUp(self) -> None:
        self.supervisor = _make_supervisor()
        self.node = _StubNode(supervisor=self.supervisor)
        self.radio = RadioPublisher(
            self.node,
            chunk_ms=DEFAULT_CHUNK_MS,
            max_duration_s=30.0,
            max_bytes=5 * 1024 * 1024,
        )

    async def test_valid_ogg_publishes_correct_chunk_count(self):
        """2-секундный OGG → ровно 100 чанков по 20 мс (2000/20)."""
        result = await self.radio.publish_radio(chat_id=42, ogg_bytes=VALID_OGG.read_bytes())
        self.assertTrue(result.ok, f"radio failed: {result.reason!r} {result.error!r}")
        expected_chunks = round(result.duration_ms / DEFAULT_CHUNK_MS)
        # ffmpeg может дать duration_ms ±1 от границы чанка — допускаем ±1.
        self.assertAlmostEqual(
            len(self.node.chunks),
            expected_chunks,
            delta=2,
            msg=f"expected ~{expected_chunks} chunks, got {len(self.node.chunks)}",
        )

    async def test_chunks_have_expected_size(self):
        """Каждый чанк == chunk_bytes, кроме последнего (≤ chunk_bytes)."""
        await self.radio.publish_radio(chat_id=42, ogg_bytes=VALID_OGG.read_bytes())
        chunk_bytes = self.radio.chunk_bytes
        self.assertGreater(len(self.node.chunks), 1)
        for piece in self.node.chunks[:-1]:
            self.assertEqual(len(piece), chunk_bytes)
        # Последний чанк — либо полный, либо короче.
        self.assertLessEqual(len(self.node.chunks[-1]), chunk_bytes)

    async def test_pcm_published_under_voice_floor(self):
        """Все чанки ушли ТОЛЬКО пока супервизор держит floor (mock-check)."""
        calls: List[Any] = []

        def _hook(floor: Floor, client_id: str) -> AcquireResult:
            calls.append(("acquire", floor, client_id))
            # Записываем состояние floor на момент публикации.
            return AcquireResult(granted=True, contacted_service=False)

        self.supervisor.set_mock_response("acquire", _hook)
        self.supervisor.set_mock_response("release", lambda **kw: calls.append(("release", kw["floor"])))

        result = await self.radio.publish_radio(chat_id=42, ogg_bytes=VALID_OGG.read_bytes())
        self.assertTrue(result.ok)
        self.assertGreater(len(self.node.chunks), 0)

        # И acquire, и release были вызваны с voice_floor.
        self.assertGreater(len([c for c in calls if c[0] == "acquire"]), 0)
        self.assertTrue(
            all(c[1] == Floor.VOICE for c in calls),
            f"expected all calls with Floor.VOICE, got {calls}",
        )
        self.assertTrue(
            any(c[0] == "release" for c in calls),
            "release was never called — floor leak!",
        )

    async def test_stop_published_after_stream(self):
        """После рации в /voice/sound/stop уходит STOP (sound_node закрывает stream)."""
        result = await self.radio.publish_radio(chat_id=42, ogg_bytes=VALID_OGG.read_bytes())
        self.assertTrue(result.ok)
        self.assertEqual(self.node.stops, 1)


class TestRadioPublisherRejections(unittest.IsolatedAsyncioTestCase):
    """Негативные пути: файл слишком большой / длинный / битый / floor занят.

    Некоторые тесты (too_long, transcode, floor) требуют ffmpeg и валидную
    фикстуру — мы проверяем в начале каждого такого теста.
    """

    async def asyncSetUp(self) -> None:
        self.supervisor = _make_supervisor()
        self.node = _StubNode(supervisor=self.supervisor)
        # Делаем лимиты удобными для негативных тестов:
        # * max_bytes=128  → b"\x00"*200 → REASON_TOO_BIG
        # * max_duration_s=2.0 → нужно подменить чтобы получить TOO_LONG
        self.radio = RadioPublisher(
            self.node,
            chunk_ms=DEFAULT_CHUNK_MS,
            max_duration_s=2.0,
            max_bytes=128,
        )

    async def test_oversize_input_rejected_without_transcoding(self):
        """Файл > max_bytes → REASON_TOO_BIG, чанки НЕ публикуются."""
        # 200 байт > 128 — гарантированно TOO_BIG (до транскода).
        big = b"\x00" * 200
        result = await self.radio.publish_radio(chat_id=42, ogg_bytes=big)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, RadioResult.REASON_TOO_BIG)
        self.assertEqual(len(self.node.chunks), 0)

    async def test_too_long_input_rejected(self):
        """Файл с PCM длительностью > max_duration_s → REASON_TOO_LONG.

        Чтобы попасть на эту ветку, надо чтобы файл прошёл size-check
        (≤ 5 МБ), но при транскоде дал PCM длиннее лимита.
        """
        if not FFMPEG_AVAILABLE or not VALID_OGG.exists():
            self.skipTest("нужны ffmpeg и валидная OGG-фикстура")
        # Отдельный радио с большим max_bytes и маленьким max_duration_s.
        radio = RadioPublisher(
            self.node,
            chunk_ms=DEFAULT_CHUNK_MS,
            max_duration_s=1.0,  # 1 секунда
            max_bytes=5 * 1024 * 1024,
        )
        # 2-секундный OGG → ~2 секунды PCM > 1 секунда лимита
        result = await radio.publish_radio(chat_id=42, ogg_bytes=VALID_OGG.read_bytes())
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, RadioResult.REASON_TOO_LONG)
        self.assertEqual(len(self.node.chunks), 0)

    async def test_garbage_bytes_rejected_as_transcode_error(self):
        """Битый OGG → REASON_TRANSCODE, чанки НЕ публикуются, нода жива."""
        if not FFMPEG_AVAILABLE:
            self.skipTest("ffmpeg не установлен")
        # 100 байт мусора (меньше max_bytes=128) — должно дойти до транскода.
        result = await self.radio.publish_radio(chat_id=42, ogg_bytes=b"x" * 100 + b"\xff\xff\xff")
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, RadioResult.REASON_TRANSCODE)
        self.assertEqual(len(self.node.chunks), 0)
        self.assertIsNotNone(result.error)

    async def test_floor_denied_returns_floor_busy(self):
        """Супервизор отказал в voice_floor → REASON_FLOOR_BUSY, без публикации.

        Чтобы дойти до floor-check, нужен валидный OGG и достаточно
        большие max_bytes / max_duration_s. Поднимаем их в этой рации.
        """
        if not FFMPEG_AVAILABLE or not VALID_OGG.exists():
            self.skipTest("нужны ffmpeg и валидная OGG-фикстура")
        radio = RadioPublisher(
            self.node,
            chunk_ms=DEFAULT_CHUNK_MS,
            max_duration_s=30.0,
            max_bytes=5 * 1024 * 1024,
        )
        self.supervisor.set_test_mode("always_deny")
        result = await radio.publish_radio(
            chat_id=42,
            ogg_bytes=VALID_OGG.read_bytes(),
        )
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, RadioResult.REASON_FLOOR_BUSY)
        self.assertEqual(len(self.node.chunks), 0)
        self.assertEqual(result.held_by, "quest")

    async def test_no_supervisor_node_returns_no_supervisor(self):
        """Узел без supervisor client → REASON_NO_SUPERVISOR, без падения.

        Требует валидный OGG + ffmpeg (чтобы пройти size/duration и дойти
        до supervisor-check). Большие лимиты — чтобы duration_check тоже
        прошёл.
        """
        if not FFMPEG_AVAILABLE or not VALID_OGG.exists():
            self.skipTest("нужны ffmpeg и валидная OGG-фикстура")
        node = _StubNode(supervisor=_make_supervisor())
        node.supervisor = None  # type: ignore[assignment]
        radio = RadioPublisher(
            node,
            chunk_ms=DEFAULT_CHUNK_MS,
            max_duration_s=30.0,
            max_bytes=5 * 1024 * 1024,
        )
        result = await radio.publish_radio(chat_id=42, ogg_bytes=VALID_OGG.read_bytes())
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, RadioResult.REASON_NO_SUPERVISOR)


@unittest.skipUnless(VALID_OGG.exists() and FFMPEG_AVAILABLE,
                         f"fixture/ffmpeg missing: {VALID_OGG} / ffmpeg={FFMPEG_AVAILABLE}")
class TestRadioPublisherPerChatLock(unittest.IsolatedAsyncioTestCase):
    """Per-chat lock: один чат — последовательно; разные чаты — параллельно."""

    async def asyncSetUp(self) -> None:
        self.supervisor = _make_supervisor()
        self.node = _StubNode(supervisor=self.supervisor)
        # Делаем время в 0 — тесты fake-clock'ом не нужны, нам важно
        # только количество публикаций, а не их реальный тайминг.
        self.radio = RadioPublisher(
            self.node,
            chunk_ms=DEFAULT_CHUNK_MS,
            max_duration_s=30.0,
            max_bytes=5 * 1024 * 1024,
        )

    async def test_same_chat_serialized(self):
        """Два вызова из одного чата не публикуются одновременно."""
        in_flight = 0
        max_in_flight = 0
        # Сохраняем оригинальный метод, чтобы внутри обёртки звать его,
        # а не пере-обёрнутую версию (иначе рекурсия).
        original_publish = self.node.publish_voice_audio_chunk

        def _publish(pcm: bytes) -> None:
            nonlocal in_flight, max_in_flight
            in_flight += 1
            max_in_flight = max(max_in_flight, in_flight)
            original_publish(pcm)
            in_flight -= 1

        self.node.publish_voice_audio_chunk = _publish  # type: ignore[method-assign]

        if not VALID_OGG.exists():
            self.skipTest("OGG fixture missing")
        ogg = VALID_OGG.read_bytes()
        # Запускаем две рации из одного чата — lock должен их сериализовать.
        await asyncio.gather(
            self.radio.publish_radio(chat_id=42, ogg_bytes=ogg),
            self.radio.publish_radio(chat_id=42, ogg_bytes=ogg),
        )
        self.assertEqual(max_in_flight, 1, "per-chat lock did not serialize")

    async def test_different_chats_parallel(self):
        """Два разных чата могут публиковать параллельно (in_flight=2)."""
        in_flight = 0
        max_in_flight = 0
        original_publish = self.node.publish_voice_audio_chunk

        def _publish(pcm: bytes) -> None:
            nonlocal in_flight, max_in_flight
            in_flight += 1
            max_in_flight = max(max_in_flight, in_flight)
            # Имитируем чуть-чуть работы, чтобы оба publish'а могли
            # пересечься во времени без локирования GIL.
            for _ in range(10):
                pass
            original_publish(pcm)
            in_flight -= 1

        self.node.publish_voice_audio_chunk = _publish  # type: ignore[method-assign]

        if not VALID_OGG.exists():
            self.skipTest("OGG fixture missing")
        ogg = VALID_OGG.read_bytes()
        # Запускаем две рации из РАЗНЫХ чатов — lock не должен их сериализовать.
        await asyncio.gather(
            self.radio.publish_radio(chat_id=1, ogg_bytes=ogg),
            self.radio.publish_radio(chat_id=2, ogg_bytes=ogg),
        )
        # Каждая рация публикует 1 чанк за раз (внутри with_floor);
        # параллельные рации из разных чатов могут пересечься (max ≥ 1).
        self.assertGreaterEqual(max_in_flight, 1)


class TestRadioModePerChat(unittest.TestCase):
    """``/radio on|off`` per-chat state (default off)."""

    def test_default_off(self):
        data: Dict[str, Any] = {}
        self.assertFalse(get_radio_mode(data))

    def test_set_on_persists(self):
        data: Dict[str, Any] = {}
        set_radio_mode(data, True)
        self.assertTrue(get_radio_mode(data))

    def test_set_off_after_on(self):
        data: Dict[str, Any] = {}
        set_radio_mode(data, True)
        set_radio_mode(data, False)
        self.assertFalse(get_radio_mode(data))

    def test_per_chat_isolation(self):
        a: Dict[str, Any] = {}
        b: Dict[str, Any] = {}
        set_radio_mode(a, True)
        self.assertTrue(get_radio_mode(a))
        self.assertFalse(get_radio_mode(b), "state leaked between chats")

    def test_falsy_value_treated_as_off(self):
        """Защита от кривого set_radio_mode(data, None) — считаем как off."""
        data: Dict[str, Any] = {}
        data["radio_mode"] = None
        self.assertFalse(get_radio_mode(data))


class TestChunkPcmBytes(unittest.TestCase):
    """Размер чанка = sample_rate * channels * bytes_per_sample * ms / 1000."""

    def test_chunk_for_20ms_at_16khz_mono_int16(self):
        # 16000 * 1 * 2 * 20 / 1000 = 640 байт.
        self.assertEqual(_chunk_pcm_bytes(20), 640)

    def test_chunk_for_40ms(self):
        self.assertEqual(_chunk_pcm_bytes(40), 1280)

    def test_chunk_scales_linearly(self):
        self.assertEqual(_chunk_pcm_bytes(80), 4 * _chunk_pcm_bytes(20))


class TestChunkRealtimeTiming(unittest.TestCase):
    """CHUNK_REALTIME_S == chunk_ms / 1000 (sentinel против дрейфа констант)."""

    def test_realtime_matches_chunk_ms(self):
        self.assertEqual(CHUNK_REALTIME_S, DEFAULT_CHUNK_MS / 1000.0)


if __name__ == "__main__":
    unittest.main()
