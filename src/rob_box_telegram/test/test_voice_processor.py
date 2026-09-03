#!/usr/bin/env python3
"""Tests for rob_box_telegram.voice_processor module."""

import unittest
from unittest.mock import AsyncMock, patch, MagicMock

from rob_box_telegram.voice_processor import transcribe_voice


class TestTranscribeVoice(unittest.IsolatedAsyncioTestCase):
    """Tests for voice transcription dispatch."""

    async def test_unknown_method_returns_none(self):
        result = await transcribe_voice(b"audio", method="unknown")
        self.assertIsNone(result)

    @patch("rob_box_telegram.voice_processor._transcribe_vosk", new_callable=AsyncMock)
    @patch("rob_box_telegram.voice_processor._transcribe_yandex", new_callable=AsyncMock)
    async def test_routes_to_yandex(self, mock_yandex, mock_vosk):
        mock_yandex.return_value = "привет"
        result = await transcribe_voice(b"ogg_data", method="yandex", language="ru-RU")
        mock_yandex.assert_called_once_with(b"ogg_data", "ru-RU")
        mock_vosk.assert_not_called()
        self.assertEqual(result, "привет")

    @patch("rob_box_telegram.voice_processor._transcribe_vosk", new_callable=AsyncMock)
    @patch("rob_box_telegram.voice_processor._transcribe_yandex", new_callable=AsyncMock)
    async def test_yandex_failure_falls_back_to_vosk(self, mock_yandex, mock_vosk):
        # 2026-09-03: Yandex balance ran out — must degrade to the local
        # model instead of the caller seeing "не удалось распознать".
        mock_yandex.return_value = None
        mock_vosk.return_value = "привет"
        result = await transcribe_voice(b"ogg_data", method="yandex", language="ru-RU")
        mock_yandex.assert_called_once_with(b"ogg_data", "ru-RU")
        mock_vosk.assert_called_once_with(b"ogg_data")
        self.assertEqual(result, "привет")

    @patch("rob_box_telegram.voice_processor._transcribe_whisper", new_callable=AsyncMock)
    async def test_routes_to_whisper(self, mock_whisper):
        mock_whisper.return_value = "hello"
        result = await transcribe_voice(b"ogg_data", method="whisper", language="en-US")
        mock_whisper.assert_called_once_with(b"ogg_data", "en-US")
        self.assertEqual(result, "hello")


class TestYandexSTT(unittest.IsolatedAsyncioTestCase):
    """Tests for Yandex STT backend."""

    @patch.dict("os.environ", {"YANDEX_API_KEY": "", "YANDEX_FOLDER_ID": ""})
    async def test_missing_api_key(self):
        from rob_box_telegram.voice_processor import _transcribe_yandex

        result = await _transcribe_yandex(b"audio", "ru-RU")
        self.assertIsNone(result)


class TestVoskSTT(unittest.IsolatedAsyncioTestCase):
    """Tests for the local Vosk STT fallback."""

    @patch("rob_box_telegram.voice_processor.ogg_to_pcm16k")
    async def test_transcode_failure_returns_none(self, mock_transcode):
        from rob_box_telegram.voice_processor import VoiceTranscodeError, _transcribe_vosk

        mock_transcode.side_effect = VoiceTranscodeError("corrupt ogg")
        result = await _transcribe_vosk(b"not-really-ogg")
        self.assertIsNone(result)

    @patch("rob_box_telegram.voice_processor._load_vosk_model")
    def test_model_missing_returns_none(self, mock_load):
        from rob_box_telegram.voice_processor import _recognize_vosk_sync

        mock_load.return_value = None
        result = _recognize_vosk_sync(b"\x00\x00" * 100)
        self.assertIsNone(result)


class TestWhisperSTT(unittest.IsolatedAsyncioTestCase):
    """Tests for Whisper STT backend."""

    @patch.dict("os.environ", {"OPENAI_API_KEY": ""})
    async def test_missing_api_key(self):
        from rob_box_telegram.voice_processor import _transcribe_whisper

        result = await _transcribe_whisper(b"audio", "ru-RU")
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
