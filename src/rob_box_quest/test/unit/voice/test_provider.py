"""Unit-тесты VoiceProvider (YandexTTSProvider + HardcodedVoiceProvider + build_default_provider)."""

import os

import pytest

from rob_box_quest.voice import (
    HardcodedVoiceProvider,
    VoiceCatalog,
    YandexTTSProvider,
    build_default_provider,
    synthesize_preview,
)
from rob_box_quest.voice.provider import _read_yandex_creds


class TestHardcodedVoiceProvider:
    def test_synthesize_returns_none(self):
        # Hardcoded-провайдер — заглушка; для реальных preview нужны Yandex
        # credentials. synthesize_preview превратит None в ERROR INTERNAL.
        from rob_box_quest.voice import PRESETS

        result = HardcodedVoiceProvider.synthesize(
            voice_id="anton",
            text="hello",
            preset=PRESETS[0],
        )
        assert result is None


class TestBuildDefaultProvider:
    def setup_method(self):
        self._saved_key = os.environ.get("YANDEX_TTS_APIKEY")
        self._saved_folder = os.environ.get("YANDEX_TTS_FOLDER_ID")
        self._saved_force = os.environ.get("FORCE_HARDCODED_TTS")

    def teardown_method(self):
        for k, v in (
            ("YANDEX_TTS_APIKEY", self._saved_key),
            ("YANDEX_TTS_FOLDER_ID", self._saved_folder),
            ("FORCE_HARDCODED_TTS", self._saved_force),
        ):
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    def test_no_env_returns_hardcoded(self):
        os.environ.pop("YANDEX_TTS_APIKEY", None)
        os.environ.pop("YANDEX_TTS_FOLDER_ID", None)
        os.environ.pop("FORCE_HARDCODED_TTS", None)
        p = build_default_provider()
        assert isinstance(p, HardcodedVoiceProvider)

    def test_force_hardcoded_overrides_env(self):
        os.environ["YANDEX_TTS_APIKEY"] = "fake"
        os.environ["YANDEX_TTS_FOLDER_ID"] = "fake-folder"
        os.environ["FORCE_HARDCODED_TTS"] = "1"
        p = build_default_provider()
        assert isinstance(p, HardcodedVoiceProvider)

    def test_with_credentials_returns_yandex(self):
        os.environ["YANDEX_TTS_APIKEY"] = "fake-api-key"
        os.environ["YANDEX_TTS_FOLDER_ID"] = "fake-folder"
        os.environ.pop("FORCE_HARDCODED_TTS", None)
        p = build_default_provider()
        assert isinstance(p, YandexTTSProvider)

    def test_only_apikey_returns_hardcoded(self):
        # Если только api_key без folder_id — не создаём Yandex (нужна пара).
        os.environ["YANDEX_TTS_APIKEY"] = "fake-api-key"
        os.environ.pop("YANDEX_TTS_FOLDER_ID", None)
        os.environ.pop("FORCE_HARDCODED_TTS", None)
        p = build_default_provider()
        assert isinstance(p, HardcodedVoiceProvider)


class TestReadYandexCreds:
    def setup_method(self):
        self._k = os.environ.get("YANDEX_TTS_APIKEY")
        self._f = os.environ.get("YANDEX_TTS_FOLDER_ID")

    def teardown_method(self):
        for k, v in (("YANDEX_TTS_APIKEY", self._k), ("YANDEX_TTS_FOLDER_ID", self._f)):
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    def test_no_creds_returns_none(self):
        os.environ.pop("YANDEX_TTS_APIKEY", None)
        os.environ.pop("YANDEX_TTS_FOLDER_ID", None)
        assert _read_yandex_creds() is None

    def test_partial_creds_returns_none(self):
        os.environ["YANDEX_TTS_APIKEY"] = "k"
        os.environ.pop("YANDEX_TTS_FOLDER_ID", None)
        assert _read_yandex_creds() is None

    def test_full_creds_returned(self):
        os.environ["YANDEX_TTS_APIKEY"] = "k"
        os.environ["YANDEX_TTS_FOLDER_ID"] = "f"
        c = _read_yandex_creds()
        assert c is not None
        assert c.api_key == "k"
        assert c.folder_id == "f"


class TestSynthesizePreview:
    def test_unknown_voice_returns_none(self):
        cat = VoiceCatalog.default()
        result = synthesize_preview(
            catalog=cat,
            provider=HardcodedVoiceProvider,
            voice_id="no_such_voice",
            text="hi",
            preset_id="standard",
        )
        assert result is None

    def test_unknown_preset_falls_back_to_standard(self):
        # Неизвестный preset_id не должен блокировать preview — fallback
        # на "standard" в synthesize_preview.
        cat = VoiceCatalog.default()
        first_voice_id = cat.voices[0].id
        # Hardcoded-провайдер возвращает None, но это уже после валидации
        # каталога. Главное: не получили None из-за неизвестного preset.
        # Меняем провайдер на stub который возвращает bytes:
        result = synthesize_preview(
            catalog=cat,
            provider=_StubProvider(b"opus_bytes"),
            voice_id=first_voice_id,
            text="hi",
            preset_id="no_such_preset",
        )
        assert result == b"opus_bytes"

    def test_happy_path(self):
        cat = VoiceCatalog.default()
        first_voice_id = cat.voices[0].id
        result = synthesize_preview(
            catalog=cat,
            provider=_StubProvider(b"\x00\x01\x02"),
            voice_id=first_voice_id,
            text="привет",
            preset_id="friendly",
        )
        assert result == b"\x00\x01\x02"


class _StubProvider:
    """Минимальный stub VoiceProvider для synthesize_preview."""

    def __init__(self, payload):
        self._payload = payload

    def synthesize(self, *, voice_id, text, preset):
        return self._payload