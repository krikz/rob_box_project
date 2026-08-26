"""Unit-тесты VoiceCatalog + ENV override + presets."""

import json
import os

import pytest

from rob_box_quest.voice import (
    PRESETS,
    VoiceCatalog,
    default_catalog,
)
from rob_box_quest.voice.catalog import _ENV_VAR_VOICES


class TestFallbackCatalog:
    def test_default_has_at_least_4_voices(self):
        # Acceptance criterion задачи t_7eba64d9: "≥4 голосов".
        cat = VoiceCatalog.default()
        assert len(cat.voices) >= 4

    def test_default_voice_ids_lowercase_snake_case(self):
        # Parent decision: voice_id формат = lowercase snake_case.
        cat = VoiceCatalog.default()
        for v in cat.voices:
            assert v.id == v.id.lower().replace(" ", "_")

    def test_default_presets_include_required(self):
        # 4 пресета из задачи §1.
        cat = VoiceCatalog.default()
        ids = {p.id for p in cat.presets}
        assert {"standard", "friendly", "authoritative", "whisper"} <= ids

    def test_module_presets_match_catalog_default(self):
        # PRESETS module-level tuple используется в default.
        cat = VoiceCatalog.default()
        assert cat.presets == PRESETS

    def test_get_voice_returns_object(self):
        cat = VoiceCatalog.default()
        v = cat.get_voice(cat.voices[0].id)
        assert v is not None
        assert v.id == cat.voices[0].id

    def test_get_voice_missing_returns_none(self):
        cat = VoiceCatalog.default()
        assert cat.get_voice("no_such_voice_xyz") is None

    def test_get_preset_standard(self):
        cat = VoiceCatalog.default()
        p = cat.get_preset("standard")
        assert p is not None and p.id == "standard"

    def test_get_preset_missing_returns_none(self):
        cat = VoiceCatalog.default()
        assert cat.get_preset("nope") is None

    def test_to_list_payload_shape(self):
        cat = VoiceCatalog.default()
        payload = cat.to_list_payload()
        assert "voices" in payload
        assert "presets" in payload
        assert isinstance(payload["voices"], list)
        v0 = payload["voices"][0]
        assert {"id", "name", "gender", "language", "description"} <= v0.keys()
        p0 = payload["presets"][0]
        assert {"id", "name", "description"} <= p0.keys()


class TestEnvOverride:
    def setup_method(self):
        self._saved = os.environ.get(_ENV_VAR_VOICES)

    def teardown_method(self):
        if self._saved is None:
            os.environ.pop(_ENV_VAR_VOICES, None)
        else:
            os.environ[_ENV_VAR_VOICES] = self._saved

    def test_env_valid_json_overrides_catalog(self):
        custom = [
            {"id": "voice_a", "name": "Voice A", "gender": "female",
             "language": "en-US", "description": "custom"},
            {"id": "voice_b", "name": "Voice B", "gender": "male",
             "language": "en-US", "description": "custom"},
            {"id": "voice_c", "name": "Voice C", "gender": "male",
             "language": "en-US", "description": "custom"},
            {"id": "voice_d", "name": "Voice D", "gender": "female",
             "language": "en-US", "description": "custom"},
        ]
        os.environ[_ENV_VAR_VOICES] = json.dumps(custom)
        cat = VoiceCatalog.from_env()
        ids = {v.id for v in cat.voices}
        assert "voice_a" in ids

    def test_env_bad_json_falls_back_to_default(self):
        os.environ[_ENV_VAR_VOICES] = "{not valid json"
        cat = VoiceCatalog.from_env()
        # Должно вернуться default (≥4 голосов), без raise.
        assert len(cat.voices) >= 4

    def test_env_too_few_voices_falls_back(self):
        os.environ[_ENV_VAR_VOICES] = json.dumps([
            {"id": "only_one"},
        ])
        cat = VoiceCatalog.from_env()
        # <4 голосов в ENV → default fallback
        assert len(cat.voices) >= 4

    def test_env_not_a_list_falls_back(self):
        os.environ[_ENV_VAR_VOICES] = json.dumps({"not": "a list"})
        cat = VoiceCatalog.from_env()
        assert len(cat.voices) >= 4

    def test_env_no_var_uses_default(self):
        os.environ.pop(_ENV_VAR_VOICES, None)
        cat = VoiceCatalog.from_env()
        assert len(cat.voices) >= 4

    def test_custom_too_few_raises_on_direct_construct(self):
        # Если кто-то обходит from_env и конструирует напрямую <4 → raise.
        with pytest.raises(ValueError):
            VoiceCatalog(voices=())


class TestCatalogSingleton:
    def test_default_catalog_is_cached(self):
        # default_catalog() возвращает singleton, а не новый инстанс.
        c1 = default_catalog()
        c2 = default_catalog()
        assert c1 is c2