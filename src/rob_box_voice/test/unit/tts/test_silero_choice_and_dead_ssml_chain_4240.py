"""#4240 (ADR-0160 PR-3a): выбор Silero реально включает Silero; мёртвая цепочка SSML-просодии убрана.

* Живьём 03.11 (PR #4270): голос silero выбран (``set_provider: 'yandex' → 'silero'``), а цепочка осталась
  ``['yandex', 'minimax', 'silero']`` и синтезировал Yandex. Причина по коду: ``_on_set_provider`` ставил Silero
  первым, а ``_normalize_provider_chain`` тут же переносил его в конец (инвариант «Silero — последний запасной»).
  Событие ``voice_changed`` при этом говорило ``provider=silero`` — неправда о звучащем голосе.
* ``ssml_attributes`` (rate/pitch/volume) и позиционный ``ssml`` в конвейере синтеза с провода больше не
  наполнялись (ADR-0160 §2.3) — убраны вместе с конвертерами ``tts_ssml``.
* Вторая копия MiniMax TTS (``rob_box_harness/tts``) удалена, мёртвый ``normalize_for_tts`` — тоже.
"""
from __future__ import annotations

import inspect
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_node  # noqa: E402
from rob_box_voice.core.voice_owner import VoiceOwner  # noqa: E402
from rob_box_voice.tts_node import TTSNode, chain_for_requested  # noqa: E402

_SRC = _PACKAGE_ROOT.parent


class _ChainNode:
    """Та часть ``TTSNode``, что решает цепочку: настоящие ``_on_set_provider`` и ``_effective_provider``."""

    _on_set_provider = TTSNode._on_set_provider
    _effective_provider = TTSNode._effective_provider
    _normalize_provider_chain = staticmethod(TTSNode._normalize_provider_chain)
    _param_for_set_voice_provider = staticmethod(TTSNode._param_for_set_voice_provider)

    def __init__(self, chain):
        self.provider_chain = list(chain)
        self._provider_dead_until = {}
        self.published = []
        self.voices = {}

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None)

    def _provider_is_dead(self, name):
        return False

    def _publish_provider_state(self, reason, provider=None, voice=None):
        self.published.append((reason, provider, voice))

    def _apply_set_voice(self, param, display, provider, voice, source):
        self.voices[provider] = voice


def _set_provider(node, provider):
    node._on_set_provider(SimpleNamespace(data=json.dumps({"provider": provider, "source": "test"})))


# ── выбор Silero ──────────────────────────────────────────────────────────────


def test_choosing_silero_makes_silero_the_only_provider():
    node = _ChainNode(["yandex", "minimax", "silero"])
    _set_provider(node, "silero")
    assert node.provider_chain == ["silero"]
    assert node._effective_provider() == "silero"


def test_cloud_after_silero_brings_the_cloud_fallbacks_back():
    node = _ChainNode(["silero"])
    _set_provider(node, "minimax")
    assert node.provider_chain == ["minimax", "yandex", "silero"]
    _set_provider(node, "yandex")
    assert node.provider_chain == ["yandex", "minimax", "silero"]


@pytest.mark.parametrize(
    "current, provider, expected",
    [
        (["yandex", "minimax", "silero"], "silero", ["silero"]),
        (["minimax", "yandex", "silero"], "yandex", ["yandex", "minimax", "silero"]),
        (["yandex", "silero"], "minimax", ["minimax", "yandex", "silero"]),
        (["silero"], "yandex", ["yandex", "minimax", "silero"]),
    ],
)
def test_chain_for_requested(current, provider, expected):
    assert chain_for_requested(provider, current) == expected


def test_voice_owner_picking_a_silero_voice_puts_silero_on_air(tmp_path):
    """Сквозной путь ``/voice/tts/voice_request`` → VoiceOwner → ``_on_set_provider``: событие и цепочка согласны."""
    node = _ChainNode(["yandex", "minimax", "silero"])
    events = []
    owner = VoiceOwner(node, str(tmp_path / "tts_voice_state.json"), lambda *a: events.append(a))
    owner.on_request(SimpleNamespace(data=json.dumps({"request_id": "r1", "voice": "aidar", "provider": "silero"})))
    assert events == [("r1", "aidar", "aidar", "silero", "")]
    assert node.provider_chain == ["silero"]
    assert node._effective_provider() == "silero"  # то, что событие назвало, и звучит


# ── мёртвая цепочка SSML-просодии ─────────────────────────────────────────────


@pytest.mark.parametrize(
    "method",
    [
        "_run_synthesis_worker",
        "_synthesize_and_play",
        "_sap_run_provider_chain",
        "_sap_synthesize_minimax",
        "_sap_synthesize_yandex",
        "_sap_silero_fallback",
        "_synthesize_minimax",
        "_minimax_settings",
    ],
)
def test_no_ssml_or_ssml_attributes_in_the_synthesis_pipeline(method):
    params = inspect.signature(getattr(TTSNode, method)).parameters
    assert "ssml" not in params and "ssml_attributes" not in params


def test_dead_prosody_converters_and_normalizer_are_gone():
    for name in ("normalize_for_tts", "normalize_silero_pitch", "_ssml_pitch_to_hz", "_ssml_volume_to_lufs_target"):
        assert not hasattr(tts_node, name), name


def test_one_minimax_tts_implementation():
    """«Не держать две реализации»: обёртка ``rob_box_harness/tts`` удалена, остаётся ``rob_box_llm``."""
    assert not (_SRC / "rob_box_harness" / "rob_box_harness" / "tts").exists()
    assert (_SRC / "rob_box_llm" / "rob_box_llm" / "providers" / "minimax_tts.py").is_file()
