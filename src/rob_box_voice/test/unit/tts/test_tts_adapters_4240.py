"""#4240 (ADR-0160 PR-3b): адаптеры Yandex и Silero на ``BaseTTSProvider``.

* Синтез Yandex и Silero живёт в ``rob_box_voice.tts_providers`` (одна реализация); методов синтеза в ``TTSNode``
  больше нет — нода зовёт ``prepare`` и ``synthesize_np`` адаптера.
* Флаги сервиса (``reads_digits``, ``sets_stress``, ``pause_markup``) — честные: то, что адаптер делает сейчас.
* ``prepare`` каждого адаптера сам снимает метки пауз MiniMax ``<#x#>``; у MiniMax — текст как есть.
* Лог Silero говорит «основной», когда Silero выбран сам, и «запасной» на откате (раньше всегда «fallback»).
"""
from __future__ import annotations

import asyncio
import inspect
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_dialog.plan.rap_pauses import markup  # noqa: E402
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider  # noqa: E402
from rob_box_llm.tts import TTSFormat, TTSSettings  # noqa: E402
from rob_box_llm.tts_provider_base import BaseTTSProvider, TTSCapabilities  # noqa: E402
from rob_box_voice import tts_node  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402
from rob_box_voice.tts_providers import SileroTTSProvider, YandexTTSProvider  # noqa: E402

MARKED = markup("Я робот Робби, я читаю под бит,\nКолёса крутятся, мотор не молчит.", 240.0 / 131).text


def _yandex() -> YandexTTSProvider:
    return YandexTTSProvider("fake-key", object(), logger=MagicMock())


def _silero() -> SileroTTSProvider:
    return SileroTTSProvider(logger=MagicMock())


# ── одна реализация ───────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "gone",
    [
        "_synthesize_yandex",
        "_synthesize_yandex_single",
        "_synthesize_yandex_single_with_latency",
        "_chunk_text",
        "_synthesize_silero",
        "_load_silero_model",
    ],
)
def test_synthesis_methods_left_the_node(gone):
    assert not hasattr(TTSNode, gone), f"TTSNode.{gone} — вторая реализация рядом с адаптером"


def test_adapters_are_base_tts_providers():
    for cls in (YandexTTSProvider, SileroTTSProvider):
        assert issubclass(cls, BaseTTSProvider)
        assert not inspect.isabstract(cls), cls


def test_yandex_ssml_aware_param_is_gone():
    assert "yandex_ssml_aware" not in inspect.getsource(tts_node)


# ── флаги сервиса ─────────────────────────────────────────────────────────────


def test_capability_flags_default_to_false():
    caps = TTSCapabilities()
    assert (caps.reads_digits, caps.sets_stress, caps.pause_markup) == (False, False, False)


@pytest.mark.parametrize(
    "provider, reads_digits, sets_stress, pause_markup",
    [
        (lambda: MiniMaxTTSProvider(api_key="k", group_id="g", client=MagicMock()), True, True, True),
        (_yandex, True, False, False),  # sil<[мс]> — PR-4, «+» — PR-7
        (_silero, False, True, False),  # нормализатор — PR-5, <break> — PR-4
    ],
)
def test_capability_flags_are_honest(provider, reads_digits, sets_stress, pause_markup):
    caps = provider().capabilities()
    assert (caps.reads_digits, caps.sets_stress, caps.pause_markup) == (reads_digits, sets_stress, pause_markup)


def test_only_silero_claims_ssml():
    assert _silero().capabilities().ssml is True
    assert _yandex().capabilities().ssml is False


# ── prepare ───────────────────────────────────────────────────────────────────


def test_base_prepare_keeps_text():
    assert MiniMaxTTSProvider(api_key="k", group_id="g", client=MagicMock()).prepare(MARKED) == MARKED


@pytest.mark.parametrize("adapter", [_yandex, _silero])
def test_prepare_strips_minimax_pause_marks(adapter):
    assert "<#" in MARKED
    prepared = adapter().prepare(MARKED)
    assert "<#" not in prepared
    assert prepared.startswith("Я робот Робби, я читаю под бит,")


@pytest.mark.parametrize("adapter", [_yandex, _silero])
def test_prepare_keeps_plain_text(adapter):
    text = "Батарея 20% & сигнал в норме.\nВторая строка."
    assert adapter().prepare(text) == text


# ── Silero: конверт на чанк и честный отказ без модели ─────────────────────


def test_silero_without_model_is_an_honest_error():
    with pytest.raises(Exception, match="не инициализирована"):
        _silero().synthesize_np("текст")


def test_silero_request_payload_is_escaped_envelope():
    payload = _silero()._build_request_payload("a < b & c", TTSSettings(voice="aidar"), None)
    assert payload["ssml_text"] == '<speak><prosody pitch="medium">a &lt; b &amp; c</prosody></speak>'
    assert payload["speaker"] == "aidar"


def test_silero_load_without_model_file_returns_false(tmp_path, monkeypatch):
    torch = MagicMock()
    monkeypatch.setitem(sys.modules, "torch", torch)
    adapter = SileroTTSProvider(logger=MagicMock(), model_paths=(str(tmp_path / "нет.pt"),))
    assert adapter.load() is False
    assert adapter.model is None
    assert "apply_resource_pack.sh" in str(adapter._logger.error.call_args)


# ── async-контракт TTSProvider ────────────────────────────────────────────────


def test_async_synthesize_prepares_and_returns_pcm():
    adapter = _silero()
    adapter.model = MagicMock()
    seen = {}

    def fake(text, *, speaker=None):
        seen["text"], seen["speaker"] = text, speaker
        return np.array([0.0, 0.5, -0.5], dtype=np.float32)

    adapter.synthesize_np = fake
    audio = asyncio.run(adapter.synthesize(MARKED, settings=TTSSettings(voice="aidar")))
    assert "<#" not in seen["text"] and seen["speaker"] == "aidar"
    assert audio.format == TTSFormat.PCM and audio.sample_rate == 48000
    assert len(audio.samples) == 3 * 2


# ── лог Silero: основной / запасной ──────────────────────────────────────────


def _silero_node(chain):
    node = SimpleNamespace(provider_chain=chain, provider=chain[0])
    node._silero_tts = _silero()
    node._silero_tts.model = MagicMock()
    node._silero_tts.synthesize_np = MagicMock(return_value=np.zeros(480, dtype=np.float32))
    node._silero_loaded = SimpleNamespace(wait=lambda timeout: True, set=lambda: None)
    node.silero_warm_load_enabled = True
    node.silero_speaker = "aidar"
    node.publish_state = MagicMock()
    node.logger = MagicMock()
    node.get_logger = lambda: node.logger
    return node


@pytest.mark.parametrize("chain, role", [(["silero"], "основной"), (["minimax", "yandex", "silero"], "запасной")])
def test_silero_log_names_its_role(chain, role):
    node = _silero_node(chain)
    ctx = {"audio_np": None, "sample_rate": None, "used_provider": None, "used_voice": None}
    TTSNode._sap_silero_fallback(node, "Привет.", None, None, None, ctx)
    infos = " | ".join(str(c.args[0]) for c in node.logger.info.call_args_list)
    assert f"Синтез через Silero v5 ({role})" in infos
    assert f"Silero v5 ({role}) успешен" in infos
    assert "fallback" not in infos
    assert ctx["used_provider"] == "silero" and ctx["sample_rate"] == 48000
