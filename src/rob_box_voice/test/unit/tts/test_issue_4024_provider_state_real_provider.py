"""#4024: ``/voice/tts/provider_state`` несёт провайдера синтеза, а не метку ``prefetch``.

Марафон 23.10, акт 4: tts_node публикует ``provider_state`` с ``provider='prefetch'`` для озвучки готового пред-синтеза,
читатели принимали метку за провайдера, каталог голосов у них пустел, и голоса сегментов сказки не сверялись (LLM взяла
Yandex-имена, MiniMax молча заменил их). Тесты гоняют настоящий путь: пред-синтез (``Presynth``) -> ``commit=true``
(``_sap_consume_presynth``) -> публикация (``_sap_publish_for_sink``) -> JSON в топике.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice.core.speech_timing import PREFETCH_PROVIDER, Presynth  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402
from rob_box_voice.tts_voice_registry import known_provider  # noqa: E402


class _Bare:
    """Заглушка ноды: только атрибуты, которых касается публикация provider_state."""


class _Topic:
    def __init__(self) -> None:
        self.sent: list = []

    def publish(self, msg) -> None:
        self.sent.append(json.loads(msg.data))


def _node(effective: str = "minimax") -> _Bare:
    node = _Bare()
    node.logger = MagicMock()
    node.get_logger = lambda: node.logger
    node.provider_state_pub = _Topic()
    node._effective_provider = lambda: effective
    node._persist_provider_state = MagicMock()
    node._publish_voices_catalog = MagicMock()
    node._publish_audio = MagicMock()
    node._prepare_audio_for_topic = lambda audio, rate: audio
    node._publish_provider_state = TTSNode._publish_provider_state.__get__(node, _Bare)
    node.minimax_voice = "male-qn-qingse"
    return node


def _new_ctx() -> dict:
    return {
        "audio_np": None,
        "sample_rate": 16000,
        "used_provider": None,
        "used_voice": None,
        "result": {},
    }


def _presynth(
    provider: str,
    voice: str | None,
    speech_id: str = "seg-1",
    requested: str | None = None,
) -> dict:
    """Готовый пред-синтез настоящим ``Presynth``: цепочка провайдеров подставлена, пул - синхронный."""
    node = SimpleNamespace(
        _extract_text_from_ssml=lambda ssml: "Куда идёшь?",
        _parse_ssml_attributes=lambda ssml: {},
        _publish_tts_finished=MagicMock(),
    )

    def chain(voice_, language, attrs, text, sink, ctx):
        ctx.update(
            audio_np=np.zeros(1600, dtype=np.float32),
            sample_rate=16000,
            used_provider=provider,
            used_voice=voice,
        )

    node._sap_run_provider_chain = chain
    node._sap_silero_fallback = MagicMock()
    held = Presynth(executor=SimpleNamespace(submit=lambda fn, *a: fn(*a)))
    held.run(
        node,
        {
            "speech_id": speech_id,
            "ssml": "<speak>Куда идёшь?</speak>",
            "voice": requested,
        },
        lambda *args: None,
        lambda sid: False,
    )
    taken = held.claim(speech_id)
    assert taken is not None
    return taken


def _commit(node: _Bare, prebaked: dict, voice: str | None = None) -> dict:
    """Что делает ``commit=true`` с пред-синтезом: берёт аудио, публикует его, провайдер - в provider_state."""
    ctx = _new_ctx()
    assert TTSNode._sap_consume_presynth(node, prebaked, voice, "seg-1", ctx) is True
    TTSNode._sap_publish_for_sink(
        node,
        ctx["audio_np"],
        ctx["sample_rate"],
        "speaker",
        ctx["used_provider"],
        ctx["result"],
        voice,
        ctx["used_voice"],
    )
    return ctx


def test_presynth_remembers_who_synthesized_it() -> None:
    held = _presynth(
        "minimax", "Russian_AttractiveGuy", requested="Russian_AttractiveGuy"
    )
    assert (held["provider"], held["voice"]) == ("minimax", "Russian_AttractiveGuy")


def test_provider_state_after_prefetch_carries_the_real_provider_not_the_label() -> (
    None
):
    node = _node()
    _commit(
        node,
        _presynth("minimax", "Russian_AttractiveGuy"),
        voice="Russian_AttractiveGuy",
    )

    (state,) = node.provider_state_pub.sent
    assert state["provider"] == "minimax"
    assert state["voice"] == "Russian_AttractiveGuy"
    assert state["reason"] == "synthesis_ok"
    assert PREFETCH_PROVIDER not in json.dumps(state)


def test_prefetch_after_fallback_to_yandex_reports_yandex_and_its_voice() -> None:
    """MiniMax мёртв, пред-синтез сделал Yandex: provider_state - Yandex с его голосом, а не default MiniMax."""
    node = _node(effective="yandex")
    _commit(node, _presynth("yandex", "anton"), voice="Russian_AttractiveGuy")

    (state,) = node.provider_state_pub.sent
    assert (state["provider"], state["voice"]) == ("yandex", "anton")


def test_prefetch_keeps_its_marker_for_the_speech_journal_not_for_the_provider() -> (
    None
):
    ctx = _commit(_node(), _presynth("minimax", "Russian_AttractiveGuy"))
    assert ctx["prefetch"] is True
    assert ctx["used_provider"] == "minimax"


def test_prebaked_audio_without_a_provider_publishes_no_provider_state() -> None:
    """Кто синтезировал - не сказано (чужой вызов): не врём ни «prefetch», ни угаданным провайдером."""
    node = _node()
    prebaked = {
        "audio_np": np.zeros(1600, dtype=np.float32),
        "sample_rate": 16000,
        "decision": "presynth",
        "confidence": 1.0,
        "basis": "commit=false",
    }
    ctx = _commit(node, prebaked)

    assert not ctx["used_provider"]
    assert node.provider_state_pub.sent == []


def test_publish_provider_state_never_accepts_a_label_as_the_provider() -> None:
    """Структурная защита воронки: что бы ни передали, наружу уходит провайдер из каталога реестра."""
    node = _node(effective="yandex")
    node._publish_provider_state(
        "synthesis_ok", provider=PREFETCH_PROVIDER, voice="anton"
    )

    (state,) = node.provider_state_pub.sent
    assert state["provider"] == "yandex"
    assert known_provider(state["provider"]) == state["provider"]
    node.logger.warn.assert_called_once()
    assert "prefetch" in node.logger.warn.call_args.args[0]


def test_publish_provider_state_still_takes_a_real_provider_as_is() -> None:
    node = _node(effective="minimax")
    node._publish_provider_state("provider_dead", provider="silero")

    (state,) = node.provider_state_pub.sent
    assert state["provider"] == "silero"
    node.logger.warn.assert_not_called()
