"""Unit tests for issue #2183 — operator-selected voice silently ignored.

Live repro (robot logs, raw evidence quoted in the fix commit):

* ``ros2 param get /tts_node yandex_voice`` → ``alena`` (param genuinely
  updated, supervisor SetVoice applied it).
* ``docker logs voice-assistant`` 8s later → ``voice=default`` synth log,
  then ``Синтез через Yandex Cloud TTS gRPC v3 (anton)`` — the robot kept
  talking as anton.

Three independent causes, each sufficient alone:

1. ``TTSNode.parameters_callback`` had no branch for ``yandex_voice`` /
   ``minimax_voice`` / ``silero_speaker`` — the live ROS param changed,
   but ``self.yandex_voice`` (read once at ``__init__``) never did.
2. Even with cause 1 fixed, the synth path resolved an unrequested voice
   (``voice=None``) straight through ``resolve_voice(provider, None)``,
   which returns the REGISTRY's hardcoded ``DEFAULT_VOICES`` constant —
   never consulting ``self.yandex_voice`` at all. Fixed at the call site
   in ``tts_node`` (not inside ``resolve_voice``, which stays a pure
   registry function with its own tests in ``test_voice_registry.py``):
   the effective "requested" voice is now ``voice or self.<provider>_voice``,
   preserving the priority explicit-request > configured-param > registry
   default.
3. The fallback warning was gated on ``if fell_back and voice:`` — with
   ``voice=None`` (the normal case, no LLM override) this was always
   False, so the alena→anton substitution never produced a single log
   line. Fixed by gating on the *effective* requested voice instead, so
   a genuine misconfiguration (configured voice not in the registry)
   still warns, while the common "nothing requested, configured voice is
   valid" case — verified by the pre-existing
   ``test_minimax_default_when_no_voice`` — stays silent.

This file covers all three fixes plus the priority contract (cause 4):
an explicit ``voice`` in the request (e.g. the LLM's ``zahar`` DJ persona
via ``set_voice``) must still beat the configured node parameter.
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice.tts_node import TTSNode  # noqa: E402
from test.unit.tts.test_voice_selection import (  # noqa: E402
    _playback_node,
    _run_with_voice,
)


# ── Cause 1: parameters_callback must apply the live param ────────────────────


def _param(name, value):
    p = MagicMock()
    p.name = name
    p.value = value
    return p


class TestParametersCallbackAppliesVoiceParams:
    """``ros2 param set /tts_node yandex_voice alena`` must reach
    ``self.yandex_voice`` — before this fix, ``parameters_callback`` had
    no ``elif`` branch for any of the three voice params, so the value
    was accepted at the ROS layer (``successful=True``) but silently
    dropped on the floor.
    """

    @pytest.fixture
    def n(self):
        node = object.__new__(TTSNode)
        node.logger = MagicMock()
        node.get_logger = lambda: node.logger
        node.yandex_voice = "anton"
        node.minimax_voice = "male-qn-qingse"
        node.silero_speaker = "aidar"
        return node

    def test_yandex_voice_applied_live(self, n):
        result = n.parameters_callback([_param("yandex_voice", "alena")])
        assert n.yandex_voice == "alena"
        assert result.successful is True

    def test_minimax_voice_applied_live(self, n):
        n.parameters_callback([_param("minimax_voice", "female-shaonv")])
        assert n.minimax_voice == "female-shaonv"

    def test_silero_speaker_applied_live(self, n):
        n.parameters_callback([_param("silero_speaker", "baya")])
        assert n.silero_speaker == "baya"

    def test_known_voice_param_logs_info_not_warn(self, n):
        n.parameters_callback([_param("yandex_voice", "alena")])
        n.logger.warn.assert_not_called()
        n.logger.info.assert_called()

    def test_unknown_voice_param_still_applied_but_warns(self, n):
        """A typo'd/unknown voice_id is still applied (operator's explicit
        choice is respected — no hidden rejection), but MUST warn since
        the next synthesis without an explicit request will fall back to
        the registry default (issue #2183, cause 3 applies here too).
        """
        n.parameters_callback([_param("yandex_voice", "not_a_real_voice")])
        assert n.yandex_voice == "not_a_real_voice"
        n.logger.warn.assert_called_once()
        assert "not_a_real_voice" in str(n.logger.warn.call_args)

    def test_unrelated_param_does_not_touch_voice_attrs(self, n):
        n.parameters_callback([_param("volume_db", -3.0)])
        assert n.yandex_voice == "anton"
        assert n.minimax_voice == "male-qn-qingse"
        assert n.silero_speaker == "aidar"


# ── Cause 2 + 4: synth path honours configured param, explicit wins ───────────


class TestSynthesisHonoursConfiguredVoice:
    """``_synthesize_and_play(voice=None)`` must use the NODE's configured
    voice (``self.yandex_voice`` etc., live via ``parameters_callback``),
    not the hardcoded ``DEFAULT_VOICES`` registry constant. An explicit
    ``voice`` in the request (LLM override) still wins.
    """

    def test_yandex_uses_configured_voice_when_none_requested(self) -> None:
        node = _playback_node()
        node.provider = "yandex"
        node.provider_chain = ["yandex", "silero"]
        node.yandex_voice = "alena"  # operator set this live
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        node._synthesize_minimax = MagicMock()
        _run_with_voice(node, voice=None)
        _, kwargs = node._synthesize_yandex.call_args
        assert kwargs.get("voice") == "alena"

    def test_yandex_explicit_voice_overrides_configured_param(self) -> None:
        """The LLM's DJ persona (``set_voice`` → zahar) must still beat
        whatever the operator configured on the node — priority order is
        explicit request > configured param > registry default.
        """
        node = _playback_node()
        node.provider = "yandex"
        node.provider_chain = ["yandex", "silero"]
        node.yandex_voice = "alena"  # configured by operator
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        node._synthesize_minimax = MagicMock()
        _run_with_voice(node, voice="zahar")  # explicit LLM override
        _, kwargs = node._synthesize_yandex.call_args
        assert kwargs.get("voice") == "zahar"

    def test_minimax_uses_configured_voice_when_none_requested(self) -> None:
        node = _playback_node()
        node.provider = "minimax"
        node.minimax_voice = "female-shaonv"  # operator set this live
        _run_with_voice(node, voice=None)
        _, kwargs = node._synthesize_minimax.call_args
        assert kwargs.get("voice") == "female-shaonv"

    def test_minimax_explicit_voice_overrides_configured_param(self) -> None:
        node = _playback_node()
        node.provider = "minimax"
        node.minimax_voice = "female-shaonv"
        _run_with_voice(node, voice="Russian_CrazyQueen")
        _, kwargs = node._synthesize_minimax.call_args
        assert kwargs.get("voice") == "Russian_CrazyQueen"

    def test_silero_uses_configured_voice_when_none_requested(self) -> None:
        node = _playback_node()
        node.provider = "minimax"
        node.provider_chain = ["minimax", "yandex", "silero"]
        node.silero_speaker = "baya"  # operator set this live
        node._synthesize_minimax = MagicMock(side_effect=RuntimeError("MiniMax dead"))
        node._synthesize_yandex = MagicMock(side_effect=RuntimeError("Yandex dead"))
        _run_with_voice(node, voice=None)
        _, kwargs = node._synthesize_silero.call_args
        assert kwargs.get("voice") == "baya"

    def test_silero_explicit_voice_overrides_configured_param(self) -> None:
        node = _playback_node()
        node.provider = "minimax"
        node.provider_chain = ["minimax", "yandex", "silero"]
        node.silero_speaker = "baya"
        node._synthesize_minimax = MagicMock(side_effect=RuntimeError("MiniMax dead"))
        node._synthesize_yandex = MagicMock(side_effect=RuntimeError("Yandex dead"))
        _run_with_voice(node, voice="xenia")
        _, kwargs = node._synthesize_silero.call_args
        assert kwargs.get("voice") == "xenia"


# ── Cause 3: fallback warning fires on REAL degradation, always ───────────────


class TestSilentDegradationFixed:
    """The historical guard ``if fell_back and voice:`` meant the alena→anton
    substitution (voice=None, configured voice silently overridden by the
    registry constant) never logged anything. After the cause-2 fix this
    scenario simply isn't a fallback any more (the configured voice IS
    used) — so silence there is correct and expected, matching the
    pre-existing ``test_minimax_default_when_no_voice`` contract. What
    MUST warn, always, is a *genuine* degradation: the configured voice
    itself is not in the provider's registry, even when nothing was
    explicitly requested in the request payload.
    """

    def test_no_warn_when_configured_voice_valid_and_none_requested(self) -> None:
        node = _playback_node()
        node.provider = "yandex"
        node.provider_chain = ["yandex", "silero"]
        node.yandex_voice = "alena"  # valid, operator-configured
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        node._synthesize_minimax = MagicMock()
        _run_with_voice(node, voice=None)
        warn_msgs = " ".join(str(c) for c in node.logger.warn.call_args_list)
        assert "недоступен" not in warn_msgs, (
            "configured voice is valid and honoured — nothing degraded, "
            f"nothing to warn about: {warn_msgs}"
        )

    def test_warns_when_configured_voice_invalid_and_none_requested(self) -> None:
        """Misconfiguration case: operator (or a stale param file) set
        ``yandex_voice`` to something the registry doesn't know. Even
        though the CALLER didn't explicitly request a voice, this IS a
        real silent-degradation risk and must warn (cause 3).
        """
        node = _playback_node()
        node.provider = "yandex"
        node.provider_chain = ["yandex", "silero"]
        node.yandex_voice = "not_a_real_voice"  # misconfigured
        node._synthesize_yandex = MagicMock(return_value=np.zeros(2205, dtype=np.float32))
        node._synthesize_minimax = MagicMock()
        _run_with_voice(node, voice=None)
        _, kwargs = node._synthesize_yandex.call_args
        assert kwargs.get("voice") == "anton"  # registry default, last resort
        warn_msgs = " ".join(str(c) for c in node.logger.warn.call_args_list)
        assert "недоступен у Yandex" in warn_msgs
        assert "not_a_real_voice" in warn_msgs
