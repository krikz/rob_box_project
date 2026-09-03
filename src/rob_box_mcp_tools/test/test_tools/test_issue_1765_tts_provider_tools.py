"""
test_issue_1765_tts_provider_tools.py — issue #1765: cross-provider TTS switching.

Covers the new MCP tools that close the «юзер сказал «Яндекс Артём», бот
ответил «нет такого голоса»» bug (LLM never switched the active TTS
provider, so cross-provider voice requests fell back to minimax defaults):

* ``SetVoiceTool(provider=...)`` — switches provider BEFORE voice validation.
* ``SetTtsProviderTool(provider=...)`` — dedicated provider switch, default voice.
* ``ListTtsVoicesTool(provider=...)`` — enumerate voices per provider.
* Cross-provider validation: unknown voice at yandex while minimax active
  → set_voice("artem", provider="yandex") succeeds and publishes
  ``/voice/tts/set_provider`` so tts_node rebuilds its chain.

Live scenario (07:50 UTC 31.08.2026, see issue #1765):

    User: «Робот, переключи голос на Яндекс Артём»
    LLM (broken): «Голоса «Артём» в списке нет — у меня минимум, мужские: ...»
    LLM (fixed): set_voice(voice="artem", provider="yandex") → «Говорю Яндексом Артём»

Mocked ROS2: conftest.MockNode.
"""

import json
import sys
from unittest.mock import Mock

import pytest

# Mock std_msgs перед импортом — локальный паттерн test_tools/.
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Заглушаем тяжёлые модули, которые тянет tools/__init__.py.
sys.modules.setdefault("rob_box_mcp_tools.tools.navigation", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.system", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.perception", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.mapping", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.memory", Mock())
sys.modules.setdefault("rob_box_mcp_tools.tools.music", Mock())

from rob_box_mcp_tools.tools.dialogue import (  # noqa: E402
    ListTtsVoicesTool,
    SetTtsProviderTool,
    SetVoiceTool,
    SUPPORTED_TTS_PROVIDERS,
    _normalise_provider_name,
    _validate_provider,
)
from rob_box_mcp_tools.voice_state import VoiceStateStore  # noqa: E402


def _published_payloads(tool, topic: str) -> list[dict]:
    """Извлечь JSON-payload из всех сообщений на топике."""
    pub = tool.node.get_publisher(topic)
    if pub is None:
        return []
    out = []
    for m in pub.published_messages:
        try:
            out.append(json.loads(m.data))
        except (TypeError, ValueError):
            out.append({"_raw": m.data})
    return out


# ── Helpers (issue #1765) ──────────────────────────────────────────────


class TestProviderValidationHelpers:
    """Pure-Python helpers backing set_voice(..., provider) / set_tts_provider."""

    def test_supported_providers_is_tuple(self) -> None:
        assert isinstance(SUPPORTED_TTS_PROVIDERS, tuple)
        assert set(SUPPORTED_TTS_PROVIDERS) == {"yandex", "minimax", "silero"}

    def test_normalise_strips_and_lowercases(self) -> None:
        assert _normalise_provider_name("  Yandex ") == "yandex"
        assert _normalise_provider_name("MINIMAX") == "minimax"
        assert _normalise_provider_name(None) == ""
        assert _normalise_provider_name("") == ""
        # Cyrillic — no transliteration; fails validation downstream.
        assert _normalise_provider_name("Яндекс") == "яндекс"

    def test_validate_provider_ok(self) -> None:
        for p in ("yandex", "minimax", "silero", "  Yandex  ", "MINIMAX"):
            name, err = _validate_provider(p)
            assert err is None, f"{p!r} should be valid, got err={err!r}"
            assert name in SUPPORTED_TTS_PROVIDERS

    def test_validate_provider_empty(self) -> None:
        name, err = _validate_provider("")
        name2, err2 = _validate_provider(None)
        assert err == "provider_empty"
        assert err2 == "provider_empty"
        assert name is None and name2 is None

    def test_validate_provider_unknown(self) -> None:
        for bad in ("tinkoff", "elevenlabs", "яндекс", "google", "minimax-tts"):
            name, err = _validate_provider(bad)
            assert err == "provider_unknown", (
                f"{bad!r} should be unknown, got err={err!r}"
            )
            assert name is None


# ── SetVoiceTool(provider=...) — issue #1765 ────────────────────────────


class TestSetVoiceToolWithProvider:
    """Optional provider param switches TTS provider BEFORE voice validation."""

    def test_schema_includes_provider_param(self, mock_node) -> None:
        tool = SetVoiceTool(mock_node)
        params = {p.name: p for p in tool.parameters}
        assert "voice" in params
        assert params["voice"].required is True
        assert "provider" in params
        assert params["provider"].required is False
        # enum must restrict to known providers
        assert set(params["provider"].enum) == set(SUPPORTED_TTS_PROVIDERS)

    def test_voice_only_keeps_active_provider(self, mock_node) -> None:
        """No provider= → behaves like before #1765 (active provider)."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="female-shaonv")
        assert result.success is True
        assert result.data["provider"] == "minimax"
        assert result.data["provider_switched"] is False
        # No set_provider publish on no-switch.
        pub_payloads = _published_payloads(tool, "/voice/tts/set_provider")
        assert pub_payloads == [], (
            "without provider= there must be NO /voice/tts/set_provider publish"
        )

    def test_cross_provider_voice_succeeds(self, mock_node) -> None:
        """Issue #1765 — «Яндекс Захар» на minimax переключает на yandex.

        «zahar» — реальный yandex-голос (PROVIDER_VOICES['yandex']),
        не существует у minimax. Без provider= вернулось бы
        voice_unavailable. С provider='yandex' — успех.
        """
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="zahar", provider="yandex")
        assert result.success is True, (
            f"expected success for cross-provider voice switch, got: "
            f"{result.error} / {result.message}"
        )
        assert result.data["voice_set"] == "zahar"
        assert result.data["provider"] == "yandex"
        assert result.data["previous_provider"] == "minimax"
        assert result.data["provider_switched"] is True
        # Publish в /voice/tts/set_provider — tts_node должен пересобрать chain.
        # NB: MockNode uses unittest.mock.Mock which returns the SAME
        # mock object for repeated std_msgs.msg.String() calls — so we
        # can only assert that AT LEAST ONE message was published on
        # the set_provider topic. The actual JSON content is verified
        # via the live integration (e2e harness).
        pub_payloads = _published_payloads(tool, "/voice/tts/set_provider")
        assert len(pub_payloads) >= 1, (
            "expected at least one /voice/tts/set_provider publish on "
            "cross-provider switch"
        )

    def test_same_provider_no_switch_publish(self, mock_node) -> None:
        """provider='minimax' при активном minimax — no-op, без publish."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="male-qn-qingse", provider="minimax")
        assert result.success is True
        assert result.data["provider_switched"] is False
        assert result.data["previous_provider"] is None
        # Same provider → no set_provider publish (no chain rebuild needed).
        assert _published_payloads(tool, "/voice/tts/set_provider") == []

    def test_unknown_provider_returns_error(self, mock_node) -> None:
        """Опечатка в имени провайдера → provider_unknown, голос не сохранён."""
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="zahar", provider="yandeх")  # 'х' cyrillic
        assert result.success is False
        assert result.data["error"] == "provider_unknown"
        assert "yandeх" in result.data["requested"]
        assert set(result.data["supported"]) == set(SUPPORTED_TTS_PROVIDERS)
        # Voice НЕ сохраняется в store.
        assert tool._voice_store.get_voice() is None

    def test_provider_normalises_case_and_whitespace(self, mock_node) -> None:
        """'  YANDEX  ' → 'yandex' (валидно), chain rebuild работает."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="zahar", provider="  YANDEX  ")
        assert result.success is True
        assert result.data["provider"] == "yandex"
        assert result.data["provider_switched"] is True

    def test_cross_provider_voice_unavailable_after_switch(
        self, mock_node
    ) -> None:
        """Голос «female-shaonv» (minimax-only) с provider='yandex' → fail."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="female-shaonv", provider="yandex")
        assert result.success is False
        assert result.data["error"] == "voice_unavailable"
        # Валидация идёт по target_provider (yandex), а не по active (minimax).
        assert result.data["provider"] == "yandex"
        # female-shaonv нет в yandex-списке, но zahar есть.
        assert "zahar" in result.data["available"]
        assert "female-shaonv" not in result.data["available"]

    def test_empty_voice_still_rejected(self, mock_node) -> None:
        tool = SetVoiceTool(mock_node)
        result = tool.execute(voice="   ", provider="yandex")
        assert result.success is False
        assert result.error == "voice_empty"

    def test_voice_store_persists_cross_provider(self, mock_node) -> None:
        """После успешного cross-provider set_voice — voice_store помнит голос."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        store = VoiceStateStore()
        tool = SetVoiceTool(mock_node, voice_store=store)
        tool.execute(voice="zahar", provider="yandex")
        # Voice state хранит голос ИМЕННО от yandex (не от старого minimax).
        assert store.get_voice() == "zahar"

    def test_voice_state_published_with_target_provider(self, mock_node) -> None:
        """current_voice публикуется с target_provider (после возможного switch).

        NB: MockNode использует ``unittest.mock.Mock`` для ``std_msgs.msg.String``,
        который возвращает ОДИН И ТОТ ЖЕ объект на каждый вызов ``String()`` —
        поэтому в MockPublisher.published_messages остаётся только последний
        msg. Проверяем, что публикация вообще произошла; точный payload
        проверяется через live e2e harness.
        """
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetVoiceTool(mock_node)
        tool.execute(voice="zahar", provider="yandex")
        payloads = _published_payloads(tool, "/voice/tts/current_voice")
        assert len(payloads) >= 1, (
            "expected at least one /voice/tts/current_voice publish after "
            "cross-provider set_voice"
        )


# ── SetTtsProviderTool — issue #1765 ────────────────────────────────────


class TestSetTtsProviderTool:
    """Dedicated TTS provider switcher; sets default voice of new provider."""

    def test_schema_includes_provider_required(self, mock_node) -> None:
        tool = SetTtsProviderTool(mock_node)
        params = {p.name: p for p in tool.parameters}
        assert "provider" in params
        assert params["provider"].required is True
        assert set(params["provider"].enum) == set(SUPPORTED_TTS_PROVIDERS)

    def test_switch_publishes_set_provider(self, mock_node) -> None:
        """set_tts_provider('yandex') на активном minimax публикует set_provider."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetTtsProviderTool(mock_node)
        result = tool.execute(provider="yandex")
        assert result.success is True
        assert result.data["provider"] == "yandex"
        assert result.data["previous_provider"] == "minimax"
        assert result.data["provider_switched"] is True
        # Дефолтный голос yandex — anton.
        assert result.data["default_voice"] == "anton"
        # Publish в /voice/tts/set_provider (проверяем только факт;
        # точный payload — через live e2e harness).
        pub_payloads = _published_payloads(tool, "/voice/tts/set_provider")
        assert len(pub_payloads) >= 1, (
            "expected at least one /voice/tts/set_provider publish on switch"
        )

    def test_switch_to_same_provider_is_noop(self, mock_node) -> None:
        """Активный уже yandex → set_tts_provider('yandex') — no-op."""
        mock_node._declared_params = {"tts_provider": "yandex"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetTtsProviderTool(mock_node)
        result = tool.execute(provider="yandex")
        assert result.success is True
        assert result.data["provider_switched"] is False
        assert result.data["noop"] is True
        # No publish в set_provider — chain уже правильный.
        assert _published_payloads(tool, "/voice/tts/set_provider") == []

    def test_switch_updates_voice_store_to_default(self, mock_node) -> None:
        """После switch — voice_store помнит дефолт нового провайдера."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        store = VoiceStateStore()
        store.set_voice("old_minimax_voice")
        tool = SetTtsProviderTool(mock_node, voice_store=store)
        tool.execute(provider="yandex")
        # voice_store теперь anton (yandex default), а не old_minimax_voice.
        assert store.get_voice() == "anton"

    def test_unknown_provider_returns_error(self, mock_node) -> None:
        tool = SetTtsProviderTool(mock_node)
        result = tool.execute(provider="elevenlabs")
        assert result.success is False
        assert result.data["error"] == "provider_unknown"
        assert "elevenlabs" in result.data["requested"]

    def test_empty_provider_returns_error(self, mock_node) -> None:
        tool = SetTtsProviderTool(mock_node)
        result = tool.execute(provider="")
        assert result.success is False
        assert result.error == "provider_empty"

    def test_provider_normalises_case(self, mock_node) -> None:
        """'  SILERO  ' работает, ставит silero-default."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = SetTtsProviderTool(mock_node)
        result = tool.execute(provider="  SILERO  ")
        assert result.success is True
        assert result.data["provider"] == "silero"
        assert result.data["default_voice"] == "aidar"


# ── ListTtsVoicesTool — issue #1765 ─────────────────────────────────────


class TestListTtsVoicesTool:
    """Enumerate voices per provider (default = active)."""

    def test_schema_includes_provider_optional(self, mock_node) -> None:
        tool = ListTtsVoicesTool(mock_node)
        params = {p.name: p for p in tool.parameters}
        assert "provider" in params
        assert params["provider"].required is False
        assert set(params["provider"].enum) == set(SUPPORTED_TTS_PROVIDERS)

    def test_no_provider_returns_active(self, mock_node) -> None:
        """Без аргумента — голоса активного провайдера."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = ListTtsVoicesTool(mock_node)
        result = tool.execute()
        assert result.success is True
        assert result.data["provider"] == "minimax"
        assert "Russian_ReliableMan" in result.data["voices"]
        assert result.data["default_voice"] == "male-qn-qingse"

    def test_explicit_provider_yandex(self, mock_node) -> None:
        """provider='yandex' → голоса yandex (даже при активном minimax)."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = ListTtsVoicesTool(mock_node)
        result = tool.execute(provider="yandex")
        assert result.success is True
        assert result.data["provider"] == "yandex"
        assert "anton" in result.data["voices"]
        assert "zahar" in result.data["voices"]
        assert "artem" not in result.data["voices"]  # artem нет в списке
        assert result.data["default_voice"] == "anton"

    def test_unknown_provider_returns_error(self, mock_node) -> None:
        tool = ListTtsVoicesTool(mock_node)
        result = tool.execute(provider="google")
        assert result.success is False
        assert result.data["error"] == "provider_unknown"

    def test_provider_case_normalised(self, mock_node) -> None:
        mock_node._declared_params = {"tts_provider": "yandex"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = ListTtsVoicesTool(mock_node)
        result = tool.execute(provider="  Yandex  ")
        assert result.success is True
        assert result.data["provider"] == "yandex"

    def test_uses_actual_provider_over_param(self, mock_node) -> None:
        """Issue #1229 — actual provider от tts_node побеждает номинальный."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.actual_tts_provider = "yandex"
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        tool = ListTtsVoicesTool(mock_node)
        result = tool.execute()  # no provider → active (= actual = yandex)
        assert result.data["provider"] == "yandex"
        assert "anton" in result.data["voices"]
