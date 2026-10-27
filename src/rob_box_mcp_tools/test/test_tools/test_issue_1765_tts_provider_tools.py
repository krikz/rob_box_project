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
from rob_box_mcp_tools.voice_request import VoiceOutcome  # noqa: E402
from rob_box_voice.tts_voice_registry import PROVIDER_VOICES  # noqa: E402


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


# ── set_voice / set_tts_provider: просьба владельцу голоса + фраза по событию (PR-4 ч.3, #3693) ──────────


class _FakeRequests:
    """Подмена ``voice_requests(node)``: запоминает просьбы, отвечает заранее заданным исходом."""

    def __init__(self, outcome=None) -> None:
        self.calls: list[tuple[str, str]] = []
        self.outcome = outcome

    def request(self, voice: str, provider: str = "", wait_s: float = 0.0) -> VoiceOutcome:
        self.calls.append((voice, provider))
        return self.outcome or VoiceOutcome("applied", voice, voice or "anton", provider or "minimax")


@pytest.fixture
def fake_requests(monkeypatch):
    fake = _FakeRequests()
    monkeypatch.setattr("rob_box_mcp_tools.tools.dialogue.voice_requests", lambda node: fake)
    return fake


def _active(mock_node, provider: str) -> None:
    mock_node._declared_params = {"tts_provider": provider}
    mock_node.get_parameter = lambda n: type("P", (), {"value": mock_node._declared_params.get(n, provider)})()


class TestSetVoiceToolWithProvider:
    """Тул ничего не хранит и не публикует голос: просит владельца и говорит исход."""

    def test_schema_includes_provider_param(self, mock_node) -> None:
        tool = SetVoiceTool(mock_node)
        params = {p.name: p for p in tool.parameters}
        assert params["voice"].required is True
        assert params["provider"].required is False
        assert set(params["provider"].enum) == set(SUPPORTED_TTS_PROVIDERS)

    def test_voice_only_asks_owner_without_provider(self, mock_node, fake_requests) -> None:
        _active(mock_node, "minimax")
        result = SetVoiceTool(mock_node).execute(voice="female-shaonv")
        assert fake_requests.calls == [("female-shaonv", "")]
        assert result.success is True
        assert result.data["voice_applied"] == "female-shaonv"
        assert result.data["provider_switched"] is False
        assert "Говорю голосом female-shaonv" in result.message

    def test_cross_provider_asks_owner_with_provider(self, mock_node, fake_requests) -> None:
        _active(mock_node, "minimax")
        fake_requests.outcome = VoiceOutcome("applied", "zahar", "zahar", "yandex")
        result = SetVoiceTool(mock_node).execute(voice="zahar", provider="yandex")
        assert fake_requests.calls == [("zahar", "yandex")]
        assert result.success is True
        assert result.data["provider"] == "yandex"
        assert result.data["previous_provider"] == "minimax"
        assert result.data["provider_switched"] is True

    def test_unavailable_voice_is_reported_by_code_not_success(self, mock_node, fake_requests) -> None:
        _active(mock_node, "yandex")
        fake_requests.outcome = VoiceOutcome("fallback", "female-shaonv", "anton", "yandex", "voice_unavailable")
        result = SetVoiceTool(mock_node).execute(voice="female-shaonv", provider="yandex")
        assert result.success is False
        assert result.data["voice_applied"] == "anton"
        assert "Голос female-shaonv недоступен, остаюсь на anton" in result.message
        assert "zahar" in result.data["available"]

    def test_no_confirmation_from_owner_is_honest(self, mock_node, fake_requests) -> None:
        _active(mock_node, "minimax")
        fake_requests.outcome = VoiceOutcome("timeout", "zahar", "", "yandex")
        result = SetVoiceTool(mock_node).execute(voice="zahar", provider="yandex")
        assert result.success is False
        assert "Не дождался подтверждения" in result.message
        assert "установлен" not in result.message

    def test_unknown_provider_returns_error_without_request(self, mock_node, fake_requests) -> None:
        result = SetVoiceTool(mock_node).execute(voice="zahar", provider="yandeх")  # 'х' cyrillic
        assert result.success is False
        assert result.data["error"] == "provider_unknown"
        assert fake_requests.calls == []

    def test_provider_normalises_case_and_whitespace(self, mock_node, fake_requests) -> None:
        _active(mock_node, "minimax")
        SetVoiceTool(mock_node).execute(voice="zahar", provider="  YANDEX  ")
        assert fake_requests.calls == [("zahar", "yandex")]

    def test_empty_voice_still_rejected(self, mock_node, fake_requests) -> None:
        result = SetVoiceTool(mock_node).execute(voice="   ", provider="yandex")
        assert result.success is False and result.error == "voice_empty"
        assert fake_requests.calls == []


class TestSetTtsProviderTool:
    """Переключение провайдера целиком: пустой голос = дефолт нового провайдера (его выбирает владелец)."""

    def test_schema_includes_provider_required(self, mock_node) -> None:
        params = {p.name: p for p in SetTtsProviderTool(mock_node).parameters}
        assert params["provider"].required is True
        assert set(params["provider"].enum) == set(SUPPORTED_TTS_PROVIDERS)

    def test_switch_asks_owner_for_default_voice(self, mock_node, fake_requests) -> None:
        _active(mock_node, "minimax")
        result = SetTtsProviderTool(mock_node).execute(provider="yandex")
        assert fake_requests.calls == [("", "yandex")]
        assert result.success is True
        assert result.data["previous_provider"] == "minimax"
        assert result.data["provider_switched"] is True

    def test_switch_to_same_provider_is_noop_without_request(self, mock_node, fake_requests) -> None:
        _active(mock_node, "yandex")
        result = SetTtsProviderTool(mock_node).execute(provider="yandex")
        assert result.success is True and result.data["noop"] is True
        assert fake_requests.calls == []

    def test_unknown_and_empty_provider(self, mock_node, fake_requests) -> None:
        bad = SetTtsProviderTool(mock_node).execute(provider="elevenlabs")
        assert bad.success is False and bad.data["error"] == "provider_unknown"
        empty = SetTtsProviderTool(mock_node).execute(provider="")
        assert empty.success is False and empty.error == "provider_empty"
        assert fake_requests.calls == []

    def test_provider_normalises_case(self, mock_node, fake_requests) -> None:
        _active(mock_node, "minimax")
        SetTtsProviderTool(mock_node).execute(provider="  SILERO  ")
        assert fake_requests.calls == [("", "silero")]


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
        assert result.data["default_voice"] == "male-qn-qingse"
        assert "voices" not in result.data  # полный список ids — только по provider (#3883)

    def test_no_provider_gives_brief_overview_by_providers(self, mock_node) -> None:
        """#3883: без аргумента — провайдеры → сколько голосов → по паре характерных (м/ж), активный помечен."""
        mock_node._declared_params = {"tts_provider": "minimax"}
        mock_node.get_parameter = lambda n: type(
            "P", (), {"value": mock_node._declared_params.get(n, "minimax")}
        )()
        data = ListTtsVoicesTool(mock_node).execute().data
        by_name = {p["provider"]: p for p in data["providers"]}
        assert set(by_name) == set(SUPPORTED_TTS_PROVIDERS)
        assert [p["provider"] for p in data["providers"] if p["active"]] == ["minimax"]
        # число голосов - из реестра, не вписано руками: новый голос каталога (#4086) не ломает тест
        assert {n: p["voices_count"] for n, p in by_name.items()} == {n: len(v) for n, v in PROVIDER_VOICES.items()}
        for p in data["providers"]:
            assert {s["gender"] for s in p["samples"]} == {"male", "female"}
            assert p["label"] and all(s["name"] and "legacy" not in s["name"] for s in p["samples"])
        assert len(str(data)) < 1500  # краткая структура, а не список ids

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

    def test_provider_listing_names_voices_for_the_human(self, mock_node) -> None:
        """#4086: у голосов каста MiniMax в ответе есть русские имена - ими модель назовёт голос человеку."""
        result = ListTtsVoicesTool(mock_node).execute(provider="minimax")
        assert "English_Jovialman" in result.data["voices"]
        assert result.data["names"]["English_Jovialman"] == "Весельчак"
        assert set(result.data["names"]) == set(result.data["voices"])  # имя есть у каждого голоса списка

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
        assert [p["provider"] for p in result.data["providers"] if p["active"]] == ["yandex"]
