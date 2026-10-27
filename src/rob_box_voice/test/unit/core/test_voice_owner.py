"""VoiceOwner: единственный владелец выбранного голоса, событие voice_changed, персист (PR-4 ч.3, #3693)."""

from __future__ import annotations

import json
from types import SimpleNamespace

from rob_box_voice.core.voice_owner import VoiceOwner, read_state, write_state


class _Node:
    """Минимум ``TTSNode``, который трогает владелец голоса."""

    def __init__(self, provider: str = "minimax") -> None:
        self.provider = provider
        self.voices = {}
        self.provider_switches = []
        self.published = []

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **k: None, warning=lambda *a, **k: None)

    def _effective_provider(self):
        return self.provider

    def _on_set_provider(self, msg):
        payload = json.loads(msg.data)
        self.provider_switches.append(payload)
        self.provider = payload["provider"]

    @staticmethod
    def _param_for_set_voice_provider(provider):
        return f"{provider}_voice", provider

    def _apply_set_voice(self, param, display, provider, voice, source):
        self.voices[provider] = voice

    def _publish_provider_state(self, reason, provider=None, voice=None):
        self.published.append((reason, provider, voice))


def _request(owner, **payload):
    owner.on_request(SimpleNamespace(data=json.dumps(payload)))


def _owner(tmp_path, node=None):
    events = []
    node = node or _Node()
    owner = VoiceOwner(node, str(tmp_path / "tts_voice_state.json"), lambda *a: events.append(a))
    return owner, node, events


def test_known_voice_is_applied_and_event_says_so(tmp_path):
    owner, node, events = _owner(tmp_path)
    _request(owner, request_id="r1", voice="female-shaonv")
    assert node.voices == {"minimax": "female-shaonv"}
    assert events == [("r1", "female-shaonv", "female-shaonv", "minimax", "")]
    assert node.published[-1] == ("voice_changed", "minimax", "female-shaonv")


def test_unknown_voice_keeps_provider_default_and_event_reports_fallback(tmp_path):
    owner, node, events = _owner(tmp_path)
    _request(owner, request_id="r2", voice="alena")  # голос Yandex у minimax
    assert node.voices == {"minimax": "male-qn-qingse"}
    assert events == [("r2", "alena", "male-qn-qingse", "minimax", "voice_unavailable")]


def test_cast_voice_asked_by_russian_name_is_applied_as_catalog_id(tmp_path):
    """#4086 «говори голосом Весельчак»: владелец применяет id каталога, событие несёт и просьбу, и применённое."""
    owner, node, events = _owner(tmp_path)
    _request(owner, request_id="r7", voice="Весельчак")
    assert node.voices == {"minimax": "English_Jovialman"}
    assert events == [("r7", "Весельчак", "English_Jovialman", "minimax", "")]
    assert read_state(str(tmp_path / "tts_voice_state.json"))["applied"] == "English_Jovialman"  # переживёт рестарт


def test_cast_voice_asked_by_id_is_applied(tmp_path):
    owner, node, events = _owner(tmp_path)
    _request(owner, request_id="r8", voice="English_Jovialman", provider="minimax")
    assert node.voices == {"minimax": "English_Jovialman"} and events[0][2:] == ("English_Jovialman", "minimax", "")


def test_unknown_name_is_not_guessed_event_reports_it(tmp_path):
    owner, node, events = _owner(tmp_path)
    _request(owner, request_id="r9", voice="Терминатор")
    assert events == [("r9", "Терминатор", "male-qn-qingse", "minimax", "voice_unavailable")]


def test_cross_provider_switches_chain_and_applies_voice(tmp_path):
    owner, node, events = _owner(tmp_path)
    _request(owner, request_id="r3", voice="zahar", provider="yandex")
    assert node.provider_switches and node.provider_switches[0]["provider"] == "yandex"
    assert node.voices == {"yandex": "zahar"}
    assert events[0][2:] == ("zahar", "yandex", "")


def test_empty_voice_means_default_of_provider(tmp_path):
    owner, node, events = _owner(tmp_path)
    _request(owner, request_id="r4", voice="", provider="yandex")
    assert node.voices == {"yandex": "anton"} and events[0][4] == ""


def test_unknown_provider_is_rejected_without_changes(tmp_path):
    owner, node, events = _owner(tmp_path)
    _request(owner, request_id="r5", voice="x", provider="elevenlabs")
    assert node.voices == {} and events == [("r5", "x", "", "elevenlabs", "unknown_provider")]


def test_choice_survives_restart(tmp_path):
    owner, _, _ = _owner(tmp_path)
    _request(owner, request_id="r6", voice="Russian_ReliableMan")
    assert read_state(str(tmp_path / "tts_voice_state.json"))["applied"] == "Russian_ReliableMan"
    fresh_node = _Node()  # рестарт: параметры снова дефолтные
    fresh, _, _ = _owner(tmp_path, fresh_node)
    fresh.restore()
    assert fresh_node.voices == {"minimax": "Russian_ReliableMan"}
    assert fresh_node.published[-1] == ("voice_changed", "minimax", "Russian_ReliableMan")


def test_restore_without_file_or_with_garbage_changes_nothing(tmp_path):
    owner, node, _ = _owner(tmp_path)
    owner.restore()
    (tmp_path / "tts_voice_state.json").write_text("{not json", encoding="utf-8")
    owner.restore()
    write_state(str(tmp_path / "tts_voice_state.json"), {"provider": "elevenlabs", "applied": "x"})
    owner.restore()
    assert node.voices == {}


def test_bad_request_payload_is_rejected(tmp_path):
    owner, _, events = _owner(tmp_path)
    owner.on_request(SimpleNamespace(data="{broken"))
    assert events and events[0][4] in ("", "unknown_provider")
