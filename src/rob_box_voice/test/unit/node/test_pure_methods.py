"""
test_pure_methods.py — Unit-тесты «лёгких» методов DialogueNode.

Покрывает:
  - _map_emotion_to_animation    (чистый dict lookup)
  - _generate_fallback_response  (чистый string matching)
  - _detect_volume_intent        (regex, без ROS)
  - _detect_pitch_intent         (regex, без ROS)
  - _speak_simple                (publish + json)
  - _trigger_sound               (publish)
  - _on_mcp_tools_update         (JSON parse → attributes)
  - _on_perception_update        (attribute update)
  - vad_callback                 (flag logic)

Не требует ROS2 — rclpy замокан в conftest.py.
"""

import json
from unittest.mock import MagicMock, call

import pytest

from rob_box_voice.dialogue_node import DialogueNode


# ─────────────────────────────────────────────────────────────────────────────
#  Fixture
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def node():
    """Минимальная DialogueNode без __init__."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger

    # Publishers
    n.response_pub = MagicMock()
    n.animation_pub = MagicMock()
    n.sound_trigger_pub = MagicMock()
    n.state_pub = MagicMock()

    # State attrs
    n.current_dialogue_id = None
    n.available_tools = []
    n.mcp_tools_available = False
    n.internet_available = True
    n.current_time_info = {}

    # VAD
    n.vad_speech_detected = False
    n.llm_processing = False
    n.interrupt_agent_loop = False

    # dialogue_manager (for _publish_state)
    n.dialogue_manager = MagicMock()
    n.dialogue_manager.state.value = "idle"

    return n


# ─────────────────────────────────────────────────────────────────────────────
#  _map_emotion_to_animation
# ─────────────────────────────────────────────────────────────────────────────

class TestMapEmotionToAnimation:
    def test_known_emotions_mapped_correctly(self, node):
        assert node._map_emotion_to_animation("happy") == "happy"
        assert node._map_emotion_to_animation("sad") == "sad"
        assert node._map_emotion_to_animation("angry") == "angry"
        assert node._map_emotion_to_animation("surprised") == "surprised"
        assert node._map_emotion_to_animation("thinking") == "thinking"
        assert node._map_emotion_to_animation("excited") == "victory"
        assert node._map_emotion_to_animation("confused") == "thinking"
        assert node._map_emotion_to_animation("worried") == "sad"

    def test_neutral_and_calm_map_to_idle(self, node):
        assert node._map_emotion_to_animation("neutral") == "idle"
        assert node._map_emotion_to_animation("calm") == "idle"

    def test_unknown_emotion_returns_idle(self, node):
        assert node._map_emotion_to_animation("furious") == "idle"
        assert node._map_emotion_to_animation("") == "idle"
        assert node._map_emotion_to_animation("XYZ") == "idle"

    def test_case_insensitive(self, node):
        assert node._map_emotion_to_animation("HAPPY") == "happy"
        assert node._map_emotion_to_animation("Sad") == "sad"
        assert node._map_emotion_to_animation("NEUTRAL") == "idle"


# ─────────────────────────────────────────────────────────────────────────────
#  _generate_fallback_response
# ─────────────────────────────────────────────────────────────────────────────

class TestGenerateFallbackResponse:
    def test_greeting_returns_no_internet_message(self, node):
        resp = node._generate_fallback_response("Привет!")
        assert "интернет" in resp.lower()

    def test_hello_english(self, node):
        resp = node._generate_fallback_response("hello robot")
        assert "интернет" in resp.lower()

    def test_how_are_you(self, node):
        resp = node._generate_fallback_response("как дела?")
        assert "интернет" in resp.lower()

    def test_thanks(self, node):
        resp = node._generate_fallback_response("спасибо тебе!")
        assert resp == "Пожалуйста!"

    def test_bye(self, node):
        resp = node._generate_fallback_response("пока!")
        assert resp == "До свидания!"

    def test_bye_english(self, node):
        resp = node._generate_fallback_response("bye")
        assert resp == "До свидания!"

    def test_unknown_returns_default(self, node):
        resp = node._generate_fallback_response("сделай что-нибудь")
        assert "интернет" in resp.lower()
        assert "недоступен" in resp.lower()


# ─────────────────────────────────────────────────────────────────────────────
#  _detect_volume_intent
# ─────────────────────────────────────────────────────────────────────────────

class TestDetectVolumeIntent:
    def test_louder(self, node):
        assert node._detect_volume_intent("громче пожалуйста") == "louder"
        assert node._detect_volume_intent("сделай громко") == "louder"

    def test_quieter(self, node):
        assert node._detect_volume_intent("говори тише") == "quieter"
        assert node._detect_volume_intent("потише пожалуйста") == "quieter"

    def test_volume_max(self, node):
        # "говори громко" → специфичный паттерн max, не louder
        assert node._detect_volume_intent("говори громко") == "max"
        assert node._detect_volume_intent("максимальная громкость") == "max"

    def test_volume_normal(self, node):
        # max/normal проверяются до louder — "громкость" не перехватывается r"громко"
        assert node._detect_volume_intent("нормальная громкость") == "normal"
        assert node._detect_volume_intent("стандартная громкость") == "normal"
        assert node._detect_volume_intent("обычная громкость") == "normal"

    def test_no_match_returns_none(self, node):
        assert node._detect_volume_intent("расскажи анекдот") is None
        assert node._detect_volume_intent("") is None

    def test_case_insensitive_via_lower(self, node):
        assert node._detect_volume_intent("ТИШЕ") == "quieter"


# ─────────────────────────────────────────────────────────────────────────────
#  _detect_pitch_intent
# ─────────────────────────────────────────────────────────────────────────────

class TestDetectPitchIntent:
    def test_higher(self, node):
        assert node._detect_pitch_intent("говори выше") == "higher"
        assert node._detect_pitch_intent("повысь голос") == "higher"

    def test_lower(self, node):
        assert node._detect_pitch_intent("говори ниже") == "lower"
        assert node._detect_pitch_intent("голос ниже") == "lower"

    def test_normal(self, node):
        assert node._detect_pitch_intent("нормальный голос") == "normal"
        assert node._detect_pitch_intent("говори нормально") == "normal"
        assert node._detect_pitch_intent("обычный голос") == "normal"

    def test_no_match_returns_none(self, node):
        assert node._detect_pitch_intent("расскажи сказку") is None
        assert node._detect_pitch_intent("") is None


# ─────────────────────────────────────────────────────────────────────────────
#  _speak_simple
# ─────────────────────────────────────────────────────────────────────────────

class TestSpeakSimple:
    def test_publishes_ssml_json(self, node):
        node._speak_simple("Привет мир")

        node.response_pub.publish.assert_called_once()
        published = node.response_pub.publish.call_args[0][0]
        data = json.loads(published.data)
        assert "<speak>Привет мир</speak>" in data["ssml"]
        assert "dialogue_id" in data

    def test_sets_current_dialogue_id(self, node):
        node._speak_simple("Тест")
        assert node.current_dialogue_id is not None

    def test_without_error_animation_no_anim_publish(self, node):
        node._speak_simple("Тест", show_error_animation=False)
        node.animation_pub.publish.assert_not_called()

    def test_with_error_animation_publishes_error(self, node):
        node._speak_simple("Ошибка", show_error_animation=True)
        node.animation_pub.publish.assert_called_once()
        anim_data = node.animation_pub.publish.call_args[0][0].data
        assert "error" in anim_data

    def test_dialogue_id_unique_per_call(self, node):
        node._speak_simple("раз")
        id1 = node.current_dialogue_id
        node._speak_simple("два")
        id2 = node.current_dialogue_id
        assert id1 != id2


# ─────────────────────────────────────────────────────────────────────────────
#  _trigger_sound
# ─────────────────────────────────────────────────────────────────────────────

class TestTriggerSound:
    def test_publishes_sound_name(self, node):
        node._trigger_sound("startup")

        node.sound_trigger_pub.publish.assert_called_once()
        msg = node.sound_trigger_pub.publish.call_args[0][0]
        assert msg.data == "startup"

    def test_no_crash_on_publish_exception(self, node):
        node.sound_trigger_pub.publish.side_effect = RuntimeError("bus error")
        # Должен поглотить ошибку без краша
        node._trigger_sound("startup")


# ─────────────────────────────────────────────────────────────────────────────
#  _on_mcp_tools_update
# ─────────────────────────────────────────────────────────────────────────────

class TestOnMcpToolsUpdate:
    def _make_msg(self, data: str):
        msg = MagicMock()
        msg.data = data
        return msg

    def test_valid_json_sets_tools(self, node):
        tools = [
            {"function": {"name": "navigate"}, "type": "function"},
            {"function": {"name": "speak"}, "type": "function"},
        ]
        node._on_mcp_tools_update(self._make_msg(json.dumps(tools)))

        assert node.available_tools == tools
        assert node.mcp_tools_available is True

    def test_empty_list_clears_tools(self, node):
        node.available_tools = [{"function": {"name": "old"}}]
        node._on_mcp_tools_update(self._make_msg("[]"))
        assert node.available_tools == []
        assert node.mcp_tools_available is True

    def test_invalid_json_sets_unavailable(self, node):
        node.mcp_tools_available = True
        node._on_mcp_tools_update(self._make_msg("{invalid json"))
        assert node.mcp_tools_available is False

    def test_invalid_json_logs_error(self, node):
        node._on_mcp_tools_update(self._make_msg("not json"))
        node.get_logger().error.assert_called()


# ─────────────────────────────────────────────────────────────────────────────
#  _on_perception_update
# ─────────────────────────────────────────────────────────────────────────────

class TestOnPerceptionUpdate:
    def _make_msg(self, internet=None, time_json=None):
        msg = MagicMock(spec=[])  # spec=[] → нет атрибутов по умолчанию
        if internet is not None:
            msg.internet_available = internet
        if time_json is not None:
            msg.time_context_json = time_json
        return msg

    def test_internet_available_true(self, node):
        node.internet_available = False
        node._on_perception_update(self._make_msg(internet=True))
        assert node.internet_available is True

    def test_internet_becomes_unavailable(self, node):
        node.internet_available = True
        node._on_perception_update(self._make_msg(internet=False))
        assert node.internet_available is False

    def test_time_context_json_parsed(self, node):
        time_data = {"time_only": "12:30", "date": "2026-02-21"}
        node._on_perception_update(self._make_msg(time_json=json.dumps(time_data)))
        assert node.current_time_info["time_only"] == "12:30"

    def test_invalid_time_json_no_crash(self, node):
        node._on_perception_update(self._make_msg(time_json="bad json"))
        # Не падает, логирует warning
        node.get_logger().warning.assert_called()

    def test_msg_without_attrs_no_crash(self, node):
        """Сообщение без internet_available и time_context_json."""
        msg = MagicMock(spec=[])
        node._on_perception_update(msg)  # не должно падать


# ─────────────────────────────────────────────────────────────────────────────
#  vad_callback
# ─────────────────────────────────────────────────────────────────────────────

class TestVadCallback:
    def _make_msg(self, active: bool):
        msg = MagicMock()
        msg.data = active
        return msg

    def test_rising_edge_sets_vad_speech_detected(self, node):
        node.vad_speech_detected = False
        node._vad_callback(self._make_msg(True)) if hasattr(node, '_vad_callback') \
            else node.vad_callback(self._make_msg(True))
        assert node.vad_speech_detected is True

    def test_rising_edge_during_llm_sets_interrupt(self, node):
        node.vad_speech_detected = False
        node.llm_processing = True
        node.mcp_tools_available = False
        node.vad_callback(self._make_msg(True))
        assert node.interrupt_agent_loop is True

    def test_no_interrupt_when_not_processing(self, node):
        node.vad_speech_detected = False
        node.llm_processing = False
        node.vad_callback(self._make_msg(True))
        assert node.interrupt_agent_loop is False

    def test_falling_edge_clears_vad_speech_detected(self, node):
        node.vad_speech_detected = True
        node.vad_callback(self._make_msg(False))
        assert node.vad_speech_detected is False

    def test_no_re_trigger_if_already_detected(self, node):
        """Rising edge уже был: повторный True не должен ставить interrupt."""
        node.vad_speech_detected = True  # уже детектирован
        node.llm_processing = True
        node.interrupt_agent_loop = False
        node.mcp_tools_available = False
        node.vad_callback(self._make_msg(True))
        # Нет rising edge (уже было True), interrupt не трогаем
        assert node.interrupt_agent_loop is False
