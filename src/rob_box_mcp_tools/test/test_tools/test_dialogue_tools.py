"""
test_dialogue_tools.py - Unit тесты для SpeakTextTool / ListenForResponseTool /
EstimateTtsDurationTool (RegisterSpeakerTool покрыт в test_dialogue_register_speaker.py).

Покрытие: success + error + invalid params + batch/finished логика.
ROS 2 модули мокаются — тесты работают без робота.
"""

import json
import sys
import threading
from unittest.mock import MagicMock, Mock, patch

import pytest

# Mock ROS 2 модулей перед импортом tools
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "geometry_msgs",
    "geometry_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())


class _FakeStringMsg:
    """Суррогат std_msgs.msg.String.

    MagicMock-версия возвращает ОДИН и тот же объект на каждый вызов
    String(), поэтому ``msg.data = ...`` для TTS-чанка затирал бы data
    анимационного сообщения (оба — один объект). Настоящий класс
    возвращает свежий инстанс с полем ``data``.
    """

    def __init__(self, data=""):
        self.data = data


sys.modules.setdefault("std_msgs", MagicMock())
sys.modules["std_msgs.msg"] = MagicMock()
sys.modules["std_msgs.msg"].String = _FakeStringMsg

from rob_box_mcp_tools.tools.dialogue import (  # noqa: E402
    SpeakTextTool,
    ListenForResponseTool,
    EstimateTtsDurationTool,
)


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warning(self, msg):
        self.messages.append(("warning", msg))

    def error(self, msg):
        self.messages.append(("error", msg))

    def debug(self, msg):
        self.messages.append(("debug", msg))


class _FakeMsg:
    def __init__(self, data=""):
        self.data = data


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()
        self.publishers = {}
        self.subscriptions = {}

    def get_logger(self):
        return self._logger

    def create_publisher(self, msg_type, topic, qos):
        pub = Mock()
        pub.msg_type = msg_type
        pub.topic = topic
        pub.qos = qos
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = Mock()
        sub.msg_type = msg_type
        sub.topic = topic
        sub.callback = callback
        sub.qos = qos
        self.subscriptions[topic] = sub
        return sub


@pytest.fixture
def fake_node():
    return _FakeNode()


# ---------------------------------------------------------------------------
# SpeakTextTool — metadata, split, publish
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSpeakTextTool:
    def test_tool_metadata(self, fake_node):
        tool = SpeakTextTool(fake_node)
        assert tool.name == "speak_text"
        assert [p.name for p in tool.parameters] == ["text", "animation"]

    def test_execute_empty_text_returns_error(self, fake_node):
        tool = SpeakTextTool(fake_node)

        result = tool.execute(text="   ")

        assert result.success is False
        assert "Пустой текст" in result.error

    def test_execute_simple_text_publishes(self, fake_node):
        tool = SpeakTextTool(fake_node)

        result = tool.execute(text="Привет!")

        assert result.success is True
        assert result.data["chunks"] == 1
        assert result.data["animation"] == "idle"  # neutral → idle (известная анимация)
        assert result.data["async"] is True
        tts_pub = fake_node.publishers["/voice/tts/request"]
        assert tts_pub.publish.called
        payload = json.loads(tts_pub.publish.call_args[0][0].data)
        assert payload["ssml"].startswith("<speak>")
        assert payload["speech_id"]
        assert payload["batch_id"]

    def test_execute_happy_animation_sets_pitch(self, fake_node):
        tool = SpeakTextTool(fake_node)

        result = tool.execute(text="Ура!", animation="happy")

        assert result.success is True
        assert result.data["animation"] == "happy"
        tts_pub = fake_node.publishers["/voice/tts/request"]
        payload = json.loads(tts_pub.publish.call_args[0][0].data)
        assert "<prosody pitch='high'>" in payload["ssml"]

    def test_execute_sets_animation_publisher_first_chunk(self, fake_node):
        tool = SpeakTextTool(fake_node)
        # Длинный текст (больше 200 символов) → несколько чанков
        long_text = "Первый куплет рэпа. " * 25 + "Второй куплет рэпа. " * 25

        result = tool.execute(text=long_text, animation="happy")

        assert result.success is True
        assert result.data["chunks"] > 1
        anim_pub = fake_node.publishers["/voice/animation/request"]
        assert anim_pub.publish.called
        assert anim_pub.publish.call_args[0][0].data == "happy"

    def test_execute_unknown_animation_falls_back_to_talking(self, fake_node):
        tool = SpeakTextTool(fake_node)

        result = tool.execute(text="Привет", animation="nonexistent_anim")

        assert result.success is True
        assert result.data["animation"] == "talking"

    def test_execute_strips_injected_tool_call_syntax(self, fake_node):
        """LLM иногда включает параметры вызова в текст — вырезаем."""
        tool = SpeakTextTool(fake_node)

        result = tool.execute(text='Привет!", animation="happy"')

        assert result.success is True
        assert '"animation' not in result.data["text"]
        assert result.data["text"].startswith("Привет")

    def test_execute_publishes_batch_registered_prelude(self, fake_node):
        tool = SpeakTextTool(fake_node)
        long_text = "Раз. " * 100 + "Два. " * 100  # > 200 символов → несколько чанков

        tool.execute(text=long_text, animation="happy")

        reg_pub = fake_node.publishers["/voice/tts/batch_registered"]
        assert reg_pub.publish.called
        payload = json.loads(reg_pub.publish.call_args[0][0].data)
        assert payload["chunks_total"] > 1
        assert payload["batch_id"]

    def test_execute_tracks_pending_speeches_and_batches(self, fake_node):
        tool = SpeakTextTool(fake_node)
        long_text = "Раз. " * 100 + "Два. " * 100  # > 200 символов → несколько чанков

        result = tool.execute(text=long_text, animation="happy")

        assert len(result.data["speech_ids"]) > 1
        batch_id = result.data["batch_id"]
        assert batch_id in tool.pending_batches
        assert tool.pending_batches[batch_id]["remaining"] > 1
        assert tool.pending_batches[batch_id]["total"] > 1
        for sid in result.data["speech_ids"]:
            assert sid in tool.pending_speeches
            assert tool.pending_speeches[sid]["batch_id"] == batch_id

    def test_execute_registers_dialogue_id_when_known(self, fake_node):
        tool = SpeakTextTool(fake_node)
        tool._current_dialogue_id = "dialogue-1"

        tool.execute(text="Привет")

        tts_pub = fake_node.publishers["/voice/tts/request"]
        payload = json.loads(tts_pub.publish.call_args[0][0].data)
        assert payload["dialogue_id"] == "dialogue-1"

    def test_on_current_dialogue_id_updates(self, fake_node):
        tool = SpeakTextTool(fake_node)
        tool._on_current_dialogue_id(_FakeMsg("dialogue-42"))
        assert tool._current_dialogue_id == "dialogue-42"


# ---------------------------------------------------------------------------
# SpeakTextTool — _on_tts_finished / batch complete (issue #980)
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSpeakTextTtsFinished:
    def _make_tool_with_batch(self, fake_node):
        tool = SpeakTextTool(fake_node)
        long_text = "Раз. " * 100 + "Два. " * 100  # > 200 символов → несколько чанков
        result = tool.execute(text=long_text, animation="happy")
        batch_id = result.data["batch_id"]
        speech_ids = result.data["speech_ids"]
        return tool, batch_id, speech_ids

    def test_finished_updates_remaining_and_publishes_complete_on_last(self, fake_node):
        tool, batch_id, speech_ids = self._make_tool_with_batch(fake_node)
        total = len(speech_ids)

        # Все чанки, кроме последнего — батч ещё не полный
        for sid in speech_ids[:-1]:
            tool._on_tts_finished(_FakeMsg(json.dumps({"speech_id": sid, "success": True})))
        assert tool.pending_batches[batch_id]["remaining"] == 1

        # Последний чанк — publish batch_complete
        tool._on_tts_finished(_FakeMsg(json.dumps({"speech_id": speech_ids[-1], "success": True})))
        assert batch_id not in tool.pending_batches
        complete_pub = fake_node.publishers["/voice/tts/batch_complete"]
        assert complete_pub.publish.called
        payload = json.loads(complete_pub.publish.call_args[0][0].data)
        assert payload["chunks_total"] == total
        assert payload["success"] is True
        assert "batch_duration_ms" in payload

    def test_finished_marks_batch_unsuccessful_on_chunk_failure(self, fake_node):
        tool, batch_id, speech_ids = self._make_tool_with_batch(fake_node)

        tool._on_tts_finished(_FakeMsg(json.dumps({"speech_id": speech_ids[0], "success": False})))
        for sid in speech_ids[1:]:
            tool._on_tts_finished(_FakeMsg(json.dumps({"speech_id": sid, "success": True})))

        complete_pub = fake_node.publishers["/voice/tts/batch_complete"]
        payload = json.loads(complete_pub.publish.call_args[0][0].data)
        assert payload["success"] is False

    def test_finished_unknown_speech_id_is_debug_not_crash(self, fake_node):
        tool, _, _ = self._make_tool_with_batch(fake_node)

        tool._on_tts_finished(_FakeMsg(json.dumps({"speech_id": "unknown", "success": True})))

        assert any(m[0] == "debug" for m in fake_node._logger.messages)

    def test_finished_corrupt_json_logs_error(self, fake_node):
        tool, _, _ = self._make_tool_with_batch(fake_node)

        tool._on_tts_finished(_FakeMsg("{corrupt"))

        assert any("Ошибка парсинга" in m[1] for m in fake_node._logger.messages if m[0] == "error")

    def test_finished_missing_speech_id_ignored(self, fake_node):
        tool, _, _ = self._make_tool_with_batch(fake_node)

        tool._on_tts_finished(_FakeMsg(json.dumps({"success": True})))

        # no crash; батч остаётся
        assert len(tool.pending_batches) == 1

    def test_finished_batch_not_found_warns(self, fake_node):
        tool, batch_id, speech_ids = self._make_tool_with_batch(fake_node)
        # удаляем батч вручную — имитация рассинхронизации
        tool.pending_batches.pop(batch_id)

        tool._on_tts_finished(_FakeMsg(json.dumps({"speech_id": speech_ids[0], "success": True})))

        assert any("не найден" in m[1] for m in fake_node._logger.messages if m[0] == "warning")


# ---------------------------------------------------------------------------
# SpeakTextTool — _split_sentences
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestSplitSentences:
    def test_short_text_single_chunk(self):
        assert SpeakTextTool._split_sentences("Привет мир") == ["Привет мир"]

    def test_splits_on_sentence_endings(self):
        # Два предложения короче max_len — буферизуются в один чанк.
        chunks = SpeakTextTool._split_sentences("Первое предложение. Второе предложение!", max_len=15)
        assert len(chunks) > 1
        assert chunks[0].startswith("Первое")

    def test_splits_long_sentence_on_commas(self):
        long_text = "Слово, " * 50
        chunks = SpeakTextTool._split_sentences(long_text, max_len=50)
        assert len(chunks) > 1
        assert all(len(c) <= 50 for c in chunks)

    def test_splits_long_word_run_by_words(self):
        text = " ".join(f"слово{i}" for i in range(100))
        chunks = SpeakTextTool._split_sentences(text, max_len=30)
        assert len(chunks) > 1
        assert all(len(c) <= 30 for c in chunks)

    def test_empty_text_returns_empty(self):
        assert SpeakTextTool._split_sentences("   ") == []


# ---------------------------------------------------------------------------
# ListenForResponseTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestListenForResponseTool:
    def test_tool_metadata(self, fake_node):
        tool = ListenForResponseTool(fake_node)
        assert tool.name == "listen_for_response"
        assert [p.name for p in tool.parameters] == ["timeout_seconds", "prompt_text"]

    def test_execute_publishes_stt_request(self, fake_node):
        tool = ListenForResponseTool(fake_node)

        result = tool.execute(timeout_seconds=15, prompt_text="Как тебя зовут?")

        assert result.success is True
        assert result.data["timeout_seconds"] == 15
        assert result.data["prompt_text"] == "Как тебя зовут?"
        pub = fake_node.publishers["/voice/stt/request"]
        assert pub.publish.called
        assert pub.publish.call_args[0][0].data == "listen:15"

    def test_execute_default_timeout(self, fake_node):
        tool = ListenForResponseTool(fake_node)

        result = tool.execute()

        assert result.success is True
        assert result.data["timeout_seconds"] == 30
        pub = fake_node.publishers["/voice/stt/request"]
        assert pub.publish.call_args[0][0].data == "listen:30"


# ---------------------------------------------------------------------------
# EstimateTtsDurationTool
# ---------------------------------------------------------------------------


@pytest.mark.unit
class TestEstimateTtsDurationTool:
    def test_tool_metadata(self, fake_node):
        tool = EstimateTtsDurationTool(fake_node)
        assert tool.name == "estimate_tts_duration"
        assert tool.execution_type.value == "fast"
        assert tool.destructive is False
        assert [p.name for p in tool.parameters] == ["text", "chars_per_second"]

    def test_execute_default_cps(self, fake_node):
        tool = EstimateTtsDurationTool(fake_node)

        result = tool.execute(text="a" * 60)

        assert result.success is True
        assert result.data["chars_per_second"] == 30.0
        assert result.data["estimate_sec"] == 2.0  # 60 / 30

    def test_execute_custom_cps(self, fake_node):
        tool = EstimateTtsDurationTool(fake_node)

        result = tool.execute(text="a" * 100, chars_per_second=25.0)

        assert result.success is True
        assert result.data["chars_per_second"] == 25.0
        assert result.data["estimate_sec"] == 4.0

    def test_execute_cps_floor(self, fake_node):
        tool = EstimateTtsDurationTool(fake_node)

        result = tool.execute(text="a" * 10, chars_per_second=0.1)

        assert result.success is True
        assert result.data["chars_per_second"] == 1.0  # safety floor

    def test_execute_empty_text(self, fake_node):
        tool = EstimateTtsDurationTool(fake_node)

        result = tool.execute(text="")

        assert result.success is True
        assert result.data["estimate_sec"] == 0.0
