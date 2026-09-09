"""
test_issue_1709_speak_text_logging.py — full-text TTS logging + Unicode guard.

Issue #1709 (live 28.08, «робот бормотал на хинди»):

* в логе ``mcp_server`` для failed/прерванного чанка оставался ТОЛЬКО
  ``speech_id`` — восстановить, что робот пытался сказать, было
  невозможно (архитектор нашёл 2 failed TTS без текста);
* LLM периодически вставляет в ``speak_text`` иероглифы/деванагари,
  TTS их бормочет.

Acceptance issue #1709, покрытый здесь:

1. TTS логирует ПОЛНЫЙ текст failed/прерванного чанка;
2. логи содержат ``voice=`` + ``text=`` для КАЖДОГО чанка;
3. чанк с >10% символов чужих письменностей НЕ произносится
   (warning + ``unsupported_script``, ни одного ``/voice/tts/request``).

Prompt-половина фикса пиннится в
``src/rob_box_voice/test/unit/test_issue_1709_prompt_unicode_speech.py``,
сам guard — в
``src/rob_box_voice/test/unit/core/test_tts_text_guard.py``.

Mocked ROS2: см. conftest.py MockNode.
"""

import json
import sys
from unittest.mock import Mock

import pytest

# Mock std_msgs перед импортом SpeakTextTool — локальный паттерн
# test_tools/ (см. test_dialogue_speak_text_batch.py).
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Заглушаем модули, тянущие rclpy через tools/__init__.py.
for _mod in (
    "rob_box_mcp_tools.tools.navigation",
    "rob_box_mcp_tools.tools.system",
    "rob_box_mcp_tools.tools.perception",
    "rob_box_mcp_tools.tools.mapping",
    "rob_box_mcp_tools.tools.memory",
    "rob_box_mcp_tools.tools.music",
):
    sys.modules.setdefault(_mod, Mock())

from rob_box_mcp_tools.tools.dialogue import SpeakTextTool  # noqa: E402


# Guard живёт в rob_box_voice; в minimal-окружении (CI linter без
# rob_box_voice на PYTHONPATH) dialogue.py подставляет no-op fallback и
# фильтр не работает — такие тесты честно скипаем, вместо «зелёного»
# прогона, который ничего не проверил (ADR-0018).
try:  # pragma: no cover — окруженческая развилка
    import rob_box_voice.tts_text_guard  # noqa: F401

    _GUARD_AVAILABLE = True
except ImportError:  # pragma: no cover
    _GUARD_AVAILABLE = False

requires_guard = pytest.mark.skipif(
    not _GUARD_AVAILABLE,
    reason="rob_box_voice.tts_text_guard недоступен (minimal env)",
)


def _make_finished_msg(speech_id: str, success: bool = True, error=None):
    payload = {"speech_id": speech_id, "success": success}
    if error is not None:
        payload["error"] = error
    msg = Mock()
    msg.data = json.dumps(payload)
    return msg


def _joined(messages) -> str:
    return "\n".join(messages)


# ── AC1/AC5: полный текст + голос в логах КАЖДОГО чанка ───────────────


@pytest.mark.unit
class TestFullTextLogging:
    def test_publish_log_contains_full_text_and_voice(self, mock_node):
        """Лог публикации чанка несёт voice= и НЕполный-обрезанный text=."""
        tool = SpeakTextTool(mock_node)
        # >40 символов: до фикса лог обрезался на 40 (`chunk[:40]`).
        text = (
            "Жил да был енотик полосатый, он катался по дороге "
            "и мурлыкал песенку"
        )
        result = tool.execute(text=text, animation="happy")
        assert result.success is True

        logs = _joined(mock_node.get_logger().info_messages)
        assert "text=" in logs
        assert "voice=" in logs
        # Полный текст, а не префикс.
        assert text in logs

    def test_finished_success_log_contains_text_and_voice(self, mock_node):
        """AC5: finished-лог содержит voice= + text= (а не только speech_id)."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет, шифу", animation="idle")
        speech_id = result.data["speech_ids"][0]
        voice_used = result.data["voice_used"]
        mock_node.get_logger().info_messages.clear()

        tool._on_tts_finished(_make_finished_msg(speech_id, success=True))

        logs = _joined(mock_node.get_logger().info_messages)
        assert "TTS finished" in logs
        assert "Привет, шифу" in logs
        assert f"voice={voice_used}" in logs

    def test_failed_chunk_logs_full_text_as_warning(self, mock_node):
        """AC1 (ядро issue): текст failed-чанка попадает в лог как WARNING.

        Именно здесь раньше терялся текст: юзер слышал странный фрагмент,
        а в логе был только speech_id + success=False.
        """
        tool = SpeakTextTool(mock_node)
        text = "Нига-стайл, Колобок-флоу, уехал я, братан"
        result = tool.execute(text=text, animation="happy")
        speech_id = result.data["speech_ids"][0]

        tool._on_tts_finished(
            _make_finished_msg(speech_id, success=False, error="stopped")
        )

        warnings = _joined(mock_node.get_logger().warning_messages)
        assert "НЕ произнесён" in warnings
        assert text in warnings
        assert "error='stopped'" in warnings

    def test_interrupted_chunk_of_batch_logs_its_own_text(self, mock_node):
        """Прерванный чанк МНОГОЧАНКОВОГО батча логирует ИМЕННО свой текст.

        Это кейс из issue: TTS прервали в середине батча, юзер услышал
        хвост, а по логу нельзя было понять какой именно чанк играл.
        """
        tool = SpeakTextTool(mock_node)
        text = (
            "Дед меня слепил, бабка меня жгла, я ушёл от них по дороге "
            "прямо в лес густой. "
            "Заяц меня ел, я его обманул, волк меня жрал, я ему не дал. "
            "Медведь пришёл, я ему сказал отойди, а лиса меня съела "
            "и на этом всё закончилось."
        )
        result = tool.execute(text=text, animation="idle")
        speech_ids = result.data["speech_ids"]
        assert len(speech_ids) >= 2, "precondition: нужен многочанковый батч"

        # Падает ПОСЛЕДНИЙ чанк — логируется его собственный текст.
        tool._on_tts_finished(
            _make_finished_msg(speech_ids[-1], success=False, error="stopped")
        )
        warnings = _joined(mock_node.get_logger().warning_messages)
        assert "НЕ произнесён" in warnings
        # В логе — фрагмент именно последнего чанка (он содержит финал фразы).
        assert "закончилось" in warnings

    def test_unknown_speech_finished_stays_debug_not_warning(self, mock_node):
        """Чужой finished (общий топик, issue #776) не должен стать warning.

        Регрессия на issue #776: /voice/tts/finished общий для всех
        отправителей; наш новый логгер не должен превращать штатное
        событие в warning.
        """
        tool = SpeakTextTool(mock_node)
        tool._on_tts_finished(_make_finished_msg("ghost-speech-id"))

        assert mock_node.get_logger().warning_messages == []
        debug = _joined(mock_node.get_logger().debug_messages)
        assert "не наш чанк" in debug


# ── AC2/AC3: Unicode guard ────────────────────────────────────────────


@pytest.mark.unit
class TestUnicodeScriptGuard:
    @requires_guard
    def test_hieroglyph_only_text_is_not_spoken(self, mock_node):
        """Чанк из иероглифов не уходит в TTS вообще."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="你好世界，加油加油", animation="happy")

        assert result.success is False
        assert result.error == "unsupported_script"
        tts_pub = mock_node.get_publisher("/voice/tts/request")
        assert tts_pub.published_messages == [], (
            "текст с иероглифами не должен попадать в /voice/tts/request"
        )

    @requires_guard
    def test_devanagari_text_is_not_spoken(self, mock_node):
        """«Хинди» из репорта юзера — тот же путь."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="नमस्ते दुनिया", animation="idle")

        assert result.success is False
        assert result.error == "unsupported_script"
        assert mock_node.get_publisher("/voice/tts/request").published_messages == []

    @requires_guard
    def test_rejected_text_is_logged_in_full_with_diagnostics(self, mock_node):
        """AC1+AC3: отклонённый текст логируется целиком + доля/письменности."""
        tool = SpeakTextTool(mock_node)
        text = "Слушай: 加油加油加油加油加油加油"
        tool.execute(text=text, animation="happy")

        warnings = _joined(mock_node.get_logger().warning_messages)
        assert "issue 1709" in warnings
        assert text in warnings
        assert "foreign_ratio=" in warnings
        assert "cjk" in warnings

    @requires_guard
    def test_rejection_result_tells_llm_to_transliterate(self, mock_node):
        """LLM должна получить actionable ответ, а не молчаливый провал."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="你好世界，加油加油", animation="idle")

        assert "транслитерацией" in result.message
        assert result.data["scripts"] == ["cjk"]
        assert result.data["foreign_ratio"] > 0.10

    @requires_guard
    def test_russian_text_with_single_hieroglyph_is_still_spoken(
        self, mock_node
    ):
        """Порог 10%: одно вкрапление не глушит длинную русскую фразу.

        Иначе робот замолкал бы на любой мелочи — это было бы хуже бага.
        """
        tool = SpeakTextTool(mock_node)
        text = (
            "Китайская идиома звучит как вай го, а пишется она "
            "вот таким единственным символом 加 в самом конце фразы"
        )
        result = tool.execute(text=text, animation="idle")

        assert result.success is True
        assert mock_node.get_publisher("/voice/tts/request").published_messages

    def test_plain_russian_text_is_unaffected(self, mock_node):
        """Guard не должен трогать нормальную русскую речь (back-compat)."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(
            text="Привет! Я РОББОКС, поехали кататься.", animation="happy"
        )
        assert result.success is True
        assert result.error is None
        assert len(
            mock_node.get_publisher("/voice/tts/request").published_messages
        ) == result.data["chunks"]

    def test_emoji_and_digits_do_not_trip_the_guard(self, mock_node):
        """Эмодзи/цифры — не «чужая письменность» (иначе бы всё замолчало)."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(
            text="Погнали 2024! 🎶🦝 128 bpm, брат!", animation="happy"
        )
        assert result.success is True
