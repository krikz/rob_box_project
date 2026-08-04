"""
test_dialogue_speak_text_batch.py — issue #980 batch_complete coverage.

SpeakTextTool разбивает длинный текст на N чанков и публикует каждый
через /voice/tts/request. Эти тесты проверяют, что:
  * каждый чанк публикуется с одинаковым batch_id в payload;
  * в /voice/tts/batch_complete публикация приходит РОВНО ОДИН РАЗ после
    последнего tts/finished, с правильными chunks_total и batch_duration_ms;
  * batch_complete не публикуется преждевременно (после первого finished
    из четырёх — это и есть корень issue #980);
  * если чанк упал (success=False), batch_complete всё равно
    публикуется — cleanup должен сработать даже при сбое одного чанка;
  * одиночный speak_text (короткий текст, 1 чанк) тоже публикует
    batch_complete — обратная совместимость с dialogue_node подписчиком.

Mocked ROS2: see conftest.py MockNode.
"""

import json
import sys
import time
from unittest.mock import Mock

import pytest

# Mock std_msgs перед импортом SpeakTextTool — это локальный паттерн
# для test_tools/ (см. test_animation.py).
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Заглушаем navigation, который тянет rclpy через tools/__init__.py.
# Без этого тест не собрать в окружениях без ROS 2 (CI linter,
# локальный pytest без colcon build).
sys.modules.setdefault(
    "rob_box_mcp_tools.tools.navigation", Mock()
)
sys.modules.setdefault(
    "rob_box_mcp_tools.tools.system", Mock()
)
sys.modules.setdefault(
    "rob_box_mcp_tools.tools.perception", Mock()
)
sys.modules.setdefault(
    "rob_box_mcp_tools.tools.mapping", Mock()
)
sys.modules.setdefault(
    "rob_box_mcp_tools.tools.memory", Mock()
)
sys.modules.setdefault(
    "rob_box_mcp_tools.tools.music", Mock()
)
# animation / sound / dialogue — настоящие, не подменяем.

from rob_box_mcp_tools.tools.dialogue import SpeakTextTool  # noqa: E402


def _make_finished_msg(speech_id: str, success: bool = True):
    """Создать String-подобный mock с payload /voice/tts/finished."""
    msg = Mock()
    msg.data = json.dumps({"speech_id": speech_id, "success": success})
    return msg


def _published_payloads(pub) -> list:
    """Достать payload'ы из MockPublisher."""
    out = []
    for m in pub.published_messages:
        try:
            out.append(json.loads(m.data))
        except (TypeError, ValueError):
            out.append({"_raw": m.data})
    return out


@pytest.mark.unit
class TestSpeakTextBatchTracking:
    """Issue #980 — batch_complete после последнего чанка."""

    def test_short_text_publishes_batch_complete_after_one_finished(
        self, mock_node
    ):
        """Короткий текст (1 чанк) → batch_complete после первого finished."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет", animation="idle")
        assert result.success is True
        chunks_total = result.data["chunks"]
        assert chunks_total == 1

        # Один TTS-запрос улетел
        tts_pub = mock_node.get_publisher("/voice/tts/request")
        assert tts_pub is not None
        assert len(tts_pub.published_messages) == 1
        request_payload = json.loads(tts_pub.published_messages[0].data)
        speech_id = request_payload["speech_id"]
        batch_id = request_payload["batch_id"]

        # batch_complete ещё не улетел
        bc_pub = mock_node.get_publisher("/voice/tts/batch_complete")
        assert bc_pub is not None
        assert bc_pub.published_messages == []

        # Прилетел finished
        tool._on_tts_finished(_make_finished_msg(speech_id, success=True))

        # batch_complete улетел ровно один раз
        assert len(bc_pub.published_messages) == 1
        payload = json.loads(bc_pub.published_messages[0].data)
        assert payload["batch_id"] == batch_id
        assert payload["chunks_total"] == 1
        assert payload["success"] is True
        assert payload["batch_duration_ms"] >= 0

    def test_long_text_emits_batch_complete_only_after_last_chunk(
        self, mock_node
    ):
        """Длинный текст (N=4 чанков) → batch_complete строго после 4-го.

        Это и есть acceptance для issue #980: после 1-го, 2-го и 3-го
        finished НЕ должно быть batch_complete. Только после 4-го.
        """
        tool = SpeakTextTool(mock_node)
        # Текст который _split_sentences точно разобьёт на несколько чанков
        # (длина > _MAX_CHUNK_CHARS = 200). Используем осмысленные
        # предложения чтобы split работал предсказуемо.
        text = (
            "Первое предложение о рэпе и музыке, "
            "чтобы пройти лимит. "
            "Второе предложение продолжает тему и добавляет эмоций. "
            "Третье предложение рассказывает историю в деталях, "
            "с подробностями и поворотами сюжета. "
            "Четвёртое предложение закрывает тему с финальным аккордом, "
            "последние мысли и точки над и."
        )
        result = tool.execute(text=text, animation="idle")
        assert result.success is True
        chunks_total = result.data["chunks"]
        assert chunks_total >= 2, (
            f"Test precondition: текст должен делиться на ≥2 чанков; "
            f"получили {chunks_total}"
        )

        tts_pub = mock_node.get_publisher("/voice/tts/request")
        bc_pub = mock_node.get_publisher("/voice/tts/batch_complete")
        assert tts_pub is not None and bc_pub is not None
        assert len(tts_pub.published_messages) == chunks_total
        assert bc_pub.published_messages == []

        # Все чанки шарят один batch_id
        batch_ids = {
            json.loads(m.data)["batch_id"]
            for m in tts_pub.published_messages
        }
        assert batch_ids == {result.data["batch_id"]}

        # Проигрываем finished для всех, кроме последнего — batch_complete
        # ещё НЕ должен уйти. Это ключевая проверка issue #980.
        speech_ids = result.data["speech_ids"]
        for sid in speech_ids[:-1]:
            tool._on_tts_finished(_make_finished_msg(sid, success=True))
            assert bc_pub.published_messages == [], (
                "batch_complete опубликован преждевременно — "
                "issue #980 не исправлен!"
            )

        # Последний чанк
        tool._on_tts_finished(
            _make_finished_msg(speech_ids[-1], success=True)
        )
        assert len(bc_pub.published_messages) == 1
        payload = json.loads(bc_pub.published_messages[0].data)
        assert payload["chunks_total"] == chunks_total
        assert payload["success"] is True
        assert payload["batch_id"] == result.data["batch_id"]

    def test_batch_with_failing_chunk_still_publishes_batch_complete(
        self, mock_node
    ):
        """Если один чанк упал, batch_complete всё равно публикуется.

        Cleanup музыки должен сработать даже при сбое — иначе музыка
        зациклится после падения TTS.
        """
        tool = SpeakTextTool(mock_node)
        text = (
            "Длинный фрагмент текста для надёжного деления на несколько "
            "чанков внутри одного speak_text вызова робота ассистента. "
            "Второй фрагмент продолжает мысль и добавляет новые идеи "
            "о роботах и их взаимодействии с человеком в быту. "
            "Третий фрагмент логически завершает повествование, "
            "чтобы лимит чанков точно был превышен."
        )
        result = tool.execute(text=text, animation="idle")
        chunks_total = result.data["chunks"]
        assert chunks_total >= 2
        bc_pub = mock_node.get_publisher("/voice/tts/batch_complete")

        speech_ids = result.data["speech_ids"]
        # Все, кроме последнего — успешные
        for sid in speech_ids[:-1]:
            tool._on_tts_finished(_make_finished_msg(sid, success=True))
        # Последний упал
        tool._on_tts_finished(
            _make_finished_msg(speech_ids[-1], success=False)
        )
        assert len(bc_pub.published_messages) == 1
        payload = json.loads(bc_pub.published_messages[0].data)
        assert payload["chunks_total"] == chunks_total
        assert payload["success"] is False

    def test_finished_for_unknown_speech_does_not_publish_batch_complete(
        self, mock_node
    ):
        """finished с неизвестным speech_id — no-op (не крашимся)."""
        tool = SpeakTextTool(mock_node)
        bc_pub = mock_node.get_publisher("/voice/tts/batch_complete")
        # Нет ни одного зарегистрированного speech — finished не должен
        # ничего публиковать и не должен падать.
        tool._on_tts_finished(_make_finished_msg("ghost-speech-id"))
        assert bc_pub.published_messages == []

    def test_finished_with_malformed_payload_does_not_crash(
        self, mock_node
    ):
        """Не-JSON payload не должен ронять обработчик."""
        tool = SpeakTextTool(mock_node)
        bc_pub = mock_node.get_publisher("/voice/tts/batch_complete")
        msg = Mock()
        msg.data = "this is not json"
        # Должно залогировать ошибку и не упасть.
        tool._on_tts_finished(msg)
        assert bc_pub.published_messages == []

    def test_batch_duration_ms_is_monotonic(self, mock_node):
        """batch_duration_ms не отрицательный и не больше timeout'а теста."""
        tool = SpeakTextTool(mock_node)
        result = tool.execute(text="Привет", animation="idle")
        speech_id = result.data["speech_ids"][0]
        # Небольшая искусственная задержка чтобы duration_ms был >0
        time.sleep(0.005)
        tool._on_tts_finished(_make_finished_msg(speech_id, success=True))
        bc_pub = mock_node.get_publisher("/voice/tts/batch_complete")
        payload = json.loads(bc_pub.published_messages[0].data)
        assert 0 <= payload["batch_duration_ms"] < 60_000

    def test_separate_speak_text_calls_get_separate_batches(
        self, mock_node
    ):
        """Два speak_text(...) вызова порождают два разных batch_id."""
        tool = SpeakTextTool(mock_node)
        r1 = tool.execute(text="Первый короткий текст", animation="idle")
        r2 = tool.execute(text="Второй короткий текст", animation="idle")
        assert r1.data["batch_id"] != r2.data["batch_id"]

        bc_pub = mock_node.get_publisher("/voice/tts/batch_complete")
        # Завершаем первый вызов
        for sid in r1.data["speech_ids"]:
            tool._on_tts_finished(_make_finished_msg(sid, success=True))
        assert len(bc_pub.published_messages) == 1
        first_payload = json.loads(bc_pub.published_messages[0].data)
        assert first_payload["batch_id"] == r1.data["batch_id"]

        # Завершаем второй — появляется второй batch_complete
        for sid in r2.data["speech_ids"]:
            tool._on_tts_finished(_make_finished_msg(sid, success=True))
        assert len(bc_pub.published_messages) == 2
        second_payload = json.loads(bc_pub.published_messages[1].data)
        assert second_payload["batch_id"] == r2.data["batch_id"]
        assert second_payload["batch_id"] != first_payload["batch_id"]
