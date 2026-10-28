"""
test_animation.py - Unit тесты для инструментов анимации

Тестирует:
- PlayAnimationTool
"""

import pytest
from unittest.mock import Mock, patch

from rob_box_dialog import knowledge as kn

# Mock std_msgs перед импортом tools
import sys
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

from rob_box_mcp_tools.tools.animation import PlayAnimationTool


@pytest.mark.unit
class TestPlayAnimationTool:
    """Тесты для PlayAnimationTool."""

    def test_tool_creation(self, mock_node):
        """Тест создания инструмента."""
        tool = PlayAnimationTool(mock_node)

        assert tool.name == "play_animation"
        assert "LED" in tool.description
        # 57e96942d добавил необязательный duration; тест не гонялся в CI
        # и продолжал ждать один параметр.
        params = {p.name: p for p in tool.parameters}
        assert set(params) == {"animation", "duration", "face"}
        # PR-3c (#3691): хватает animation или face, поэтому обязательного параметра нет
        assert not any(p.required for p in params.values())

    def test_available_animations(self, mock_node):
        """Тест списка доступных анимаций."""
        tool = PlayAnimationTool(mock_node)

        # Проверяем что есть основные анимации
        animations = tool.AVAILABLE_ANIMATIONS
        assert "idle" in animations
        assert "happy" in animations
        assert "sad" in animations
        assert "thinking" in animations

    def test_execute_valid_animation(self, mock_node):
        """Тест выполнения с валидной анимацией."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy")

        assert result.success is True
        assert "happy" in result.message.lower()

        # ADR-0150 PR-3b (#3690): тул лицо не трогает - сцену показывает диалог по ExpressionRequest
        assert mock_node.get_publisher("/voice/animation/request") is None

    def test_execute_invalid_animation(self, mock_node):
        """Тест выполнения с невалидной анимацией."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="nonexistent_animation")

        assert result.success is False
        assert "неизвестная анимация" in result.error.lower()

    @pytest.mark.parametrize("animation", ["happy", "sad", "angry", "surprised", "thinking"])
    def test_execute_multiple_animations(self, mock_node, animation):
        """Параметризованный тест для разных анимаций."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation=animation)

        assert result.success is True
        assert result.data["animation"] == animation

    def test_execute_with_valid_duration(self, mock_node):
        """Тест выполнения с валидной длительностью."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy", duration=5.0)

        assert result.success is True
        assert result.data["animation"] == "happy"
        assert result.data["duration"] == 5.0

        assert mock_node.get_publisher("/voice/animation/request") is None

    def test_execute_with_invalid_duration_too_short(self, mock_node):
        """Тест выполнения с слишком короткой длительностью - должна быть установлена минимальная."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy", duration=1.0)

        # Теперь должно быть успешно с минимальной длительностью
        assert result.success is True
        assert result.data["animation"] == "happy"
        assert result.data["duration"] == kn.FACE_SHOW_MIN_S

        # правило показа одно с диалогом (expression.show_seconds, #4089)
        assert mock_node.get_publisher("/voice/animation/request") is None

    def test_execute_with_invalid_duration_too_long(self, mock_node):
        """Тест выполнения с слишком длинной длительностью - должна быть установлена минимальная."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy", duration=35.0)

        # Теперь должно быть успешно с минимальной длительностью
        assert result.success is True
        assert result.data["animation"] == "happy"
        assert result.data["duration"] == kn.FACE_SCENE_MAX_S

        # правило показа одно с диалогом (expression.show_seconds, #4089)
        assert mock_node.get_publisher("/voice/animation/request") is None

    @pytest.mark.parametrize("duration", [5.0, 10.0, 20.0, 30.0])
    def test_execute_with_edge_case_durations(self, mock_node, duration):
        """Параметризованный тест граничных значений длительности."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy", duration=duration)

        assert result.success is True
        assert result.data["duration"] == duration

    @pytest.mark.parametrize("asked, shown", [
        (60.0, 30.0), (35.0, 30.0), (1.0, 5.0), (1.5, 5.0), (0.0, 8.0), (-5.0, 8.0), (12.5, 12.5)])
    def test_execute_with_out_of_range_durations_follow_the_show_rule(self, mock_node, asked, shown):
        """Вне ``[FACE_SHOW_MIN_S, FACE_SCENE_MAX_S]`` - к ближайшему краю, не в минимум; ноль и минус - умолчание."""
        result = PlayAnimationTool(mock_node).execute(animation="happy", duration=asked)

        assert result.success is True
        assert result.data["duration"] == shown

    def test_execute_without_duration_leaves_it_unset(self, mock_node):
        """Время не задано - в данных его нет: умолчание ставит правило показа у диалога, тул чисел не выдумывает."""
        result = PlayAnimationTool(mock_node).execute(animation="happy")

        assert result.success is True and "duration" not in result.data

    def test_duration_description_comes_from_the_knowledge(self, mock_node):
        by_name = {p.name: p for p in PlayAnimationTool(mock_node).parameters}

        assert by_name["duration"].description == kn.FACE_SHOW_DURATION_HINT
        assert all(f"{n:g}" in kn.FACE_SHOW_DURATION_HINT
                   for n in (kn.FACE_SHOW_MIN_S, kn.FACE_SCENE_MAX_S, kn.FACE_SHOW_DEFAULT_S))


@pytest.mark.parametrize("asked, seconds", [(60, 30.0), (1, 5.0), (None, 8.0), (12, 12.0), (0, 8.0)])
def test_tool_and_dialogue_hold_the_scene_by_one_rule(mock_node, asked, seconds):
    """#4089: тул и диалог не расходятся во времени показа: 60 -> 30, 1 -> 5, без времени -> 8 (умолчание показа)."""
    from rob_box_dialog.expression import show_seconds
    from rob_box_dialog.turn import shown_face

    data = PlayAnimationTool(mock_node).execute(animation="happy", duration=asked).data

    assert show_seconds(asked) == seconds
    assert data.get("duration", seconds) == seconds, "тул отдаёт то же число, что правило"
    assert shown_face(None, data).duration_s == seconds, "диалог держит сцену то же время"
    assert shown_face(None, {"animation": "happy", "duration": asked}).duration_s == seconds, "и по сырым данным"
