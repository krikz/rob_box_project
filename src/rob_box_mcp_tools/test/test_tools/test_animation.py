"""
test_animation.py - Unit тесты для инструментов анимации

Тестирует:
- PlayAnimationTool
"""

import pytest
from unittest.mock import Mock, patch

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
        assert set(params) == {"animation", "duration"}
        assert params["animation"].required is True
        assert params["duration"].required is False

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
        assert result.data["duration"] == 2.0

        # то же правило применяет диалог к ttl запроса лица (animations.clamp_show_seconds)
        assert mock_node.get_publisher("/voice/animation/request") is None

    def test_execute_with_invalid_duration_too_long(self, mock_node):
        """Тест выполнения с слишком длинной длительностью - должна быть установлена минимальная."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy", duration=35.0)

        # Теперь должно быть успешно с минимальной длительностью
        assert result.success is True
        assert result.data["animation"] == "happy"
        assert result.data["duration"] == 2.0

        # то же правило применяет диалог к ttl запроса лица (animations.clamp_show_seconds)
        assert mock_node.get_publisher("/voice/animation/request") is None

    @pytest.mark.parametrize("duration", [2.0, 5.0, 10.0, 20.0, 30.0])
    def test_execute_with_edge_case_durations(self, mock_node, duration):
        """Параметризованный тест граничных значений длительности."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy", duration=duration)

        assert result.success is True
        assert result.data["duration"] == duration

    @pytest.mark.parametrize("invalid_duration", [0.0, -5.0, 1.5, 31.0, 100.0])
    def test_execute_with_out_of_range_durations_clamped_to_min(self, mock_node, invalid_duration):
        """Параметризованный тест - все значения вне диапазона устанавливаются в минимум."""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy", duration=invalid_duration)

        # Должно быть успешно с минимальной длительностью
        assert result.success is True
        assert result.data["animation"] == "happy"
        assert result.data["duration"] == 2.0


def test_show_seconds_rule_is_one_for_the_tool_and_the_dialogue():
    from rob_box_mcp_tools.animations import SHOW_MAX_S, SHOW_MIN_S, clamp_show_seconds

    assert clamp_show_seconds(None) is None
    assert clamp_show_seconds(5.0) == 5.0
    assert clamp_show_seconds(SHOW_MIN_S) == SHOW_MIN_S and clamp_show_seconds(SHOW_MAX_S) == SHOW_MAX_S
    assert clamp_show_seconds(SHOW_MIN_S - 0.1) == SHOW_MIN_S and clamp_show_seconds(SHOW_MAX_S + 0.1) == SHOW_MIN_S
