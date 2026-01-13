"""
test_animation.py - Unit тесты для инструментов анимации

Тестирует:
- PlayAnimationTool
- SetEmotionTool
"""

import pytest

from rob_box_mcp_tools.tools.animation import PlayAnimationTool, SetEmotionTool


@pytest.mark.unit
class TestPlayAnimationTool:
    """Тесты для PlayAnimationTool"""

    def test_tool_creation(self, mock_node):
        """Тест создания инструмента"""
        tool = PlayAnimationTool(mock_node)

        assert tool.name == "play_animation"
        assert "LED" in tool.description
        assert len(tool.parameters) == 1

    def test_available_animations(self, mock_node):
        """Тест списка доступных анимаций"""
        tool = PlayAnimationTool(mock_node)

        # Проверяем что есть основные анимации
        animations = tool.AVAILABLE_ANIMATIONS
        assert "idle" in animations
        assert "happy" in animations
        assert "sad" in animations
        assert "thinking" in animations

    def test_execute_valid_animation(self, mock_node):
        """Тест выполнения с валидной анимацией"""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="happy")

        assert result.success is True
        assert "happy" in result.message.lower()

        # Проверяем что сообщение опубликовано
        pub = mock_node.get_publisher("/voice/animation/request")
        assert pub is not None
        assert len(pub.published_messages) == 1
        assert pub.published_messages[0].data == "happy"

    def test_execute_invalid_animation(self, mock_node):
        """Тест выполнения с невалидной анимацией"""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation="nonexistent_animation")

        assert result.success is False
        assert "неизвестная анимация" in result.error.lower()

    @pytest.mark.parametrize("animation", ["happy", "sad", "angry", "surprised", "thinking"])
    def test_execute_multiple_animations(self, mock_node, animation):
        """Параметризованный тест для разных анимаций"""
        tool = PlayAnimationTool(mock_node)

        result = tool.execute(animation=animation)

        assert result.success is True
        assert result.data["animation"] == animation


@pytest.mark.unit
class TestSetEmotionTool:
    """Тесты для SetEmotionTool"""

    def test_tool_creation(self, mock_node):
        """Тест создания инструмента"""
        tool = SetEmotionTool(mock_node)

        assert tool.name == "set_emotion"
        assert "эмоцию" in tool.description.lower()
        assert len(tool.parameters) == 1

    def test_emotion_mapping(self, mock_node):
        """Тест маппинга эмоций на анимации"""
        tool = SetEmotionTool(mock_node)

        mapping = tool.EMOTION_TO_ANIMATION
        assert mapping["радость"] == "happy"
        assert mapping["грусть"] == "sad"
        assert mapping["думаю"] == "thinking"

    def test_execute_valid_emotion(self, mock_node):
        """Тест выполнения с валидной эмоцией"""
        tool = SetEmotionTool(mock_node)

        result = tool.execute(emotion="радость")

        assert result.success is True
        assert result.data["emotion"] == "радость"
        assert result.data["animation"] == "happy"

        # Проверяем публикацию
        pub = mock_node.get_publisher("/voice/animation/request")
        assert len(pub.published_messages) == 1
        assert pub.published_messages[0].data == "happy"

    def test_execute_invalid_emotion(self, mock_node):
        """Тест выполнения с невалидной эмоцией"""
        tool = SetEmotionTool(mock_node)

        result = tool.execute(emotion="неизвестная_эмоция")

        assert result.success is False
        assert "неизвестная эмоция" in result.error.lower()

    @pytest.mark.parametrize(
        "emotion,expected_animation",
        [
            ("радость", "happy"),
            ("грусть", "sad"),
            ("злость", "angry"),
            ("удивление", "surprised"),
            ("думаю", "thinking"),
            ("победа", "victory"),
        ],
    )
    def test_emotion_to_animation_mapping(self, mock_node, emotion, expected_animation):
        """Параметризованный тест маппинга эмоций"""
        tool = SetEmotionTool(mock_node)

        result = tool.execute(emotion=emotion)

        assert result.success is True
        assert result.data["animation"] == expected_animation
