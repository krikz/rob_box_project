"""
Тесты для VoiceCommandHandler.
"""

import pytest
from rob_box_voice.core.voice_command_handler import (
    VoiceCommandHandler,
    VoiceIntent,
    VoiceParameter,
    VoiceCommandResult,
)


class TestVoiceCommandHandler:
    """Тесты VoiceCommandHandler."""

    @pytest.fixture
    def handler(self):
        """Создать handler."""
        return VoiceCommandHandler()

    # ============= Тесты распознавания Volume команд =============

    def test_detect_volume_louder(self, handler):
        """Распознавание команд увеличения громкости."""
        result = handler.detect_voice_intent("Говори громче")
        assert result is not None
        assert result[0] == VoiceParameter.VOLUME
        assert result[1] == VoiceIntent.LOUDER

    def test_detect_volume_quieter(self, handler):
        """Распознавание команд уменьшения громкости."""
        result = handler.detect_voice_intent("сделай тише")
        assert result is not None
        assert result[0] == VoiceParameter.VOLUME
        assert result[1] == VoiceIntent.QUIETER

    def test_detect_volume_max(self, handler):
        """Распознавание команды максимальной громкости."""
        result = handler.detect_voice_intent("говори громко")
        assert result is not None
        assert result[0] == VoiceParameter.VOLUME
        assert result[1] == VoiceIntent.VOLUME_MAX

    def test_detect_volume_normal(self, handler):
        """Распознавание команды нормальной громкости."""
        result = handler.detect_voice_intent("нормальная громкость")
        assert result is not None
        assert result[0] == VoiceParameter.VOLUME
        assert result[1] == VoiceIntent.VOLUME_NORMAL

    # ============= Тесты распознавания Pitch команд =============

    def test_detect_pitch_higher(self, handler):
        """Распознавание команд повышения голоса."""
        result = handler.detect_voice_intent("говори выше")
        assert result is not None
        assert result[0] == VoiceParameter.PITCH
        assert result[1] == VoiceIntent.PITCH_HIGHER

    def test_detect_pitch_lower(self, handler):
        """Распознавание команд понижения голоса."""
        result = handler.detect_voice_intent("голос ниже")
        assert result is not None
        assert result[0] == VoiceParameter.PITCH
        assert result[1] == VoiceIntent.PITCH_LOWER

    def test_detect_pitch_normal(self, handler):
        """Распознавание команды нормального голоса."""
        result = handler.detect_voice_intent("нормальный голос")
        assert result is not None
        assert result[0] == VoiceParameter.PITCH
        assert result[1] == VoiceIntent.PITCH_NORMAL

    # ============= Тесты распознавания Speed команд =============

    def test_detect_speed_faster(self, handler):
        """Распознавание команд ускорения."""
        result = handler.detect_voice_intent("говори быстрее")
        assert result is not None
        assert result[0] == VoiceParameter.SPEED
        assert result[1] == VoiceIntent.SPEED_FASTER

    def test_detect_speed_slower(self, handler):
        """Распознавание команд замедления."""
        result = handler.detect_voice_intent("медленнее")
        assert result is not None
        assert result[0] == VoiceParameter.SPEED
        assert result[1] == VoiceIntent.SPEED_SLOWER

    def test_detect_speed_normal(self, handler):
        """Распознавание команды нормальной скорости."""
        result = handler.detect_voice_intent("нормальная скорость")
        assert result is not None
        assert result[0] == VoiceParameter.SPEED
        assert result[1] == VoiceIntent.SPEED_NORMAL

    def test_detect_no_command(self, handler):
        """Нераспознанная команда."""
        result = handler.detect_voice_intent("расскажи про погоду")
        assert result is None

    # ============= Тесты вычисления Volume =============

    def test_calculate_volume_louder(self, handler):
        """Вычисление увеличения громкости."""
        new_val, text, at_limit = handler.calculate_new_volume(-3.0, VoiceIntent.LOUDER)
        assert new_val == 0.0  # -3 + 3
        assert "громче" in text.lower()
        assert not at_limit

    def test_calculate_volume_louder_at_max(self, handler):
        """Увеличение громкости на максимуме."""
        new_val, text, at_limit = handler.calculate_new_volume(6.0, VoiceIntent.LOUDER)
        assert new_val == 6.0
        assert "максимальная" in text.lower()
        assert at_limit

    def test_calculate_volume_quieter(self, handler):
        """Вычисление уменьшения громкости."""
        new_val, text, at_limit = handler.calculate_new_volume(-3.0, VoiceIntent.QUIETER)
        assert new_val == -6.0  # -3 - 3
        assert "тише" in text.lower()
        assert not at_limit

    def test_calculate_volume_quieter_at_min(self, handler):
        """Уменьшение громкости на минимуме."""
        new_val, text, at_limit = handler.calculate_new_volume(-20.0, VoiceIntent.QUIETER)
        assert new_val == -20.0
        assert "минимальная" in text.lower()
        assert at_limit

    def test_calculate_volume_max(self, handler):
        """Установка максимальной громкости."""
        new_val, text, at_limit = handler.calculate_new_volume(-3.0, VoiceIntent.VOLUME_MAX)
        assert new_val == 6.0
        assert "максимальная" in text.lower()
        assert not at_limit

    def test_calculate_volume_normal(self, handler):
        """Установка нормальной громкости."""
        new_val, text, at_limit = handler.calculate_new_volume(0.0, VoiceIntent.VOLUME_NORMAL)
        assert new_val == -3.0
        assert "нормальная" in text.lower()
        assert not at_limit

    # ============= Тесты вычисления Pitch =============

    def test_calculate_pitch_higher(self, handler):
        """Вычисление повышения pitch."""
        new_val, text, at_limit = handler.calculate_new_pitch(1.0, VoiceIntent.PITCH_HIGHER)
        assert new_val == pytest.approx(1.2)
        assert "выше" in text.lower()
        assert not at_limit

    def test_calculate_pitch_higher_at_max(self, handler):
        """Повышение pitch на максимуме."""
        new_val, text, at_limit = handler.calculate_new_pitch(2.0, VoiceIntent.PITCH_HIGHER)
        assert new_val == 2.0
        assert "максимально высокий" in text.lower()
        assert at_limit

    def test_calculate_pitch_lower(self, handler):
        """Вычисление понижения pitch."""
        new_val, text, at_limit = handler.calculate_new_pitch(1.0, VoiceIntent.PITCH_LOWER)
        assert new_val == pytest.approx(0.8)
        assert "ниже" in text.lower()
        assert not at_limit

    def test_calculate_pitch_normal(self, handler):
        """Установка нормального pitch."""
        new_val, text, at_limit = handler.calculate_new_pitch(1.5, VoiceIntent.PITCH_NORMAL)
        assert new_val == 1.0
        assert "нормальный" in text.lower()
        assert not at_limit

    # ============= Тесты вычисления Speed =============

    def test_calculate_speed_faster(self, handler):
        """Вычисление ускорения."""
        new_val, text, at_limit = handler.calculate_new_speed(1.0, VoiceIntent.SPEED_FASTER)
        assert new_val == pytest.approx(1.2)
        assert "быстрее" in text.lower()
        assert not at_limit

    def test_calculate_speed_faster_at_max(self, handler):
        """Ускорение на максимуме."""
        new_val, text, at_limit = handler.calculate_new_speed(2.0, VoiceIntent.SPEED_FASTER)
        assert new_val == 2.0
        assert "максимальная" in text.lower()
        assert at_limit

    def test_calculate_speed_slower(self, handler):
        """Вычисление замедления."""
        new_val, text, at_limit = handler.calculate_new_speed(1.0, VoiceIntent.SPEED_SLOWER)
        assert new_val == pytest.approx(0.8)
        assert "медленнее" in text.lower()
        assert not at_limit

    def test_calculate_speed_normal(self, handler):
        """Установка нормальной скорости."""
        new_val, text, at_limit = handler.calculate_new_speed(1.5, VoiceIntent.SPEED_NORMAL)
        assert new_val == 1.0
        assert "нормальная" in text.lower()
        assert not at_limit

    # ============= Тесты process_voice_command =============

    def test_process_volume_command(self, handler):
        """Обработка команды громкости."""
        result = handler.process_voice_command("говори громче", -3.0, 1.0, 1.0)
        assert result is not None
        assert result.parameter == VoiceParameter.VOLUME
        assert result.old_value == -3.0
        assert result.new_value == 0.0
        assert "громче" in result.response_text.lower()
        assert not result.at_limit

    def test_process_pitch_command(self, handler):
        """Обработка команды pitch."""
        result = handler.process_voice_command("говори выше", -3.0, 1.0, 1.0)
        assert result is not None
        assert result.parameter == VoiceParameter.PITCH
        assert result.old_value == 1.0
        assert result.new_value == pytest.approx(1.2)
        assert "выше" in result.response_text.lower()
        assert not result.at_limit

    def test_process_speed_command(self, handler):
        """Обработка команды скорости."""
        result = handler.process_voice_command("говори быстрее", -3.0, 1.0, 1.0)
        assert result is not None
        assert result.parameter == VoiceParameter.SPEED
        assert result.old_value == 1.0
        assert result.new_value == pytest.approx(1.2)
        assert "быстрее" in result.response_text.lower()
        assert not result.at_limit

    def test_process_no_command(self, handler):
        """Обработка нераспознанной команды."""
        result = handler.process_voice_command("расскажи про погоду", -3.0, 1.0, 1.0)
        assert result is None
