"""
Обработчик команд управления голосом (громкость, высота, скорость).

Извлечено из dialogue_node.py для тестируемой логики управления параметрами TTS.
"""

import re
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class VoiceParameter(Enum):
    """Параметры голоса."""

    VOLUME = "volume"
    PITCH = "pitch"
    SPEED = "speed"


class VoiceIntent(Enum):
    """Намерения управления голосом."""

    # Volume
    LOUDER = "louder"
    QUIETER = "quieter"
    VOLUME_MAX = "volume_max"
    VOLUME_NORMAL = "volume_normal"

    # Pitch
    PITCH_HIGHER = "pitch_higher"
    PITCH_LOWER = "pitch_lower"
    PITCH_NORMAL = "pitch_normal"

    # Speed
    SPEED_FASTER = "speed_faster"
    SPEED_SLOWER = "speed_slower"
    SPEED_NORMAL = "speed_normal"


@dataclass
class VoiceCommandResult:
    """Результат обработки команды голоса."""

    parameter: VoiceParameter
    old_value: float
    new_value: float
    response_text: str
    at_limit: bool = False


class VoiceCommandHandler:
    """
    Обработчик команд управления голосом.

    Распознаёт команды изменения громкости, высоты и скорости голоса,
    вычисляет новые значения параметров.

    Чистый Python модуль без зависимостей от ROS.
    """

    # Паттерны для распознавания команд
    VOLUME_PATTERNS = {
        # Специфичные паттерны идут ПЕРВЫМИ — иначе r"громко" из LOUDER
        # ложно срабатывает на "говори громко" и подстроку в "громкость".
        VoiceIntent.VOLUME_MAX: [
            r"говори громко",
            r"максимальн\w* громкост",
            r"на полную громкост",
        ],
        VoiceIntent.VOLUME_NORMAL: [
            r"нормальн\w* громкост",
            r"стандартн\w* громкост",
            r"обычн\w* громкост",
        ],
        VoiceIntent.LOUDER: [
            r"громче",
            r"погромч\w+",
            r"прибав\w* громкост",
            r"увелич\w* громкост",
        ],
        VoiceIntent.QUIETER: [
            r"тише",
            r"потише",
            r"убав\w* громкост",
            r"уменьш\w* громкост",
        ],
    }

    PITCH_PATTERNS = {
        VoiceIntent.PITCH_HIGHER: [
            r"говори выше",
            r"голос выше",
            r"повыс\w* голос",
            r"выше говор",
        ],
        VoiceIntent.PITCH_LOWER: [
            r"говори ниже",
            r"голос ниже",
            r"пониж\w* голос",
            r"ниже говор",
        ],
        VoiceIntent.PITCH_NORMAL: [
            r"нормальн\w* голос",
            r"обычн\w* голос",
            r"говори нормально",
        ],
    }

    SPEED_PATTERNS = {
        VoiceIntent.SPEED_FASTER: [
            r"говори быстрее",
            r"быстрее",
            r"ускор\w*",
            r"побыстрее",
        ],
        VoiceIntent.SPEED_SLOWER: [
            r"говори медленнее",
            r"медленнее",
            r"замедл\w*",
            r"помедленнее",
        ],
        VoiceIntent.SPEED_NORMAL: [
            r"нормальн\w* скорост",
            r"обычн\w* скорост",
        ],
    }

    # Диапазоны параметров
    VOLUME_MIN = -20.0  # dB
    VOLUME_MAX = 6.0  # dB
    VOLUME_NORMAL = -3.0  # dB
    VOLUME_STEP = 3.0  # dB

    PITCH_MIN = 0.5  # множитель
    PITCH_MAX = 2.0  # множитель
    PITCH_NORMAL = 1.0  # множитель
    PITCH_STEP = 0.2  # множитель

    SPEED_MIN = 0.5  # множитель
    SPEED_MAX = 2.0  # множитель
    SPEED_NORMAL = 1.0  # множитель
    SPEED_STEP = 0.2  # множитель

    def detect_voice_intent(self, text: str) -> Optional[Tuple[VoiceParameter, VoiceIntent]]:
        """
        Определить намерение команды управления голосом.

        Args:
            text: Текст команды пользователя

        Returns:
            Tuple[VoiceParameter, VoiceIntent] или None если команда не распознана
        """
        text_lower = text.lower()

        # Проверяем volume команды
        for intent, patterns in self.VOLUME_PATTERNS.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return (VoiceParameter.VOLUME, intent)

        # Проверяем pitch команды
        for intent, patterns in self.PITCH_PATTERNS.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return (VoiceParameter.PITCH, intent)

        # Проверяем speed команды
        for intent, patterns in self.SPEED_PATTERNS.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    return (VoiceParameter.SPEED, intent)

        return None

    def calculate_new_volume(
        self, current_volume: float, intent: VoiceIntent
    ) -> Tuple[float, str, bool]:
        """
        Вычислить новое значение громкости.

        Args:
            current_volume: Текущая громкость в dB
            intent: Намерение изменения громкости

        Returns:
            Tuple[новое_значение, текст_ответа, достигнут_лимит]
        """
        new_volume = current_volume
        response_text = ""
        at_limit = False

        if intent == VoiceIntent.LOUDER:
            new_volume = min(current_volume + self.VOLUME_STEP, self.VOLUME_MAX)
            response_text = "Делаю громче"
            at_limit = abs(new_volume - current_volume) < 0.1
        elif intent == VoiceIntent.QUIETER:
            new_volume = max(current_volume - self.VOLUME_STEP, self.VOLUME_MIN)
            response_text = "Делаю тише"
            at_limit = abs(new_volume - current_volume) < 0.1
        elif intent == VoiceIntent.VOLUME_MAX:
            new_volume = self.VOLUME_MAX
            response_text = "Максимальная громкость"
        elif intent == VoiceIntent.VOLUME_NORMAL:
            new_volume = self.VOLUME_NORMAL
            response_text = "Нормальная громкость"

        if at_limit:
            if intent == VoiceIntent.LOUDER:
                response_text = "Громкость уже максимальная"
            elif intent == VoiceIntent.QUIETER:
                response_text = "Громкость уже минимальная"

        return (new_volume, response_text, at_limit)

    def calculate_new_pitch(
        self, current_pitch: float, intent: VoiceIntent
    ) -> Tuple[float, str, bool]:
        """
        Вычислить новое значение высоты голоса.

        Args:
            current_pitch: Текущий pitch shift
            intent: Намерение изменения pitch

        Returns:
            Tuple[новое_значение, текст_ответа, достигнут_лимит]
        """
        new_pitch = current_pitch
        response_text = ""
        at_limit = False

        if intent == VoiceIntent.PITCH_HIGHER:
            new_pitch = min(current_pitch + self.PITCH_STEP, self.PITCH_MAX)
            response_text = "Говорю выше"
            at_limit = abs(new_pitch - current_pitch) < 0.01
        elif intent == VoiceIntent.PITCH_LOWER:
            new_pitch = max(current_pitch - self.PITCH_STEP, self.PITCH_MIN)
            response_text = "Говорю ниже"
            at_limit = abs(new_pitch - current_pitch) < 0.01
        elif intent == VoiceIntent.PITCH_NORMAL:
            new_pitch = self.PITCH_NORMAL
            response_text = "Нормальный голос"

        if at_limit:
            if intent == VoiceIntent.PITCH_HIGHER:
                response_text = "Голос уже максимально высокий"
            elif intent == VoiceIntent.PITCH_LOWER:
                response_text = "Голос уже минимально низкий"

        return (new_pitch, response_text, at_limit)

    def calculate_new_speed(
        self, current_speed: float, intent: VoiceIntent
    ) -> Tuple[float, str, bool]:
        """
        Вычислить новое значение скорости речи.

        Args:
            current_speed: Текущая скорость
            intent: Намерение изменения скорости

        Returns:
            Tuple[новое_значение, текст_ответа, достигнут_лимит]
        """
        new_speed = current_speed
        response_text = ""
        at_limit = False

        if intent == VoiceIntent.SPEED_FASTER:
            new_speed = min(current_speed + self.SPEED_STEP, self.SPEED_MAX)
            response_text = "Говорю быстрее"
            at_limit = abs(new_speed - current_speed) < 0.01
        elif intent == VoiceIntent.SPEED_SLOWER:
            new_speed = max(current_speed - self.SPEED_STEP, self.SPEED_MIN)
            response_text = "Говорю медленнее"
            at_limit = abs(new_speed - current_speed) < 0.01
        elif intent == VoiceIntent.SPEED_NORMAL:
            new_speed = self.SPEED_NORMAL
            response_text = "Нормальная скорость"

        if at_limit:
            if intent == VoiceIntent.SPEED_FASTER:
                response_text = "Скорость уже максимальная"
            elif intent == VoiceIntent.SPEED_SLOWER:
                response_text = "Скорость уже минимальная"

        return (new_speed, response_text, at_limit)

    def process_voice_command(
        self, text: str, current_volume: float, current_pitch: float, current_speed: float
    ) -> Optional[VoiceCommandResult]:
        """
        Обработать команду управления голосом.

        Args:
            text: Текст команды
            current_volume: Текущая громкость (dB)
            current_pitch: Текущий pitch shift
            current_speed: Текущая скорость

        Returns:
            VoiceCommandResult или None если команда не распознана
        """
        detection = self.detect_voice_intent(text)
        if not detection:
            return None

        parameter, intent = detection

        if parameter == VoiceParameter.VOLUME:
            new_value, response_text, at_limit = self.calculate_new_volume(current_volume, intent)
            return VoiceCommandResult(
                parameter=parameter,
                old_value=current_volume,
                new_value=new_value,
                response_text=response_text,
                at_limit=at_limit,
            )
        elif parameter == VoiceParameter.PITCH:
            new_value, response_text, at_limit = self.calculate_new_pitch(current_pitch, intent)
            return VoiceCommandResult(
                parameter=parameter,
                old_value=current_pitch,
                new_value=new_value,
                response_text=response_text,
                at_limit=at_limit,
            )
        elif parameter == VoiceParameter.SPEED:
            new_value, response_text, at_limit = self.calculate_new_speed(current_speed, intent)
            return VoiceCommandResult(
                parameter=parameter,
                old_value=current_speed,
                new_value=new_value,
                response_text=response_text,
                at_limit=at_limit,
            )

        return None
