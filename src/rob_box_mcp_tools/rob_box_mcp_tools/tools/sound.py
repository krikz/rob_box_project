#!/usr/bin/env python3
"""
sound.py - Инструменты для управления звуковыми эффектами

Инструменты:
- PlaySoundTool: Воспроизвести звуковой эффект
"""

from typing import List, TYPE_CHECKING

# Ленивый импорт ROS 2 модулей для поддержки unit тестов
if TYPE_CHECKING:
    from std_msgs.msg import String

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


class PlaySoundTool(MCPTool):
    """Инструмент для воспроизведения звуковых эффектов"""

    # Доступные звуковые эффекты (соответствуют триггерам из sound_catalog.json)
    # Организованы по категориям для удобства LLM
    AVAILABLE_SOUNDS = [
        # BASE emotional/reaction sounds (21)
        "robot_affirm",          # Утвердительный ответ
        "robot_angry",           # Злость, недовольство
        "robot_concerned",       # Беспокойство, тревога
        "robot_confirm",         # Подтверждение действия
        "robot_confused",        # Непонимание, вопрос
        "robot_confused_alt",    # Сильное непонимание
        "robot_cute",            # Милая реакция
        "robot_drip_a1",         # Роботический дрип A1
        "robot_drip_d4",         # Роботический дрип D4
        "robot_drip_d5",         # Электронный дрип D5
        "robot_drip_e4",         # Электронный дрип E4
        "robot_error",           # Ошибка, проблема
        "robot_happy",           # Радость, успех
        "robot_sigh",            # Вздох, усталость
        "robot_surprise",        # Удивление, wow
        "robot_talk_1",          # Имитация речи #1
        "robot_talk_2",          # Имитация речи #2
        "robot_talk_3",          # Имитация речи #3
        "robot_talk_4",          # Имитация речи #4
        "robot_thinking",        # Думание, обработка
        "robot_very_cute",       # Очень милая реакция
        
        # UI interaction sounds (12)
        "ui_activate",           # Активация функции
        "ui_bell",               # Звонок начала/конца задачи
        "ui_button",             # Клик по кнопке
        "ui_chime",              # Подтверждение завершения
        "ui_confirm",            # Милое подтверждение
        "ui_dot",                # Минимальный клик
        "ui_menu_click",         # Клик меню
        "ui_note_e",             # Музыкальное подтверждение
        "ui_notification",       # Уведомление, alert
        "ui_radio_start",        # Включение рации
        "ui_random",             # Недоступное действие
        "ui_roger",              # Подтверждение голосовой команды
        
        # ROBOT special effects (18)
        "robot_alert",           # Критическая ошибка, опасность
        "robot_bubbles",         # Бульканье, жидкость
        "robot_fantasy",         # Фантастический UI
        "robot_flyby",           # Движение, пролет мимо
        "robot_glitch",          # Сбой системы, глюк
        "robot_impact",          # Столкновение, удар
        "robot_liquid",          # Жидкостный эффект
        "robot_loop",            # Фоновый loop, обработка
        "robot_power_up",        # Включение, зарядка
        "robot_stinger",         # Драматический акцент
        "robot_stun",            # Оглушение, импульс
        "robot_talk_beep_1",     # Короткий beep речи
        "robot_talk_beep_2",     # Высокий тон beep
        "robot_terminal",        # Вывод текста, лог
        "robot_whoosh",          # Взмах, свуш эффект
        "robot_work_1",          # Обработка данных #1
        "robot_work_2",          # Обработка данных #2
        "robot_work_3",          # Обработка данных #3
        
        # Legacy names for backward compatibility
        "thinking",              # → robot_thinking
        "cute",                  # → robot_cute
        "very_cute",             # → robot_very_cute
        "confused",              # → robot_confused
        "angry_1",               # → robot_angry
        "surprise",              # → robot_surprise
        "talk_1",                # → robot_talk_1
        "talk_2",                # → robot_talk_2
        "talk_3",                # → robot_talk_3
        "talk_4",                # → robot_talk_4
        "error",                 # → robot_error
    ]

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String
        
        # Publisher для триггеров звуков
        self.sound_pub = node.create_publisher(String, "/voice/sound/trigger", 10)

    @property
    def name(self) -> str:
        return "play_sound"

    @property
    def description(self) -> str:
        return """Воспроизвести звуковой эффект. ИСПОЛЬЗУЙ АВТОМАТИЧЕСКИ для звукового сопровождения эмоций во время разговора.

Категории звуков:
- BASE (robot_*): Эмоции и реакции робота - affirm, angry, concerned, confirm, confused, cute, happy, sigh, surprise, thinking, talk_1-4, error, drip_*
- UI (ui_*): Звуки интерфейса - activate, button, bell, chime, confirm, notification, roger, menu_click
- ROBOT (robot_*): Спецэффекты - alert, glitch, impact, power_up, flyby, whoosh, bubbles, stinger, work_1-3

Примеры использования:
- Подтверждение команды: robot_confirm, ui_roger, robot_affirm
- Ошибка: robot_error, robot_glitch, robot_alert
- Обработка: robot_thinking, robot_loop, robot_work_1
- Успех: robot_happy, ui_confirm, ui_chime
- Удивление: robot_surprise, robot_concerned"""

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="sound",
                type="string",
                description="Название звукового эффекта",
                required=True,
                enum=self.AVAILABLE_SOUNDS,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        """Звуки - мгновенные операции (fire-and-forget), sound_node не возвращает результат"""
        return ToolExecutionType.INSTANT

    def execute(self, sound: str) -> MCPToolResult:
        """Воспроизвести звук"""
        self.log_info(f"Воспроизведение звука: {sound}")

        if sound not in self.AVAILABLE_SOUNDS:
            return MCPToolResult(
                success=False, error=f"Неизвестный звук: {sound}", message=f"Доступные: {', '.join(self.AVAILABLE_SOUNDS)}"
            )

        # Публикуем триггер звука
        from std_msgs.msg import String
        msg = String()
        msg.data = sound
        self.sound_pub.publish(msg)

        self.log_info(f"Звук '{sound}' отправлен")

        return MCPToolResult(success=True, data={"sound": sound}, message=f"Воспроизвожу звук: {sound}")
