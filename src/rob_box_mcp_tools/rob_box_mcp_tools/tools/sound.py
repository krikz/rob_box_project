#!/usr/bin/env python3
"""
sound.py - Инструменты для управления звуковыми эффектами

Инструменты:
- PlaySoundTool: Воспроизвести звуковой эффект
- GetSoundInfoTool: Получить информацию о доступных звуках
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
- Обработка/размышление: robot_thinking, robot_loop, robot_work_1
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


class GetSoundInfoTool(MCPTool):
    """Инструмент для получения информации о доступных звуковых эффектах"""
    
    def __init__(self, node):
        super().__init__(node)
        self._sound_catalog = None
        self._catalog_path = None
    
    def _load_catalog(self):
        """Загрузить каталог звуков из sound_catalog.json"""
        if self._sound_catalog is not None:
            return self._sound_catalog
        
        import json
        from pathlib import Path
        
        # Попробуем найти sound_catalog.json
        possible_paths = [
            Path("/home/ros2/rob_box_project/sound_pack/sound_catalog.json"),
            Path("sound_pack/sound_catalog.json"),
            Path("../sound_pack/sound_catalog.json"),
            Path("../../sound_pack/sound_catalog.json"),
        ]
        
        for path in possible_paths:
            if path.exists():
                self._catalog_path = path
                with open(path, 'r', encoding='utf-8') as f:
                    self._sound_catalog = json.load(f)
                self.log_info(f"Загружен каталог звуков из {path}")
                return self._sound_catalog
        
        # Если не нашли файл, создадим минимальный каталог из AVAILABLE_SOUNDS
        self.log_warning("sound_catalog.json не найден, используем минимальный каталог")
        self._sound_catalog = {
            "sounds": {
                f"{sound}.mp3": {
                    "trigger": sound,
                    "category": "unknown",
                    "description": f"Sound: {sound}",
                    "description_ru": f"Звук: {sound}"
                }
                for sound in PlaySoundTool.AVAILABLE_SOUNDS
            }
        }
        return self._sound_catalog
    
    @property
    def name(self) -> str:
        return "get_sound_info"
    
    @property
    def description(self) -> str:
        return (
            "Получить информацию о доступных звуковых эффектах. "
            "Используй этот инструмент чтобы узнать какие звуки доступны, "
            "их описание, длительность, категорию и рекомендуемое использование. "
            "Можно запросить информацию о конкретном звуке или получить список всех звуков определенной категории."
        )
    
    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="sound_name",
                type="string",
                description="Название конкретного звука для получения подробной информации (опционально). Если не указано, вернется список всех звуков.",
                required=False,
            ),
            MCPToolParameter(
                name="category",
                type="string",
                description="Фильтр по категории: 'base', 'ui', 'robot' (опционально)",
                required=False,
                enum=["base", "ui", "robot"],
            ),
        ]
    
    @property
    def execution_type(self) -> ToolExecutionType:
        """Получение информации - быстрая операция"""
        return ToolExecutionType.FAST
    
    def execute(self, sound_name: str = None, category: str = None) -> MCPToolResult:
        """Получить информацию о звуках"""
        catalog = self._load_catalog()
        
        if not catalog or "sounds" not in catalog:
            return MCPToolResult(
                success=False,
                error="Не удалось загрузить каталог звуков",
                message="Каталог звуков недоступен"
            )
        
        sounds = catalog["sounds"]
        
        # Если запрошен конкретный звук
        if sound_name:
            # Найти звук по trigger или по имени файла
            sound_info = None
            for filename, info in sounds.items():
                if info.get("trigger") == sound_name or filename.replace(".mp3", "") == sound_name:
                    sound_info = info.copy()
                    sound_info["filename"] = filename
                    break
            
            if sound_info:
                # Форматируем информацию для LLM
                result_msg = f"Звук '{sound_name}':\n"
                result_msg += f"  • Описание: {sound_info.get('description_ru', sound_info.get('description', 'N/A'))}\n"
                result_msg += f"  • Категория: {sound_info.get('category', 'unknown')}\n"
                if 'duration' in sound_info:
                    result_msg += f"  • Длительность: {sound_info['duration']:.2f}с\n"
                if 'usage' in sound_info:
                    result_msg += f"  • Использование: {sound_info['usage']}\n"
                if 'author' in sound_info:
                    result_msg += f"  • Автор: {sound_info['author']}\n"
                
                return MCPToolResult(
                    success=True,
                    data=sound_info,
                    message=result_msg
                )
            else:
                return MCPToolResult(
                    success=False,
                    error=f"Звук '{sound_name}' не найден в каталоге",
                    message=f"Звук '{sound_name}' не найден. Используй get_sound_info() без параметров для списка всех звуков."
                )
        
        # Фильтр по категории
        if category:
            filtered_sounds = {
                info.get("trigger", filename.replace(".mp3", "")): {
                    "description": info.get("description_ru", info.get("description", "")),
                    "duration": info.get("duration"),
                    "usage": info.get("usage", "")
                }
                for filename, info in sounds.items()
                if info.get("category") == category
            }
            
            if not filtered_sounds:
                return MCPToolResult(
                    success=True,
                    data={"sounds": [], "category": category},
                    message=f"Звуки категории '{category}' не найдены"
                )
            
            # Форматируем список
            result_msg = f"Звуки категории '{category}' ({len(filtered_sounds)}):\n"
            for trigger, info in list(filtered_sounds.items())[:20]:  # Ограничим вывод
                desc = info['description'][:60] + "..." if len(info['description']) > 60 else info['description']
                result_msg += f"  • {trigger}: {desc}\n"
            
            if len(filtered_sounds) > 20:
                result_msg += f"  ... и ещё {len(filtered_sounds) - 20} звуков\n"
            
            return MCPToolResult(
                success=True,
                data={"sounds": list(filtered_sounds.keys()), "category": category, "details": filtered_sounds},
                message=result_msg
            )
        
        # Общий список всех звуков
        all_triggers = []
        categories_count = {"base": 0, "ui": 0, "robot": 0, "other": 0}
        
        for filename, info in sounds.items():
            trigger = info.get("trigger", filename.replace(".mp3", ""))
            all_triggers.append(trigger)
            cat = info.get("category", "other")
            categories_count[cat] = categories_count.get(cat, 0) + 1
        
        result_msg = f"Всего доступно {len(all_triggers)} звуков:\n"
        result_msg += f"  • BASE (эмоции): {categories_count.get('base', 0)} звуков\n"
        result_msg += f"  • UI (интерфейс): {categories_count.get('ui', 0)} звуков\n"
        result_msg += f"  • ROBOT (эффекты): {categories_count.get('robot', 0)} звуков\n"
        result_msg += "\nИспользуй get_sound_info(category='base') для списка звуков категории\n"
        result_msg += "Или get_sound_info(sound_name='robot_happy') для подробностей о конкретном звуке"
        
        return MCPToolResult(
            success=True,
            data={
                "total_sounds": len(all_triggers),
                "categories": categories_count,
                "all_triggers": all_triggers[:30]  # Первые 30 для примера
            },
            message=result_msg
        )

