#!/usr/bin/env python3
"""
sound.py - Инструменты для управления звуковыми эффектами

Инструменты:
- PlaySoundTool: Воспроизвести звуковой эффект; итог - по ``SoundEvent`` владельца звука (``sound_node``)
- GetSoundInfoTool: Получить информацию о доступных звуках
"""

from typing import Any, List, Optional
import json
from pathlib import Path

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from ..owner_event import OwnerEvents, owner_events

TRIGGER_TOPIC = "/voice/sound/trigger"
EVENT_TOPIC = "/voice/sound/event"


class PlaySoundTool(MCPTool):
    """Звуковой эффект: запрос ``{sound, request_id}`` владельцу и фраза по его ``SoundEvent`` (ADR-0150 §5.2).

    Успех - не публикация триггера, а ``started`` владельца; ``skipped{busy|voice_stream|not_found|error}`` - честный
    отказ с причиной; нет события к дедлайну класса - ``timeout``.
    """

    event_source = EVENT_TOPIC

    def __init__(self, node):
        super().__init__(node)
        # Динамический импорт во время выполнения
        from std_msgs.msg import String

        # Publisher для триггеров звуков
        self.sound_pub = node.create_publisher(String, TRIGGER_TOPIC, 10)

        # Загружаем доступные звуки и их длительности из catalog.json
        self._available_sounds, self._duration_map = self._load_sounds_from_catalog()
        self.log_info(f"Загружено {len(self._available_sounds)} звуков из catalog")

    def _load_sounds_from_catalog(self):
        """Загрузить список triggers и map длительностей из sound_catalog.json.

        Returns:
            (triggers: List[str], duration_map: dict[str, float])
        """
        # Попробуем найти sound_catalog.json
        possible_paths = [
            Path("/ws/sound_pack/sound_catalog.json"),  # Docker путь
            Path("/home/ros2/rob_box_project/sound_pack/sound_catalog.json"),  # Pi путь
            Path("sound_pack/sound_catalog.json"),
            Path("../sound_pack/sound_catalog.json"),
            Path("../../sound_pack/sound_catalog.json"),
        ]

        for path in possible_paths:
            if path.exists():
                try:
                    with open(path, 'r', encoding='utf-8') as f:
                        catalog = json.load(f)

                    # Извлечь все triggers и duration из catalog['sounds']
                    triggers = []
                    duration_map = {}
                    sounds = catalog.get('sounds', {})
                    for filename, info in sounds.items():
                        if not filename.endswith('.mp3'):
                            continue
                        if 'trigger' in info:
                            trigger = info['trigger']
                            triggers.append(trigger)
                            if 'duration' in info:
                                duration_map[trigger] = float(info['duration'])

                    self.log_info(f"Загружено {len(triggers)} звуков из {path}")
                    return sorted(triggers), duration_map

                except Exception as e:
                    self.log_error(f"Ошибка загрузки catalog.json из {path}: {e}")

        # Fallback - минимальный список базовых звуков
        self.log_warning("sound_catalog.json не найден, используем fallback список")
        return [
            "robot_affirm", "robot_angry", "robot_confused", "robot_cute",
            "robot_error", "robot_happy", "robot_thinking", "robot_surprise",
            "ui_button", "ui_confirm", "ui_notification",
        ], {}

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
                description="Название звукового эффекта из sound_catalog.json",
                required=True,
                enum=self._available_sounds,
            )
        ]

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: sound-эффекты. personality.
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        """Ждём только ответа ``sound_node`` (``started``/``skipped``, миллисекунды), не конца звука.

        ALSA-конфликт между sound_node и tts_node устранён через dmix
        (/etc/asound.conf монтируется volume → pcm.!default = dmix_respeaker).
        Оба узла пишут в 'default' device, dmix микширует потоки без конкуренции.
        Звук и речь воспроизводятся параллельно — это ожидаемое поведение.
        """
        return ToolExecutionType.FAST

    def execute(self, sound: str) -> MCPToolResult:
        """Попросить ``sound_node`` сыграть звук и дождаться его ответа (звук и речь микширует dmix)."""
        self.log_info(f"Воспроизведение звука: {sound}")

        if sound not in self._available_sounds:
            return MCPToolResult(
                success=False, error=f"Неизвестный звук: {sound}", message=f"Доступные: {', '.join(self._available_sounds[:10])}..."
            )

        from std_msgs.msg import String

        def publish(request_id: str) -> None:
            msg = String()
            msg.data = json.dumps({"sound": sound, "request_id": request_id}, ensure_ascii=False)
            self.sound_pub.publish(msg)

        request_id, event = self._events().ask(publish, sound_deadline_s())
        status, reason = sound_outcome(event)
        self.log_info(f"Звук '{sound}' (request {request_id[:8]}): {status} {reason or ''}".rstrip())
        data = {"sound": sound, "request_id": request_id, "status": status, "reason": reason,
                "duration": self._duration_map.get(sound, 1.5)}
        return MCPToolResult(success=status == "done", data=data, error=None if status == "done" else (reason or status),
                             message=sound_phrase(status, reason, sound))

    def _events(self) -> OwnerEvents:
        from rob_box_dialog_msgs.msg import SoundEvent

        return owner_events(self.node, "sound", lambda: OwnerEvents(
            self.node, SoundEvent, EVENT_TOPIC, key_of=lambda m: m.request_id))


def sound_deadline_s() -> float:
    """Дедлайн события звука - дедлайн класса ``play_sound`` из ``knowledge`` (одна таблица, §5.1)."""
    from rob_box_dialog import knowledge as kn

    return float(kn.ACTION_CLASSES[kn.TOOL_CLASSES["play_sound"]].timeout_s)


def sound_outcome(event: Optional[Any]) -> tuple:
    """``SoundEvent`` → ``(status, reason)``: заиграл (``started``/``finished``) - ``done``, пропуск - ``rejected``."""
    if event is None:
        return "timeout", None
    if event.kind == event.KIND_SKIPPED:
        return "rejected", event.reason or "error"
    return "done", None


def sound_phrase(status: str, reason: Optional[str], sound: str) -> str:
    """Фраза исхода - каталог ``phrases/ru.yaml`` тем же ключом, что строит ``respond`` (код, не LLM)."""
    from rob_box_dialog import phrases
    from rob_box_dialog.respond import ActionResult, phrase_key
    from rob_box_dialog import knowledge as kn

    result = ActionResult(turn_id="", epoch=0, agent="personality", tool="play_sound",
                          cls=kn.TOOL_CLASSES["play_sound"], status=status, reason=reason)
    return phrases.text(phrase_key(result), sound=sound)


class GetSoundInfoTool(MCPTool):
    """Инструмент для получения информации о доступных звуковых эффектах."""

    def __init__(self, node):
        super().__init__(node)
        self._sound_catalog = None
        self._catalog_path = None

    def _load_catalog(self):
        """Загрузить каталог звуков из sound_catalog.json."""
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
        """Получение информации - быстрая операция."""
        return ToolExecutionType.FAST

    def execute(self, sound_name: str = None, category: str = None) -> MCPToolResult:
        """Получить информацию о звуках."""
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
