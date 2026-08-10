"""
Управление памятью событий для контекста восприятия.

Модуль для хранения и управления событиями с типизацией,
автоматической очисткой старых данных и резюмированием.
"""

import time
from typing import Dict, List, Optional


class MemoryManager:
    """Управляет памятью событий с типизацией и временным окном."""

    def __init__(self, memory_window: float = 300.0):
        """
        Инициализировать менеджера памяти.

        Args
        ----
        memory_window:
            Временное окно хранения событий в секундах (default: 300s = 5min)

        """
        self.memory_window = memory_window

        # Общая память событий
        self.recent_events: List[Dict] = []

        # Типизированные очереди событий
        self.speech_events: List[Dict] = []
        self.robot_response_events: List[Dict] = []
        self.robot_thought_events: List[Dict] = []
        self.vision_events: List[Dict] = []
        self.system_events: List[Dict] = []

    def add_event(
        self, event_type: str, content: str, important: bool = False
    ) -> None:
        """
        Добавить событие в память с типизацией.

        Args
        ----
        event_type:
            Тип события (user_speech, robot_response, robot_thought,
            vision, etc.)
        content:
            Содержание события
        important:
            Флаг важности события

        """
        event = {
            'time': time.time(),
            'type': event_type,
            'content': content,
            'important': important
        }

        # Добавляем в общую память
        self.recent_events.append(event)

        # Добавляем в типизированные очереди
        if event_type == 'user_speech':
            self.speech_events.append(event)
        elif event_type == 'robot_response':
            self.robot_response_events.append(event)
        elif event_type == 'robot_thought':
            self.robot_thought_events.append(event)
        elif event_type in ['vision', 'apriltag']:
            self.vision_events.append(event)
        elif event_type in ['error', 'warning', 'battery', 'system']:
            self.system_events.append(event)

        # Автоматическая очистка старых событий
        self._cleanup_old_events()

    def _cleanup_old_events(self) -> None:
        """Удалить события старше memory_window."""
        cutoff = time.time() - self.memory_window

        self.recent_events = [
            e for e in self.recent_events if e['time'] > cutoff
        ]
        self.speech_events = [
            e for e in self.speech_events if e['time'] > cutoff
        ]
        self.robot_response_events = [
            e for e in self.robot_response_events if e['time'] > cutoff
        ]
        self.robot_thought_events = [
            e for e in self.robot_thought_events if e['time'] > cutoff
        ]
        self.vision_events = [
            e for e in self.vision_events if e['time'] > cutoff
        ]
        self.system_events = [
            e for e in self.system_events if e['time'] > cutoff
        ]

    def get_summary(self, count: int = 5) -> str:
        """
        Получить краткое резюме последних событий.

        Args
        ----
        count:
            Количество последних событий для включения в резюме

        Returns
        -------
        Текстовое резюме событий

        """
        if not self.recent_events:
            return 'Недавних событий нет'

        recent = self.recent_events[-count:]
        lines = []

        for event in recent:
            age = time.time() - event['time']
            emoji = '❗' if event.get('important') else '•'
            lines.append(
                f"{emoji} [{age:.0f}s] {event['type']}: {event['content']}"
            )

        return '\n'.join(lines)

    def get_all_events(self) -> List[Dict]:
        """Получить все события в памяти."""
        return self.recent_events.copy()

    def get_events_by_type(self, event_type: str) -> List[Dict]:
        """
        Получить события определенного типа.

        Args
        ----
        event_type:
            Тип событий для получения

        Returns
        -------
        Список событий указанного типа

        """
        if event_type == 'user_speech':
            return self.speech_events.copy()
        elif event_type == 'robot_response':
            return self.robot_response_events.copy()
        elif event_type == 'robot_thought':
            return self.robot_thought_events.copy()
        elif event_type in ['vision', 'apriltag']:
            return self.vision_events.copy()
        elif event_type in ['error', 'warning', 'battery', 'system']:
            return self.system_events.copy()
        else:
            return [e for e in self.recent_events if e['type'] == event_type]

    def get_recent_count(self, event_type: Optional[str] = None) -> int:
        """
        Получить количество событий.

        Args
        ----
        event_type:
            Тип событий для подсчета (если None, подсчет всех событий)

        Returns
        -------
        Количество событий

        """
        if event_type is None:
            return len(self.recent_events)
        return len(self.get_events_by_type(event_type))

    def clear_all(self) -> None:
        """Очистить всю память."""
        self.recent_events.clear()
        self.speech_events.clear()
        self.robot_response_events.clear()
        self.robot_thought_events.clear()
        self.vision_events.clear()
        self.system_events.clear()

    def clear_by_type(self, event_type: str) -> None:
        """
        Очистить события определенного типа.

        Args:
            event_type: Тип событий для очистки
        """
        # Удаляем из общей памяти
        self.recent_events = [
            e for e in self.recent_events if e['type'] != event_type
        ]

        # Удаляем из типизированных очередей
        if event_type == 'user_speech':
            self.speech_events.clear()
        elif event_type == 'robot_response':
            self.robot_response_events.clear()
        elif event_type == 'robot_thought':
            self.robot_thought_events.clear()
        elif event_type in ['vision', 'apriltag']:
            self.vision_events.clear()
        elif event_type in ['error', 'warning', 'battery', 'system']:
            self.system_events.clear()

    def get_important_events(self, count: Optional[int] = None) -> List[Dict]:
        """
        Получить важные события.

        Args:
            count: Количество событий (если None, все важные события)

        Returns:
            Список важных событий
        """
        important = [
            e for e in self.recent_events if e.get('important', False)
        ]
        if count is not None:
            return important[-count:]
        return important

    def has_events(self) -> bool:
        """Проверить наличие событий в памяти."""
        return len(self.recent_events) > 0
