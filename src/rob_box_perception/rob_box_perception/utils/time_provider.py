#!/usr/bin/env python3
"""
time_provider.py - Time Awareness Provider

Провайдер осознания текущего времени и контекста времени суток.

Dependencies:
    - pytz (optional): For timezone support. If unavailable, falls back to local time
                       without timezone awareness. Install with: pip install pytz

When pytz is not available:
    - Timezone parameter is ignored
    - Local system time is used instead
    - All timezone-related features fall back to local time
"""

import time
from datetime import datetime
from typing import Dict

try:
    import pytz

    PYTZ_AVAILABLE = True
except ImportError:
    PYTZ_AVAILABLE = False


class TimeAwarenessProvider:
    """
    Провайдер осознания времени.

    Предоставляет текущее время с контекстом времени суток
    (утро, день, вечер, ночь) и human-readable форматирование.

    Attributes:
        timezone: Часовой пояс (pytz timezone)
    """

    def __init__(self, timezone: str = "Europe/Moscow"):
        """
        Инициализация провайдера.

        Args:
            timezone: Название часового пояса (например, 'Europe/Moscow')
        """
        self.timezone_name = timezone

        if PYTZ_AVAILABLE:
            try:
                self.timezone = pytz.timezone(timezone)
            except pytz.UnknownTimeZoneError:
                self.timezone = pytz.UTC
        else:
            self.timezone = None

    def get_current_time_context(self) -> Dict:
        """
        Получить контекст текущего времени.

        Returns:
            Словарь с контекстом времени:
            - timestamp: Unix timestamp
            - datetime: ISO 8601 формат
            - human_readable: "YYYY-MM-DD HH:MM:SS"
            - time_only: "HH:MM"
            - date_only: "YYYY-MM-DD"
            - hour: Час (0-23)
            - minute: Минута (0-59)
            - weekday: День недели (English)
            - weekday_ru: День недели (Russian)
            - period: Период суток (morning/day/evening/night)
            - period_ru: Период суток (утро/день/вечер/ночь)
            - timezone: Название часового пояса
        """
        if self.timezone:
            now = datetime.now(self.timezone)
        else:
            now = datetime.now()

        hour = now.hour

        # Определяем период суток
        if 5 <= hour < 12:
            period = "morning"
            period_ru = "утро"
        elif 12 <= hour < 17:
            period = "day"
            period_ru = "день"
        elif 17 <= hour < 22:
            period = "evening"
            period_ru = "вечер"
        else:
            period = "night"
            period_ru = "ночь"

        return {
            "timestamp": time.time(),
            "datetime": now.isoformat(),
            "human_readable": now.strftime("%Y-%m-%d %H:%M:%S"),
            "time_only": now.strftime("%H:%M"),
            "date_only": now.strftime("%Y-%m-%d"),
            "hour": hour,
            "minute": now.minute,
            "weekday": now.strftime("%A"),
            "weekday_ru": self._get_weekday_ru(now.weekday()),
            "period": period,
            "period_ru": period_ru,
            "timezone": self.timezone_name if self.timezone else "local",
        }

    def _get_weekday_ru(self, weekday: int) -> str:
        """
        Получить день недели на русском.

        Args:
            weekday: Номер дня недели (0-6)

        Returns:
            Название дня недели на русском
        """
        days = {
            0: "Понедельник",
            1: "Вторник",
            2: "Среда",
            3: "Четверг",
            4: "Пятница",
            5: "Суббота",
            6: "Воскресенье",
        }
        return days.get(weekday, "Неизвестно")
