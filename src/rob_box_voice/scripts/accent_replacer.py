#!/usr/bin/env python3
"""
Модуль для автоматической расстановки ударений в тексте
на основе словаря замен
"""

import json
import os
import re
from typing import Dict


class AccentReplacer:
    """Заменяет слова на версии с ударениями."""

    def __init__(self, config_path: str = None):
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(__file__),
                '../config/accent_replacements.json'
            )

        self.replacements = self._load_config(config_path)
        self.word_dict = self._build_word_dict()

    def _load_config(self, path: str) -> dict:
        """Загружает конфиг с заменами."""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"⚠️  Файл {path} не найден, используем пустой словарь")
            return {}

    def _build_word_dict(self) -> Dict[str, str]:
        """Собирает плоский словарь всех замен."""
        word_dict = {}

        for category, words in self.replacements.items():
            if category.startswith('_'):
                continue

            if category == 'homographs':
                # Омографы обрабатываются отдельно
                continue

            if isinstance(words, dict):
                word_dict.update(words)

        return word_dict

    def add_accents(self, text: str) -> str:
        """
        Добавляет ударения в текст

        Args:
            text: исходный текст

        Returns:
            текст с ударениями
        """
        result = text

        # Простые замены (слово целиком)
        for word_no_accent, word_with_accent in self.word_dict.items():
            # Ищем слово с границами (не часть другого слова)
            pattern = r'\b' + re.escape(word_no_accent) + r'\b'
            result = re.sub(pattern, word_with_accent, result, flags=re.IGNORECASE)

        # TODO: Омографы с контекстом (пока пропускаем)

        return result

    def get_stats(self) -> dict:
        """Возвращает статистику словаря."""
        stats = {
            'total_words': len(self.word_dict),
            'categories': {}
        }

        for category, words in self.replacements.items():
            if category.startswith('_'):
                continue

            if category == 'homographs':
                stats['categories'][category] = len(words)
            elif isinstance(words, dict):
                stats['categories'][category] = len(words)

        return stats


if __name__ == "__main__":
    # Тест
    replacer = AccentReplacer()

    print("📊 Статистика словаря:")
    print(json.dumps(replacer.get_stats(), indent=2, ensure_ascii=False))

    print("\n🧪 Тесты:")

    test_cases = [
        "Привет! Я робот РОББОКС из города Сочи.",
        "Теорема Пифагора гласит: квадрат гипотенузы.",
        "Город Москва - столица России.",
        "Система навигации с лидаром работает отлично.",
        "Батарея заряжена на восемьдесят процент."
    ]

    for test in test_cases:
        result = replacer.add_accents(test)
        print(f"\nИсходный: {test}")
        print(f"Результат: {result}")
