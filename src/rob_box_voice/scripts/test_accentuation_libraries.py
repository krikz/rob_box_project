#!/usr/bin/env python3
"""
Тестирование библиотек для автоматической расстановки ударений в русском языке

Библиотеки для тестирования:
1. russtress - https://github.com/MashaPo/russtress
2. accentor - https://github.com/Desklop/accentor
3. russian-accentuation - https://github.com/reynoldsnlp/russian-accentuation
"""

import sys

print("=" * 70)
print("ТЕСТИРОВАНИЕ БИБЛИОТЕК ДЛЯ РАССТАНОВКИ УДАРЕНИЙ")
print("=" * 70)
print()

# Тестовые фразы
test_phrases = [
    "Город Сочи находится на побережье Черного моря.",
    "Открываю замок на двери замка.",
    "Белки содержат много белка.",
    "Робот движется вперед со скоростью один метр в секунду.",
    "Множество Мандельброта это известный фрактал.",
    "Я несу муку домой, какая мука!",
    "В замке открыли замок.",
    "Белки прыгают по деревьям и едят белки.",
]

# 1. Тестируем russtress
print("1. Библиотека: russtress")
print("-" * 70)
try:
    import russtress
    print("✅ Установлена")
    print("\nПримеры:")
    for phrase in test_phrases[:3]:
        try:
            stressed = russtress.put_stress(phrase, stress_symbol='+')
            print(f"  Исходный: {phrase}")
            print(f"  С ударениями: {stressed}")
            print()
        except Exception as e:
            print(f"  ❌ Ошибка: {e}")
except ImportError:
    print("❌ Не установлена")
    print("   Установка: pip install russtress")
print()

# 2. Тестируем accentor
print("2. Библиотека: accentor")
print("-" * 70)
try:
    from accentor import Accentor
    print("✅ Установлена")
    print("\nПримеры:")
    accentor = Accentor()
    for phrase in test_phrases[:3]:
        try:
            stressed = accentor.accent(phrase, stress_symbol='+')
            print(f"  Исходный: {phrase}")
            print(f"  С ударениями: {stressed}")
            print()
        except Exception as e:
            print(f"  ❌ Ошибка: {e}")
except ImportError:
    print("❌ Не установлена")
    print("   Установка: pip install accentor")
except Exception as e:
    print(f"❌ Ошибка инициализации: {e}")
print()

# 3. Тестируем russian-accentuation (ru_accentor)
print("3. Библиотека: russian-accentuation")
print("-" * 70)
try:
    from russian_accentuation import accentize
    print("✅ Установлена")
    print("\nПримеры:")
    for phrase in test_phrases[:3]:
        try:
            # Эта библиотека использует символ ́ (combining acute accent)
            stressed = accentize(phrase)
            # Конвертируем в формат Silero (+)
            import unicodedata
            stressed_silero = ""
            skip_next = False
            for i, char in enumerate(stressed):
                if skip_next:
                    skip_next = False
                    continue
                if unicodedata.category(char) == 'Mn':  # Combining mark
                    # Пропускаем combining accent, добавляем + перед предыдущей гласной
                    stressed_silero = stressed_silero[:-1] + '+' + stressed_silero[-1]
                else:
                    stressed_silero += char
            
            print(f"  Исходный: {phrase}")
            print(f"  С ударениями (оригинал): {stressed}")
            print(f"  С ударениями (Silero): {stressed_silero}")
            print()
        except Exception as e:
            print(f"  ❌ Ошибка: {e}")
except ImportError:
    print("❌ Не установлена")
    print("   Установка: pip install russian-accentuation")
except Exception as e:
    print(f"❌ Ошибка: {e}")
print()

# 4. Тестируем RuPOS (часть русского NLP)
print("4. Библиотека: RuPOS (pymorphy2 + словарь ударений)")
print("-" * 70)
try:
    import pymorphy2
    print("✅ pymorphy2 установлен")
    print("   Примечание: pymorphy2 не дает ударения напрямую,")
    print("   но можно использовать в связке со словарями ударений")
except ImportError:
    print("❌ pymorphy2 не установлен")
    print("   Установка: pip install pymorphy2")
print()

# 5. Резюме
print("=" * 70)
print("РЕЗЮМЕ И РЕКОМЕНДАЦИИ")
print("=" * 70)
print("""
Наиболее перспективные решения:

1. russtress (https://github.com/MashaPo/russtress)
   - Специализированная библиотека для русских ударений
   - Простой API
   - Использует нейросеть для предсказания
   - ✅ Рекомендуется попробовать первой

2. accentor (https://github.com/Desklop/accentor)
   - Также на основе ML модели
   - Хорошая точность для большинства случаев
   - ✅ Хорошая альтернатива

3. russian-accentuation
   - Использует словари и правила
   - Может быть медленнее на больших текстах
   - Формат ударений нужно конвертировать

Для интеграции в систему:
- Добавить в text_normalizer.py функцию auto_accent()
- Применять перед отправкой в TTS
- Кэшировать результаты для частых слов
- Можно сделать опциональным (включать/выключать)
""")

print("\nДля установки всех библиотек:")
print("pip install russtress accentor russian-accentuation pymorphy2")
