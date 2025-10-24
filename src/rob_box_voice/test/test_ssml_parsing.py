#!/usr/bin/env python3
"""
Тест SSML парсинга для TTS Node

Проверяет извлечение атрибутов pitch и rate из SSML тегов.
"""

import re


def _parse_ssml_attributes(ssml: str) -> dict:
    """
    Извлекает атрибуты из SSML тегов (pitch, rate/speed)
    
    Returns:
        dict: {'pitch': float, 'rate': float} или пустой dict
    """
    attributes = {}
    
    # Ищем <prosody> теги с атрибутами
    prosody_pattern = r'<prosody\s+([^>]+)>'
    matches = re.finditer(prosody_pattern, ssml, re.IGNORECASE)
    
    for match in matches:
        attrs_str = match.group(1)
        
        # Парсим pitch
        pitch_match = re.search(r'pitch\s*=\s*["\']?([^"\'>\s]+)["\']?', attrs_str, re.IGNORECASE)
        if pitch_match:
            pitch_value = pitch_match.group(1)
            # Конвертируем в множитель для Yandex
            if '%' in pitch_value:
                try:
                    percent = float(pitch_value.replace('%', ''))
                    attributes['pitch'] = 1.0 + (percent / 100.0)
                except ValueError:
                    pass
            elif pitch_value == 'high':
                attributes['pitch'] = 1.2
            elif pitch_value == 'low':
                attributes['pitch'] = 0.8
            elif pitch_value == 'medium':
                attributes['pitch'] = 1.0
            else:
                try:
                    attributes['pitch'] = float(pitch_value)
                except ValueError:
                    pass
        
        # Парсим rate
        rate_match = re.search(r'rate\s*=\s*["\']?([^"\'>\s]+)["\']?', attrs_str, re.IGNORECASE)
        if rate_match:
            rate_value = rate_match.group(1)
            if '%' in rate_value:
                try:
                    percent = float(rate_value.replace('%', ''))
                    attributes['rate'] = percent / 100.0
                except ValueError:
                    pass
            elif rate_value == 'fast':
                attributes['rate'] = 1.5
            elif rate_value == 'slow':
                attributes['rate'] = 0.7
            elif rate_value == 'medium':
                attributes['rate'] = 1.0
            else:
                try:
                    attributes['rate'] = float(rate_value)
                except ValueError:
                    pass
    
    return attributes


def test_ssml_parsing():
    """Тестирование различных SSML форматов"""
    
    test_cases = [
        # (SSML, Expected attributes)
        ('<speak><prosody rate="1.5">Быстрая речь</prosody></speak>', {'rate': 1.5}),
        ('<speak><prosody rate="fast">Быстро</prosody></speak>', {'rate': 1.5}),
        ('<speak><prosody rate="slow">Медленно</prosody></speak>', {'rate': 0.7}),
        ('<speak><prosody rate="150%">Полтора скорости</prosody></speak>', {'rate': 1.5}),
        ('<speak><prosody pitch="+10%">Выше на 10%</prosody></speak>', {'pitch': 1.1}),
        ('<speak><prosody pitch="-10%">Ниже на 10%</prosody></speak>', {'pitch': 0.9}),
        ('<speak><prosody pitch="high">Высокий голос</prosody></speak>', {'pitch': 1.2}),
        ('<speak><prosody pitch="low">Низкий голос</prosody></speak>', {'pitch': 0.8}),
        (
            '<speak><prosody pitch="high" rate="1.2">Высокий и быстрый</prosody></speak>',
            {'pitch': 1.2, 'rate': 1.2}
        ),
        ('<speak>Обычная речь без атрибутов</speak>', {}),
    ]
    
    print("🧪 Тест SSML парсинга\n")
    
    passed = 0
    failed = 0
    
    for ssml, expected in test_cases:
        result = _parse_ssml_attributes(ssml)
        
        if result == expected:
            print(f"✅ PASS: {ssml[:60]}...")
            print(f"   Результат: {result}")
            passed += 1
        else:
            print(f"❌ FAIL: {ssml[:60]}...")
            print(f"   Ожидалось: {expected}")
            print(f"   Получено:  {result}")
            failed += 1
    
    print(f"\n📊 Результаты: {passed} passed, {failed} failed")
    
    return failed == 0


if __name__ == '__main__':
    success = test_ssml_parsing()
    exit(0 if success else 1)
