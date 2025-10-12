#!/usr/bin/env python3
"""
Нормализация текста для Silero TTS v4

Проблемы Silero TTS:
1. Не читает цифры: "123" → молчание
2. Не читает латиницу: "ROBBOX" → молчание  
3. Не читает спецсимволы: "✅" → молчание

Решение:
- Конвертируем цифры в слова
- Транслитерируем латиницу в кириллицу
- Убираем markdown разметку от DeepSeek
- Удаляем команды <CMD:.../>
- Чистим emoji и спецсимволы
"""

import re
from typing import Dict, List


class TextNormalizer:
    """Нормализация текста для TTS"""
    
    # Маппинг цифр на слова
    DIGITS = {
        '0': 'ноль', '1': 'один', '2': 'два', '3': 'три', '4': 'четыре',
        '5': 'пять', '6': 'шесть', '7': 'семь', '8': 'восемь', '9': 'девять'
    }
    
    TENS = {
        '10': 'десять', '11': 'одиннадцать', '12': 'двенадцать', '13': 'тринадцать',
        '14': 'четырнадцать', '15': 'пятнадцать', '16': 'шестнадцать',
        '17': 'семнадцать', '18': 'восемнадцать', '19': 'девятнадцать',
        '20': 'двадцать', '30': 'тридцать', '40': 'сорок', '50': 'пятьдесят',
        '60': 'шестьдесят', '70': 'семьдесят', '80': 'восемьдесят', '90': 'девяносто'
    }
    
    HUNDREDS = {
        '100': 'сто', '200': 'двести', '300': 'триста', '400': 'четыреста',
        '500': 'пятьсот', '600': 'шестьсот', '700': 'семьсот',
        '800': 'восемьсот', '900': 'девятьсот'
    }
    
    # Транслитерация технических терминов
    TECH_TERMS = {
        'ROBBOX': 'РОББОКС',
        'RobBox': 'РОББОКС',
        'robbox': 'роббокс',
        'WiFi': 'вай фай',
        'wifi': 'вай фай',
        'LiDAR': 'лидар',
        'lidar': 'лидар',
        'LIDAR': 'лидар',
        'GPS': 'джи пи эс',
        'gps': 'джи пи эс',
        'IMU': 'ай эм ю',
        'imu': 'ай эм ю',
        'ROS': 'рос',
        'ros': 'рос',
        'LED': 'лед',
        'led': 'лед',
        'RGB': 'эр джи би',
        'rgb': 'эр джи би',
        'USB': 'ю эс би',
        'usb': 'ю эс би',
        'CPU': 'цэ пэ у',
        'cpu': 'цэ пэ у',
        'GPU': 'джи пи ю',
        'gpu': 'джи пи ю',
        'RAM': 'рам',
        'ram': 'рам',
        'Pi': 'пай',
        'Raspberry': 'распберри',
        'raspberry': 'распберри',
        'Linux': 'линукс',
        'linux': 'линукс',
        'Ubuntu': 'убунту',
        'ubuntu': 'убунту',
        'VESC': 'веск',
        'vesc': 'веск',
        'AprilTag': 'эйприл тэг',
        'apriltag': 'эйприл тэг',
        'SLAM': 'слэм',
        'slam': 'слэм',
        'OK': 'окей',
        'ok': 'окей',
        'IP': 'ай пи',
        'ip': 'ай пи',
    }
    
    # Emoji → текст
    EMOJI_MAP = {
        '✅': '',  # Убираем галочки
        '❌': '',  # Убираем крестики
        '⚠️': 'внимание',
        '🔋': 'батарея',
        '🤖': 'робот',
        '📡': 'связь',
        '🎯': '',
        '⏸️': 'пауза',
        '▶️': '',
        '⏹️': 'стоп',
    }
    
    def normalize(self, text: str) -> str:
        """
        Полная нормализация текста для TTS
        
        Args:
            text: Исходный текст (может содержать markdown, команды, цифры, латиницу)
            
        Returns:
            Нормализованный текст готовый для TTS
        """
        # 1. Удаляем команды <CMD:.../>
        text = self.remove_commands(text)
        
        # 2. Удаляем markdown разметку
        text = self.remove_markdown(text)
        
        # 3. Заменяем emoji
        text = self.replace_emoji(text)
        
        # 4. Заменяем технические термины
        text = self.replace_tech_terms(text)
        
        # 5. Конвертируем числа в слова
        text = self.numbers_to_words(text)
        
        # 6. Убираем лишние пробелы
        text = re.sub(r'\s+', ' ', text).strip()
        
        return text
    
    def remove_commands(self, text: str) -> str:
        """Удаление команд вида <CMD:.../>"""
        return re.sub(r'<CMD:[^>]+/>', '', text)
    
    def remove_markdown(self, text: str) -> str:
        """Удаление markdown разметки"""
        # **bold** → bold
        text = re.sub(r'\*\*([^*]+)\*\*', r'\1', text)
        
        # *italic* → italic
        text = re.sub(r'\*([^*]+)\*', r'\1', text)
        
        # __underline__ → underline
        text = re.sub(r'__([^_]+)__', r'\1', text)
        
        # Списки: - item или * item → item
        text = re.sub(r'^[\-\*]\s+', '', text, flags=re.MULTILINE)
        
        # Заголовки: ## Header → Header
        text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)
        
        # Код: `code` → code
        text = re.sub(r'`([^`]+)`', r'\1', text)
        
        # Ссылки: [text](url) → text
        text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', text)
        
        return text
    
    def replace_emoji(self, text: str) -> str:
        """Замена emoji на текст"""
        for emoji, replacement in self.EMOJI_MAP.items():
            text = text.replace(emoji, replacement)
        return text
    
    def replace_tech_terms(self, text: str) -> str:
        """Замена технических терминов на кириллицу"""
        # Сортируем по длине (сначала длинные) чтобы избежать частичных замен
        sorted_terms = sorted(self.TECH_TERMS.items(), key=lambda x: len(x[0]), reverse=True)
        
        for english, russian in sorted_terms:
            # Word boundary для точной замены
            text = re.sub(r'\b' + re.escape(english) + r'\b', russian, text)
        
        return text
    
    def numbers_to_words(self, text: str) -> str:
        """Конвертация чисел в слова"""
        # Находим все числа
        def convert_number(match):
            num_str = match.group(0)
            num = int(num_str)  # int() автоматически убирает ведущие нули
            
            # Простые числа 0-9
            if 0 <= num <= 9:
                return self.DIGITS[str(num)]
            
            # 10-19
            if 10 <= num <= 19:
                return self.TENS[str(num)]
            
            # 20-99
            if 20 <= num <= 99:
                tens = (num // 10) * 10
                units = num % 10
                result = self.TENS[str(tens)]
                if units > 0:
                    result += ' ' + self.DIGITS[str(units)]
                return result
            
            # 100-999
            if 100 <= num <= 999:
                hundreds = (num // 100) * 100
                remainder = num % 100
                result = self.HUNDREDS[str(hundreds)]
                
                if remainder > 0:
                    if 10 <= remainder <= 19:
                        result += ' ' + self.TENS[str(remainder)]
                    elif remainder >= 20:
                        tens = (remainder // 10) * 10
                        units = remainder % 10
                        result += ' ' + self.TENS[str(tens)]
                        if units > 0:
                            result += ' ' + self.DIGITS[str(units)]
                    else:
                        result += ' ' + self.DIGITS[str(remainder)]
                
                return result
            
            # Для больших чисел просто читаем по цифрам
            if num >= 1000:
                return ' '.join(self.DIGITS[str(int(d))] for d in str(num))
            
            return num_str
        
        # Заменяем все числа
        text = re.sub(r'\b\d+\b', convert_number, text)
        
        return text
    
    def clean_for_speech(self, text: str) -> List[str]:
        """
        Очистка и разбивка текста на предложения для речи
        
        Returns:
            Список предложений готовых для озвучивания
        """
        # Нормализуем
        text = self.normalize(text)
        
        # Разбиваем на предложения
        sentences = re.split(r'[.!?]+', text)
        
        # Фильтруем пустые и очищаем
        sentences = [s.strip() for s in sentences if s.strip()]
        
        return sentences


# Пример использования
if __name__ == "__main__":
    normalizer = TextNormalizer()
    
    # Тестовые примеры
    test_cases = [
        "ROBBOX готов! WiFi активен на 192.168.1.1",
        "Обнаружено 3 препятствия впереди",
        "✅ Двигатели: VESC драйверы онлайн",
        "**Статус систем:** LiDAR активен, IMU калиброван",
        "<CMD:stop/> Начинаю обновление программного обеспечения.",
        "Температура: 32°C, влажность: 45%",
        "Батарея: 78% заряда",
        "- ✅ Двигатели онлайн\n- ✅ Камера работает",
        "Raspberry Pi 5 с Ubuntu 24.04",
        "AprilTag detection готов, SLAM активен"
    ]
    
    print("=" * 60)
    print("ТЕСТ НОРМАЛИЗАЦИИ ТЕКСТА ДЛЯ TTS")
    print("=" * 60)
    print()
    
    for i, test in enumerate(test_cases, 1):
        normalized = normalizer.normalize(test)
        print(f"{i}. Оригинал:")
        print(f"   {test}")
        print(f"   Нормализованный:")
        print(f"   {normalized}")
        print()
    
    print("=" * 60)
    print("ПРИМЕР ОЧИСТКИ ОТВЕТА DEEPSEEK")
    print("=" * 60)
    print()
    
    deepseek_answer = """Запускаю диагностику всех систем...

**Статус систем:**
- ✅ Двигатели: VESC драйверы онлайн, энкодеры работают
- ✅ LiDAR: LSLIDAR N10 активен, сканирует окружение  
- ✅ Камера: OAK-D Lite подключена, AprilTag detection готов
- ✅ IMU: гироскоп и акселерометр калиброваны
- ✅ Микрофоны: ReSpeaker массив слушает
- ✅ LED матрица: 400 пикселей работают
- ✅ Сеть: WiFi точка доступа активна
- ✅ Батарея: 78% заряда, стабильное напряжение

**Параметры:**
- Температура внутри: 32°C
- Влажность: 45%
- Обнаружено препятствий: 3
- Текущая позиция: x=1.23м, y=-0.45м, θ=45°

Все системы работают нормально. Готов к работе!"""
    
    print("Оригинал:")
    print(deepseek_answer)
    print()
    print("Очищенные предложения для TTS:")
    sentences = normalizer.clean_for_speech(deepseek_answer)
    for i, sent in enumerate(sentences, 1):
        print(f"{i}. {sent}")
