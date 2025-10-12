#!/usr/bin/env python3
"""
Расширенная нормализация текста для Silero TTS v4

Поддержка:
1. JSON формат с полями: text, ssml, commands, emotion
2. SSML теги для интонаций (<prosody>, <break>)
3. Команды роботу (<CMD:.../>)
4. Нормализация чисел, латиницы, markdown
"""

import re
import json
from typing import Dict, List, Tuple, Optional
from text_normalizer import TextNormalizer as BaseNormalizer


class ResponseParser:
    """Парсер JSON ответа от DeepSeek"""
    
    def __init__(self):
        self.base_normalizer = BaseNormalizer()
    
    def parse_response(self, response: str) -> Dict:
        """
        Парсит JSON ответ от DeepSeek
        
        Returns:
            {
                'text': str,           # Нормализованный текст для TTS
                'ssml': str,           # SSML разметка (если есть)
                'commands': List[str], # Команды для выполнения
                'emotion': str         # Эмоция (neutral/happy/sad/etc)
            }
        """
        try:
            # Попытка распарсить JSON
            data = json.loads(response.strip())
        except json.JSONDecodeError:
            # Если не JSON - считаем что это просто текст
            return {
                'text': self.base_normalizer.normalize(response),
                'ssml': None,
                'commands': [],
                'emotion': 'neutral'
            }
        
        # Извлекаем поля
        text = data.get('text', '')
        ssml = data.get('ssml', None)
        commands = data.get('commands', [])
        emotion = data.get('emotion', 'neutral')
        
        # Нормализуем обычный текст
        normalized_text = self.base_normalizer.normalize(text)
        
        # Нормализуем текст внутри SSML (если есть)
        normalized_ssml = None
        if ssml:
            normalized_ssml = self._normalize_ssml(ssml)
        
        return {
            'text': normalized_text,
            'ssml': normalized_ssml,
            'commands': commands,
            'emotion': emotion
        }
    
    def _normalize_ssml(self, ssml: str) -> str:
        """
        Нормализует текст внутри SSML тегов
        
        Сохраняет SSML теги (<speak>, <prosody>, <break>)
        Нормализует только текстовое содержимое
        """
        # Найти все текстовые узлы (вне тегов)
        # Простой подход: заменяем контент между >текст<
        
        def normalize_text_node(match):
            text = match.group(1)
            # Если это не тег (не начинается с <)
            if not text.startswith('<'):
                return '>' + self.base_normalizer.normalize(text) + '<'
            return match.group(0)
        
        # Паттерн: захватывает текст между > и <
        pattern = r'>([^<]+)<'
        normalized = re.sub(pattern, normalize_text_node, ssml)
        
        return normalized


class SSMLProcessor:
    """Обработчик SSML для Silero TTS"""
    
    @staticmethod
    def extract_text_from_ssml(ssml: str) -> str:
        """
        Извлекает чистый текст из SSML
        (на случай если Silero не поддерживает SSML)
        """
        # Убираем все теги
        text = re.sub(r'<[^>]+>', '', ssml)
        return text.strip()
    
    @staticmethod
    def parse_ssml_for_timing(ssml: str) -> List[Tuple[str, Optional[int]]]:
        """
        Парсит SSML и возвращает список фраз с паузами
        
        Returns:
            [(text1, pause_ms), (text2, pause_ms), ...]
        """
        chunks = []
        
        # Удаляем обёртку <speak>
        ssml = re.sub(r'<speak>|</speak>', '', ssml)
        
        # Разбиваем по тегам <break>
        parts = re.split(r'<break\s+time="(\d+)ms"\s*/>', ssml)
        
        for i, part in enumerate(parts):
            if i % 2 == 0:
                # Это текст
                text = SSMLProcessor.extract_text_from_ssml(part)
                if text:
                    chunks.append((text, None))
            else:
                # Это время паузы после предыдущего текста
                if chunks:
                    text, _ = chunks[-1]
                    chunks[-1] = (text, int(part))
        
        return chunks
    
    @staticmethod
    def has_ssml_tags(text: str) -> bool:
        """Проверяет есть ли в тексте SSML теги"""
        return bool(re.search(r'<(speak|break)', text))


class CommandExecutor:
    """Исполнитель команд робота"""
    
    VALID_COMMANDS = {
        'move_forward': float,
        'move_backward': float,
        'turn_left': float,
        'turn_right': float,
        'stop': None,
        'scan_360': None,
        'led_emotion': str,
        'get_battery': None,
        'get_sensors': None,
    }
    
    @staticmethod
    def parse_command(cmd_string: str) -> Tuple[str, Optional[str]]:
        """
        Парсит строку команды
        
        Args:
            cmd_string: "command:param" или "command"
            
        Returns:
            (command, param)
        """
        if ':' in cmd_string:
            command, param = cmd_string.split(':', 1)
            return command.strip(), param.strip()
        return cmd_string.strip(), None
    
    @staticmethod
    def validate_command(command: str, param: Optional[str] = None) -> bool:
        """Валидирует команду и параметр"""
        if command not in CommandExecutor.VALID_COMMANDS:
            return False
        
        expected_type = CommandExecutor.VALID_COMMANDS[command]
        
        if expected_type is None:
            # Команда без параметров
            return param is None
        
        if expected_type is float:
            try:
                float(param)
                return True
            except (ValueError, TypeError):
                return False
        
        if expected_type is str:
            return param is not None
        
        return False
    
    @staticmethod
    def execute(command: str, param: Optional[str] = None):
        """
        Выполняет команду (пока что просто печатает)
        В будущем здесь будет ROS2 integration
        """
        print(f"🤖 КОМАНДА: {command}", end='')
        if param:
            print(f" [{param}]")
        else:
            print()
        
        # TODO: ROS2 publisher для команд
        # self.command_pub.publish(Command(name=command, param=param))


# Основной класс для интеграции
class RobboxVoiceHandler:
    """
    Главный обработчик голосовых ответов ROBBOX
    
    Полный цикл:
    1. Получает JSON от DeepSeek
    2. Извлекает команды и выполняет
    3. Нормализует текст
    4. Возвращает готовые фразы для TTS
    """
    
    def __init__(self):
        self.parser = ResponseParser()
        self.ssml_processor = SSMLProcessor()
        self.executor = CommandExecutor()
    
    def process_response(self, response: str, execute_commands: bool = True) -> Dict:
        """
        Обрабатывает ответ DeepSeek
        
        Args:
            response: JSON или текст от DeepSeek
            execute_commands: Выполнять ли команды (False для тестов)
            
        Returns:
            {
                'phrases': List[str],      # Фразы для озвучивания
                'ssml_chunks': List[tuple], # [(text, pause_ms), ...] если SSML
                'commands': List[str],      # Выполненные команды
                'emotion': str              # Эмоция
            }
        """
        # Парсим ответ
        parsed = self.parser.parse_response(response)
        
        # Выполняем команды
        if execute_commands and parsed['commands']:
            print(f"\n{'='*60}")
            print("⚙️  ВЫПОЛНЕНИЕ КОМАНД:")
            print('='*60)
            for cmd_string in parsed['commands']:
                command, param = self.executor.parse_command(cmd_string)
                if self.executor.validate_command(command, param):
                    self.executor.execute(command, param)
                else:
                    print(f"⚠️  Неизвестная команда: {cmd_string}")
        
        # Готовим данные для TTS
        result = {
            'phrases': [],
            'ssml_chunks': [],
            'commands': parsed['commands'],
            'emotion': parsed['emotion']
        }
        
        # Если есть SSML - используем его
        if parsed['ssml'] and self.ssml_processor.has_ssml_tags(parsed['ssml']):
            result['ssml_chunks'] = self.ssml_processor.parse_ssml_for_timing(parsed['ssml'])
            # Для совместимости также сохраняем просто текст
            result['phrases'] = [chunk[0] for chunk in result['ssml_chunks']]
        else:
            # Иначе используем нормализованный текст
            # Разбиваем на предложения
            sentences = re.split(r'[.!?]+', parsed['text'])
            result['phrases'] = [s.strip() for s in sentences if s.strip()]
        
        return result


# ==================== ТЕСТЫ ====================

def test_json_parsing():
    """Тест парсинга JSON"""
    parser = ResponseParser()
    
    print("="*60)
    print("ТЕСТ 1: Простой JSON")
    print("="*60)
    
    json_response = '''
    {
        "text": "Привет! Я ROBBOX робот. Батарея 78%.",
        "emotion": "happy"
    }
    '''
    
    result = parser.parse_response(json_response)
    print(f"Text: {result['text']}")
    print(f"Emotion: {result['emotion']}")
    print()


def test_ssml_parsing():
    """Тест парсинга SSML"""
    parser = ResponseParser()
    processor = SSMLProcessor()
    
    print("="*60)
    print("ТЕСТ 2: SSML с интонациями")
    print("="*60)
    
    json_response = '''
    {
        "text": "Дай подумаю. Кажется я знаю решение.",
        "ssml": "<speak><prosody rate=\\"slow\\">Дай подумаю...</prosody><break time=\\"800ms\\"/><prosody pitch=\\"medium\\">Кажется я знаю решение.</prosody></speak>",
        "emotion": "thinking"
    }
    '''
    
    result = parser.parse_response(json_response)
    print(f"Text: {result['text']}")
    print(f"SSML: {result['ssml']}")
    
    # Разбираем SSML
    chunks = processor.parse_ssml_for_timing(result['ssml'])
    print("\nSSML chunks:")
    for text, pause in chunks:
        print(f"  - '{text}' {f'(pause {pause}ms)' if pause else ''}")
    print()


def test_commands():
    """Тест парсинга и выполнения команд"""
    handler = RobboxVoiceHandler()
    
    print("="*60)
    print("ТЕСТ 3: Команды робота")
    print("="*60)
    
    json_response = '''
    {
        "text": "Хорошо, еду вперёд на скорости 0.3 метра в секунду.",
        "commands": ["move_forward:0.3", "led_emotion:neutral"],
        "emotion": "neutral"
    }
    '''
    
    result = handler.process_response(json_response)
    print(f"\nText для озвучивания: {result['phrases']}")
    print(f"Emotion: {result['emotion']}")
    print()


def test_full_response():
    """Тест полного сложного ответа"""
    handler = RobboxVoiceHandler()
    
    print("="*60)
    print("ТЕСТ 4: Сложный ответ с SSML + командами")
    print("="*60)
    
    json_response = '''
    {
        "text": "Ура! Задача выполнена успешно! Возвращаюсь на базу.",
        "ssml": "<speak><prosody pitch=\\"high\\" rate=\\"fast\\">Ура!</prosody><break time=\\"300ms\\"/>Задача выполнена успешно!<break time=\\"500ms\\"/>Возвращаюсь на базу.</speak>",
        "commands": ["stop", "turn_left:180", "move_forward:0.5", "led_emotion:happy"],
        "emotion": "happy"
    }
    '''
    
    result = handler.process_response(json_response)
    print(f"\nФразы для озвучивания:")
    for i, phrase in enumerate(result['phrases'], 1):
        print(f"  {i}. {phrase}")
    
    if result['ssml_chunks']:
        print(f"\nSSML chunks с паузами:")
        for text, pause in result['ssml_chunks']:
            print(f"  - '{text}' {f'→ пауза {pause}ms' if pause else ''}")
    print()


def test_plain_text():
    """Тест обычного текста (не JSON)"""
    handler = RobboxVoiceHandler()
    
    print("="*60)
    print("ТЕСТ 5: Обычный текст (backward compatibility)")
    print("="*60)
    
    text_response = "Привет! Я ROBBOX. WiFi активен на 192.168.1.1."
    
    result = handler.process_response(text_response)
    print(f"Text: {result['phrases']}")
    print()


if __name__ == '__main__':
    print("\n🧪 ТЕСТИРОВАНИЕ TEXT NORMALIZER V2\n")
    
    test_json_parsing()
    test_ssml_parsing()
    test_commands()
    test_full_response()
    test_plain_text()
    
    print("="*60)
    print("✅ ВСЕ ТЕСТЫ ЗАВЕРШЕНЫ")
    print("="*60)
