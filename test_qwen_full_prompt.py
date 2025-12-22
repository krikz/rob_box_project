#!/usr/bin/env python3
"""
Тест Qwen с полным master_prompt.txt от РОБОБОКС
"""

import requests
import json
import time

API_KEY = "sk-bac6416afb4a4cb69f1a067ffa51db4b"
API_BASE = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
MODEL = "qwen-max"

def load_system_prompt():
    """Загружаем master_prompt.txt"""
    with open('/home/ros2/rob_box_project/src/rob_box_voice/prompts/master_prompt.txt', 'r', encoding='utf-8') as f:
        return f.read()

def test_qwen_with_real_prompt():
    """Тест с реальным системным промптом робота"""
    
    system_prompt = load_system_prompt()
    
    print("="*80)
    print(f"🤖 Тестирование Qwen Max с РЕАЛЬНЫМ системным промптом РОББОКС")
    print("="*80)
    print(f"\n📏 Размер system prompt: {len(system_prompt)} символов")
    print(f"📏 Строк: {len(system_prompt.splitlines())}")
    print()
    
    url = f"{API_BASE}/chat/completions"
    
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }
    
    # Разные типы вопросов для тестирования
    test_cases = [
        {
            "name": "Простое приветствие",
            "query": "Роббокс, привет! Как дела?"
        },
        {
            "name": "Команда движения",
            "query": "Поезжай вперёд на полметра"
        },
        {
            "name": "Сложный вопрос",
            "query": "Расскажи что ты можешь делать?"
        },
        {
            "name": "Математика",
            "query": "Что такое теорема Пифагора?"
        }
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n{'='*80}")
        print(f"ТЕСТ {i}/{len(test_cases)}: {test_case['name']}")
        print(f"{'='*80}")
        print(f"❓ Вопрос: {test_case['query']}")
        print("-"*80)
        
        payload = {
            "model": MODEL,
            "messages": [
                {
                    "role": "system",
                    "content": system_prompt
                },
                {
                    "role": "user",
                    "content": test_case['query']
                }
            ],
            "stream": True,
            "temperature": 0.7,
            "max_tokens": 1000
        }
        
        try:
            start_time = time.time()
            
            response = requests.post(
                url,
                headers=headers,
                json=payload,
                stream=True,
                timeout=60
            )
            
            if response.status_code != 200:
                print(f"❌ Ошибка {response.status_code}: {response.text[:200]}")
                continue
            
            print("💬 Ответ:")
            full_response = ""
            first_chunk_time = None
            chunk_count = 0
            
            for line in response.iter_lines():
                if line:
                    line_str = line.decode('utf-8')
                    
                    if line_str.startswith('data: '):
                        data_str = line_str[6:]
                        
                        if data_str.strip() == '[DONE]':
                            break
                        
                        try:
                            chunk_data = json.loads(data_str)
                            chunk_count += 1
                            
                            if first_chunk_time is None:
                                first_chunk_time = time.time()
                                ttfb = first_chunk_time - start_time
                                print(f"⚡ TTFB: {ttfb:.3f}s\n")
                            
                            if 'choices' in chunk_data and len(chunk_data['choices']) > 0:
                                choice = chunk_data['choices'][0]
                                
                                if 'delta' in choice and 'content' in choice['delta']:
                                    content = choice['delta']['content']
                                    full_response += content
                                    print(content, end='', flush=True)
                        
                        except json.JSONDecodeError:
                            pass
            
            end_time = time.time()
            total_time = end_time - start_time
            
            print()
            print()
            print("-"*80)
            print(f"⏱️  Общее время: {total_time:.3f}s")
            print(f"📊 Чанков: {chunk_count}")
            print(f"📏 Символов: {len(full_response)}")
            
            # Пытаемся распарсить как JSON
            print()
            print("🔍 Анализ ответа:")
            try:
                # Ищем JSON в ответе
                json_start = full_response.find('{')
                json_end = full_response.rfind('}') + 1
                
                if json_start >= 0 and json_end > json_start:
                    json_str = full_response[json_start:json_end]
                    parsed = json.loads(json_str)
                    
                    print("✅ Валидный JSON найден!")
                    print(json.dumps(parsed, indent=2, ensure_ascii=False))
                    
                    # Проверяем обязательные поля
                    if 'ssml' in parsed:
                        print("  ✓ Поле 'ssml' присутствует")
                    else:
                        print("  ✗ ОШИБКА: Нет поля 'ssml'")
                    
                    if 'emotion' in parsed:
                        print(f"  ✓ Эмоция: {parsed['emotion']}")
                    
                    if 'commands' in parsed:
                        print(f"  ✓ Команды: {parsed['commands']}")
                
                else:
                    print("⚠️  JSON не найден в ответе")
                    print(f"Ответ: {full_response[:200]}...")
            
            except json.JSONDecodeError as e:
                print(f"❌ Ошибка парсинга JSON: {e}")
                print(f"Строка: {json_str[:100]}...")
        
        except Exception as e:
            print(f"❌ Ошибка: {e}")
            import traceback
            traceback.print_exc()
        
        # Пауза между тестами
        if i < len(test_cases):
            print("\n⏳ Пауза 2 секунды...")
            time.sleep(2)
    
    print()
    print("="*80)
    print("✅ Все тесты завершены!")
    print("="*80)


if __name__ == "__main__":
    test_qwen_with_real_prompt()
