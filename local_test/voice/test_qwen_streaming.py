#!/usr/bin/env python3
"""
Тестовый скрипт для проверки Qwen API со стримингом
"""

import requests
import json
import time

# API Configuration
API_KEY = "sk-bac6416afb4a4cb69f1a067ffa51db4b"
API_BASE = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
MODEL = "qwen-max"

def test_qwen_streaming():
    """Тест стриминга с Qwen API"""
    
    url = f"{API_BASE}/chat/completions"
    
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }
    
    payload = {
        "model": MODEL,
        "messages": [
            {
                "role": "system",
                "content": "Ты ROBBOX - дружелюбный робот-ассистент. Отвечай кратко и весело."
            },
            {
                "role": "user",
                "content": "Привет! Расскажи короткую шутку про роботов."
            }
        ],
        "stream": True,
        "temperature": 0.7,
        "max_tokens": 500
    }
    
    print("="*60)
    print(f"🚀 Тестирование Qwen API")
    print(f"📡 URL: {url}")
    print(f"🤖 Model: {MODEL}")
    print("="*60)
    print()
    
    try:
        start_time = time.time()
        
        # Отправка запроса со стримингом
        response = requests.post(
            url,
            headers=headers,
            json=payload,
            stream=True,
            timeout=30
        )
        
        print(f"📊 Status Code: {response.status_code}")
        print()
        
        if response.status_code != 200:
            print(f"❌ Ошибка: {response.status_code}")
            print(f"Response: {response.text}")
            return
        
        print("💬 Стриминг ответа:")
        print("-"*60)
        
        full_response = ""
        chunk_count = 0
        first_chunk_time = None
        
        # Читаем стриминг построчно
        for line in response.iter_lines():
            if line:
                line_str = line.decode('utf-8')
                
                # SSE формат: "data: {json}"
                if line_str.startswith('data: '):
                    data_str = line_str[6:]  # Убираем "data: "
                    
                    # Проверка на окончание стрима
                    if data_str.strip() == '[DONE]':
                        print()
                        print("-"*60)
                        print("✅ Стриминг завершён")
                        break
                    
                    try:
                        chunk_data = json.loads(data_str)
                        chunk_count += 1
                        
                        if first_chunk_time is None:
                            first_chunk_time = time.time()
                            ttfb = first_chunk_time - start_time
                            print(f"⚡ Time to first byte: {ttfb:.3f}s")
                            print()
                        
                        # Извлекаем контент
                        if 'choices' in chunk_data and len(chunk_data['choices']) > 0:
                            choice = chunk_data['choices'][0]
                            
                            if 'delta' in choice and 'content' in choice['delta']:
                                content = choice['delta']['content']
                                full_response += content
                                print(content, end='', flush=True)
                            
                            # Проверка на завершение
                            if 'finish_reason' in choice and choice['finish_reason']:
                                print()
                                print()
                                print(f"🏁 Finish reason: {choice['finish_reason']}")
                    
                    except json.JSONDecodeError as e:
                        print(f"\n⚠️  JSON decode error: {e}")
                        print(f"Line: {data_str[:100]}")
        
        end_time = time.time()
        total_time = end_time - start_time
        
        print()
        print("="*60)
        print("📈 Статистика:")
        print(f"  Всего чанков: {chunk_count}")
        print(f"  Символов получено: {len(full_response)}")
        print(f"  Общее время: {total_time:.3f}s")
        if first_chunk_time:
            print(f"  TTFB: {first_chunk_time - start_time:.3f}s")
        print("="*60)
        print()
        print("📝 Полный ответ:")
        print(full_response)
        print()
        
    except requests.exceptions.RequestException as e:
        print(f"❌ Ошибка запроса: {e}")
    except Exception as e:
        print(f"❌ Неожиданная ошибка: {e}")
        import traceback
        traceback.print_exc()


def test_qwen_non_streaming():
    """Тест без стриминга для сравнения"""
    
    url = f"{API_BASE}/chat/completions"
    
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }
    
    payload = {
        "model": MODEL,
        "messages": [
            {
                "role": "system",
                "content": "Ты ROBBOX - дружелюбный робот-ассистент. Отвечай кратко и весело."
            },
            {
                "role": "user",
                "content": "Привет! Расскажи короткую шутку про роботов."
            }
        ],
        "stream": False,
        "temperature": 0.7,
        "max_tokens": 500
    }
    
    print()
    print("="*60)
    print(f"🔄 Тестирование без стриминга (для сравнения)")
    print("="*60)
    print()
    
    try:
        start_time = time.time()
        
        response = requests.post(
            url,
            headers=headers,
            json=payload,
            timeout=30
        )
        
        end_time = time.time()
        total_time = end_time - start_time
        
        print(f"📊 Status Code: {response.status_code}")
        print(f"⏱️  Время ответа: {total_time:.3f}s")
        print()
        
        if response.status_code == 200:
            data = response.json()
            
            if 'choices' in data and len(data['choices']) > 0:
                content = data['choices'][0]['message']['content']
                print("💬 Ответ:")
                print("-"*60)
                print(content)
                print("-"*60)
                
                if 'usage' in data:
                    print()
                    print("📊 Использование токенов:")
                    print(f"  Prompt: {data['usage'].get('prompt_tokens', 'N/A')}")
                    print(f"  Completion: {data['usage'].get('completion_tokens', 'N/A')}")
                    print(f"  Total: {data['usage'].get('total_tokens', 'N/A')}")
        else:
            print(f"❌ Ошибка: {response.text}")
    
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    print()
    print("🤖 QWEN API TEST SCRIPT")
    print("="*60)
    
    # Тест со стримингом
    test_qwen_streaming()
    
    # Небольшая пауза
    time.sleep(2)
    
    # Тест без стриминга
    test_qwen_non_streaming()
    
    print()
    print("✅ Тесты завершены!")
    print()
