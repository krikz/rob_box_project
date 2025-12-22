#!/usr/bin/env python3
"""
Тест нового dialogue_node с Qwen API и streaming
"""

import os
import json
import time
from openai import OpenAI

# Конфигурация Qwen
API_BASE = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
MODEL = "qwen-max"
API_KEY = "sk-bac6416afb4a4cb69f1a067ffa51db4b"

# Загружаем полный промпт
PROMPT_PATH = "/home/ros2/rob_box_project/src/rob_box_voice/prompts/master_prompt.txt"

with open(PROMPT_PATH, 'r', encoding='utf-8') as f:
    SYSTEM_PROMPT = f.read()

print(f"📖 Промпт загружен: {len(SYSTEM_PROMPT)} символов")

# Создаём клиент
client = OpenAI(
    api_key=API_KEY,
    base_url=API_BASE
)

def test_streaming_request(user_query: str):
    """Тест streaming запроса с полным промптом"""
    print(f"\n{'='*60}")
    print(f"🤔 Запрос: {user_query}")
    print(f"{'='*60}")
    
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": user_query}
    ]
    
    start_time = time.time()
    full_response = ""
    current_chunk = ""
    brace_count = 0
    in_json = False
    chunk_count = 0
    ttfb = None
    
    try:
        stream = client.chat.completions.create(
            model=MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=500,
            stream=True
        )
        
        for chunk in stream:
            if ttfb is None:
                ttfb = time.time() - start_time
                print(f"⏱️  TTFB: {ttfb:.3f}s")
            
            # Проверяем finish_reason
            if chunk.choices[0].finish_reason:
                print(f"🏁 Stream завершён: {chunk.choices[0].finish_reason}")
                break
            
            if chunk.choices[0].delta.content:
                token = chunk.choices[0].delta.content
                full_response += token
                current_chunk += token
                
                # Подсчёт скобок для определения границ JSON
                for char in token:
                    if char == "{":
                        brace_count += 1
                        in_json = True
                    elif char == "}":
                        brace_count -= 1
                
                # Если скобки сбалансированы - парсим
                if in_json and brace_count == 0:
                    json_text = current_chunk.strip()
                    
                    # Убираем markdown ```json если есть
                    if json_text.startswith("```json"):
                        json_text = json_text.replace("```json", "").replace("```", "").strip()
                    
                    # Парсим JSON
                    try:
                        chunk_data = json.loads(json_text)
                        chunk_count += 1
                        
                        print(f"\n📦 Chunk {chunk_count}:")
                        if "ssml" in chunk_data:
                            ssml = chunk_data["ssml"]
                            print(f"  SSML: {ssml[:80]}{'...' if len(ssml) > 80 else ''}")
                        
                        if "emotion" in chunk_data:
                            print(f"  Эмоция: {chunk_data['emotion']}")
                        
                        if "commands" in chunk_data and chunk_data["commands"]:
                            print(f"  Команды: {chunk_data['commands']}")
                        
                        print(f"  JSON: {json.dumps(chunk_data, ensure_ascii=False)}")
                        
                    except json.JSONDecodeError as e:
                        print(f"⚠️  Не удалось распарсить chunk: {e}")
                        print(f"  Текст: {json_text[:100]}")
                    
                    # Сброс для следующего chunk
                    current_chunk = ""
                    in_json = False
                    brace_count = 0
        
        total_time = time.time() - start_time
        print(f"\n✅ Тест завершён за {total_time:.3f}s")
        print(f"📊 Получено {chunk_count} chunks")
        print(f"📝 Полный ответ ({len(full_response)} символов):")
        print(full_response)
        
    except Exception as e:
        print(f"\n❌ Ошибка: {e}")

# Тесты
print("\n🧪 Тест 1: Приветствие")
test_streaming_request("Роббокс, привет!")

print("\n🧪 Тест 2: Команда движения")
test_streaming_request("Поехали вперёд")

print("\n🧪 Тест 3: Эмоциональный запрос")
test_streaming_request("Ты меня любишь?")

print("\n🧪 Тест 4: Математика")
test_streaming_request("Сколько будет 15 умножить на 7?")
