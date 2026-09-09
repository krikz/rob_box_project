#!/usr/bin/env python3
"""Тест streaming режима DeepSeek с большим промптом"""

import json
import os
import time
from pathlib import Path
from openai import OpenAI

# API ключ
api_key = os.getenv("DEEPSEEK_API_KEY", "sk-a9bb742844e344658e5987c0ccaeae07")

# МИНИМАЛЬНЫЙ prompt для теста
system_prompt = "Ты робот. Отвечай JSON chunks: {\"chunk\": N, \"ssml\": \"<speak>текст</speak>\"}"
print(f"✅ Тестовый prompt: {len(system_prompt)} байт (~{len(system_prompt) // 4} токенов)\n")

# Создаём клиент
client = OpenAI(
    api_key=api_key,
    base_url="https://api.deepseek.com",
    timeout=30.0
)

user_query = "расскажи анекдот"
messages = [
    {"role": "system", "content": system_prompt},
    {"role": "user", "content": user_query}
]

print("=" * 80)
print("🧪 ТЕСТ: Streaming режим с БОЛЬШИМ промптом")
print("=" * 80)
print(f"👤 User: {user_query}")
print("🤔 Запрос через streaming...")
print()

start = time.time()
full_response = ""
chunk_count = 0
first_chunk_time = None

try:
    stream = client.chat.completions.create(
        model="deepseek-chat",
        messages=messages,
        temperature=0.7,
        max_tokens=500,
        stream=True  # STREAMING!
    )
    
    for chunk in stream:
        if chunk.choices[0].delta.content:
            content = chunk.choices[0].delta.content
            full_response += content
            chunk_count += 1
            
            if first_chunk_time is None:
                first_chunk_time = time.time() - start
                print(f"⚡ Первый chunk получен за {first_chunk_time:.2f}s")
            
            # Показываем КАЖДЫЙ chunk для отладки
            print(f"   Chunk #{chunk_count}: '{content}' (всего {len(full_response)} символов)")
    
    elapsed = time.time() - start
    
    print()
    print(f"✅ Streaming завершён за {elapsed:.2f}s")
    print(f"   Всего chunks: {chunk_count}")
    print(f"   Всего символов: {len(full_response)}")
    print(f"   Время до первого chunk: {first_chunk_time:.2f}s")
    print()
    print("=" * 80)
    print("Ответ:")
    print("=" * 80)
    print(full_response[:500] + "..." if len(full_response) > 500 else full_response)
    print("=" * 80)
    
    # Проверяем JSON chunks
    json_count = full_response.count('"chunk"')
    print(f"\n✅ Найдено ~{json_count} JSON chunks в ответе")
    
except Exception as e:
    elapsed = time.time() - start
    print(f"❌ Ошибка после {elapsed:.2f}s: {e}")
    print(f"   Получено chunks до ошибки: {chunk_count}")
    print(f"   Получено символов: {len(full_response)}")
