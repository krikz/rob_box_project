#!/usr/bin/env python3
"""Простой тест non-streaming режима с реальным prompt"""

import json
import os
import time
from pathlib import Path
from openai import OpenAI
from httpx import Timeout

# API ключ
api_key = os.getenv("DEEPSEEK_API_KEY", "sk-a9bb742844e344658e5987c0ccaeae07")

# Загружаем РЕАЛЬНЫЙ prompt
prompt_file = Path("src/rob_box_voice/prompts/master_prompt_simple.txt")
with open(prompt_file, "r", encoding="utf-8") as f:
    system_prompt = f.read()

print(f"✅ Загружен prompt: {len(system_prompt)} байт\n")

# Тестируем разные варианты timeout
configs = [
    ("Вариант 1: timeout=20.0 (простой float)", 20.0),
    ("Вариант 2: Timeout с default", Timeout(timeout=20.0)),
    ("Вариант 3: Timeout со всеми параметрами", Timeout(connect=5.0, read=20.0, write=5.0, pool=5.0)),
    ("Вариант 4: None (без timeout)", None),
]

user_query = "расскажи анекдот"
messages = [
    {"role": "system", "content": system_prompt},
    {"role": "user", "content": user_query}
]

for i, (name, timeout_config) in enumerate(configs, 1):
    print("=" * 80)
    print(f"\n🧪 {name}")
    print(f"   Timeout config: {timeout_config}")
    
    try:
        client = OpenAI(
            api_key=api_key,
            base_url="https://api.deepseek.com",
            timeout=timeout_config
        )
        
        print(f"👤 User: {user_query}")
        print("🤔 Запрос к DeepSeek (non-streaming)...")
        
        start = time.time()
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=messages,
            temperature=0.7,
            max_tokens=500,
            stream=False
        )
        
        elapsed = time.time() - start
        full_response = response.choices[0].message.content
        
        print(f"✅ Ответ за {elapsed:.2f}s ({len(full_response)} символов)")
        
        # Проверяем chunks
        chunk_count = full_response.count('"chunk"')
        print(f"✅ Найдено ~{chunk_count} chunks в ответе")
        print(f"   Первые 200 символов: {full_response[:200]}...")
        
        # Если успешно - больше не тестируем
        print(f"\n🎉 УСПЕХ! {name} работает!")
        break
        
    except Exception as e:
        elapsed = time.time() - start
        print(f"❌ Ошибка после {elapsed:.2f}s: {e}")
        if i == len(configs):
            print("\n💀 Все варианты провалились!")
    
    print()
    
    # Пауза между запросами
    if i < len(configs):
        print("⏳ Ждём 3 секунды перед следующим тестом...")
        time.sleep(3)


