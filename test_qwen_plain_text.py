#!/usr/bin/env python3
"""
Тест: Qwen отвечает plain text без JSON разметки
Используем ПОЛНЫЙ system prompt как на роботе
"""

import os
from openai import OpenAI
from httpx import Timeout
import time

API_KEY = os.getenv("QWEN_API_KEY", "sk-bac6416afb4a4cb69f1a067ffa51db4b")
API_BASE = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
MODEL = "qwen-max"

def load_master_prompt():
    """Загружаем полный master_prompt.txt как на роботе"""
    prompt_path = "/home/ros2/rob_box_project/src/rob_box_voice/prompts/master_prompt.txt"
    
    try:
        with open(prompt_path, 'r', encoding='utf-8') as f:
            prompt = f.read()
        print(f"✅ Загружен master_prompt: {len(prompt)} символов")
        return prompt
    except Exception as e:
        print(f"❌ Ошибка загрузки промпта: {e}")
        return "Ты ROBBOX - робот-ассистент."


def test_plain_text_response():
    """Тест запроса который может вернуть plain text"""
    
    print("="*70)
    print("🎤 QWEN TEST: Запрос с ПОЛНЫМ master_prompt (как на роботе)")
    print("="*70)
    
    client = OpenAI(
        api_key=API_KEY,
        base_url=API_BASE,
        timeout=Timeout(60.0, connect=10.0)
    )
    
    # Загружаем ПОЛНЫЙ промпт
    system_prompt = load_master_prompt()
    
    # Запрос: создать оригинальный рэп в стиле, а не цитировать
    messages = [
        {
            "role": "system",
            "content": system_prompt
        },
        {
            "role": "user",
            "content": "Зачитай короткий рэпчик в стиле Егора Летова про роботов и эксперименты, придумай свой текст"
        }
    ]
    
    print(f"\n📤 Запрос: {messages[-1]['content']}")
    print(f"🔍 enable_search: True")
    print("\n💬 Ответ:\n" + "-"*70)
    
    start_time = time.time()
    full_response = ""
    chunk_count = 0
    has_json = False
    
    try:
        stream = client.chat.completions.create(
            model=MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=500,
            stream=True,
            extra_body={"enable_search": True}
        )
        
        for chunk in stream:
            if chunk.choices and len(chunk.choices) > 0:
                delta = chunk.choices[0].delta
                
                if delta.content:
                    token = delta.content
                    print(token, end='', flush=True)
                    full_response += token
                    chunk_count += 1
                    
                    # Проверяем есть ли JSON
                    if '{' in token or '}' in token:
                        has_json = True
                
                finish_reason = chunk.choices[0].finish_reason
                if finish_reason:
                    print(f"\n\n🏁 Finish reason: {finish_reason}")
        
        elapsed = time.time() - start_time
        
        print("-"*70)
        print(f"\n✅ Успешно завершено!")
        print(f"📊 Статистика:")
        print(f"  Всего токенов: {chunk_count}")
        print(f"  Символов: {len(full_response)}")
        print(f"  Время: {elapsed:.2f}s")
        print(f"  Содержит JSON: {'✅ Да' if has_json else '❌ Нет (plain text!)'}")
        
        if not has_json:
            print(f"\n⚠️  PLAIN TEXT DETECTED!")
            print(f"📝 Полный текст ({len(full_response)} chars):")
            print(f"{full_response[:300]}...")
        
        return True, has_json
        
    except Exception as e:
        elapsed = time.time() - start_time
        print(f"\n\n❌ ОШИБКА после {elapsed:.2f}s: {e}")
        return False, False


if __name__ == "__main__":
    success, has_json = test_plain_text_response()
    
    print("\n\n" + "="*70)
    print("📋 ИТОГ:")
    print("="*70)
    print(f"Запрос выполнен: {'✅ OK' if success else '❌ FAIL'}")
    print(f"Формат ответа: {'JSON' if has_json else 'Plain Text'}")
    print("="*70)
