#!/usr/bin/env python3
"""
Тест Qwen API с enable_search параметром
"""

import os
from openai import OpenAI
from httpx import Timeout
import time

# API Configuration
API_KEY = os.getenv("QWEN_API_KEY", "sk-bac6416afb4a4cb69f1a067ffa51db4b")
API_BASE = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
MODEL = "qwen-max"

def test_with_enable_search():
    """Тест с enable_search=True"""
    
    print("="*70)
    print("🔍 QWEN API TEST: enable_search=True")
    print("="*70)
    
    client = OpenAI(
        api_key=API_KEY,
        base_url=API_BASE,
        timeout=Timeout(60.0, connect=10.0)
    )
    
    messages = [
        {
            "role": "system",
            "content": "Ты ROBBOX - робот-ассистент. Отвечай кратко."
        },
        {
            "role": "user",
            "content": "Найди информацию: сколько стоит биткоин сейчас?"
        }
    ]
    
    print(f"\n📤 Запрос: {messages[-1]['content']}")
    print(f"⏰ Timeout: 60 секунд")
    print(f"🔍 enable_search: True")
    print("\n💬 Ответ:\n" + "-"*70)
    
    start_time = time.time()
    chunk_count = 0
    full_response = ""
    first_chunk_time = None
    
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
                    if first_chunk_time is None:
                        first_chunk_time = time.time()
                        ttfb = first_chunk_time - start_time
                        print(f"⚡ TTFB: {ttfb:.2f}s\n")
                    
                    print(delta.content, end='', flush=True)
                    full_response += delta.content
                    chunk_count += 1
                
                finish_reason = chunk.choices[0].finish_reason
                if finish_reason:
                    print(f"\n\n🏁 Finish reason: {finish_reason}")
        
        elapsed = time.time() - start_time
        
        print("-"*70)
        print(f"\n✅ Успешно завершено!")
        print(f"📊 Статистика:")
        print(f"  Всего чанков: {chunk_count}")
        print(f"  Символов: {len(full_response)}")
        print(f"  Время: {elapsed:.2f}s")
        if first_chunk_time:
            print(f"  TTFB: {first_chunk_time - start_time:.2f}s")
        
        return True
        
    except Exception as e:
        elapsed = time.time() - start_time
        print(f"\n\n❌ ОШИБКА после {elapsed:.2f}s: {e}")
        return False


def test_without_enable_search():
    """Тест БЕЗ enable_search для сравнения"""
    
    print("\n\n" + "="*70)
    print("🚀 QWEN API TEST: enable_search=False (для сравнения)")
    print("="*70)
    
    client = OpenAI(
        api_key=API_KEY,
        base_url=API_BASE,
        timeout=Timeout(60.0, connect=10.0)
    )
    
    messages = [
        {
            "role": "system",
            "content": "Ты ROBBOX - робот-ассистент. Отвечай кратко."
        },
        {
            "role": "user",
            "content": "Расскажи шутку про биткоин"
        }
    ]
    
    print(f"\n📤 Запрос: {messages[-1]['content']}")
    print(f"⏰ Timeout: 60 секунд")
    print(f"🔍 enable_search: False")
    print("\n💬 Ответ:\n" + "-"*70)
    
    start_time = time.time()
    chunk_count = 0
    full_response = ""
    first_chunk_time = None
    
    try:
        stream = client.chat.completions.create(
            model=MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=500,
            stream=True,
            extra_body={}  # БЕЗ enable_search
        )
        
        for chunk in stream:
            if chunk.choices and len(chunk.choices) > 0:
                delta = chunk.choices[0].delta
                
                if delta.content:
                    if first_chunk_time is None:
                        first_chunk_time = time.time()
                        ttfb = first_chunk_time - start_time
                        print(f"⚡ TTFB: {ttfb:.2f}s\n")
                    
                    print(delta.content, end='', flush=True)
                    full_response += delta.content
                    chunk_count += 1
                
                finish_reason = chunk.choices[0].finish_reason
                if finish_reason:
                    print(f"\n\n🏁 Finish reason: {finish_reason}")
        
        elapsed = time.time() - start_time
        
        print("-"*70)
        print(f"\n✅ Успешно завершено!")
        print(f"📊 Статистика:")
        print(f"  Всего чанков: {chunk_count}")
        print(f"  Символов: {len(full_response)}")
        print(f"  Время: {elapsed:.2f}s")
        if first_chunk_time:
            print(f"  TTFB: {first_chunk_time - start_time:.2f}s")
        
        return True
        
    except Exception as e:
        elapsed = time.time() - start_time
        print(f"\n\n❌ ОШИБКА после {elapsed:.2f}s: {e}")
        return False


if __name__ == "__main__":
    # Тест с enable_search
    result1 = test_with_enable_search()
    
    # Пауза между тестами
    time.sleep(2)
    
    # Тест без enable_search
    result2 = test_without_enable_search()
    
    print("\n\n" + "="*70)
    print("📋 ИТОГИ:")
    print("="*70)
    print(f"enable_search=True:  {'✅ OK' if result1 else '❌ FAIL'}")
    print(f"enable_search=False: {'✅ OK' if result2 else '❌ FAIL'}")
    print("="*70)
