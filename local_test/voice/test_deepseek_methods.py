#!/usr/bin/env python3
"""Тестируем разные способы вызова DeepSeek API"""

import json
import os
import time
from pathlib import Path

# API ключ
api_key = os.getenv("DEEPSEEK_API_KEY", "sk-a9bb742844e344658e5987c0ccaeae07")

# Загружаем РЕАЛЬНЫЙ prompt
prompt_file = Path("src/rob_box_voice/prompts/master_prompt_simple.txt")
with open(prompt_file, "r", encoding="utf-8") as f:
    system_prompt = f.read()

print(f"✅ Загружен prompt: {len(system_prompt)} байт")
print(f"   (~{len(system_prompt) // 4} токенов)")
print()

user_query = "расскажи анекдот"
messages = [
    {"role": "system", "content": system_prompt},
    {"role": "user", "content": user_query}
]

# ============================================================================
# МЕТОД 1: OpenAI SDK
# ============================================================================
def test_openai_sdk():
    print("=" * 80)
    print("🧪 МЕТОД 1: OpenAI SDK (библиотека openai)")
    print("=" * 80)
    
    try:
        from openai import OpenAI
        
        client = OpenAI(
            api_key=api_key,
            base_url="https://api.deepseek.com",
            timeout=30.0
        )
        
        print("👤 User:", user_query)
        print("🤔 Запрос через OpenAI SDK...")
        
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
        print(f"   Первые 150 символов: {full_response[:150]}...")
        return True
        
    except Exception as e:
        elapsed = time.time() - start
        print(f"❌ Ошибка после {elapsed:.2f}s: {e}")
        return False


# ============================================================================
# МЕТОД 2: requests библиотека
# ============================================================================
def test_requests():
    print("\n" + "=" * 80)
    print("🧪 МЕТОД 2: Прямой HTTP запрос через requests")
    print("=" * 80)
    
    try:
        import requests
        
        url = "https://api.deepseek.com/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }
        payload = {
            "model": "deepseek-chat",
            "messages": messages,
            "temperature": 0.7,
            "max_tokens": 500,
            "stream": False
        }
        
        print("👤 User:", user_query)
        print("🤔 Запрос через requests.post()...")
        
        start = time.time()
        response = requests.post(
            url, 
            headers=headers, 
            json=payload, 
            timeout=30
        )
        elapsed = time.time() - start
        
        response.raise_for_status()
        data = response.json()
        
        full_response = data["choices"][0]["message"]["content"]
        print(f"✅ Ответ за {elapsed:.2f}s ({len(full_response)} символов)")
        print(f"   Первые 150 символов: {full_response[:150]}...")
        return True
        
    except Exception as e:
        elapsed = time.time() - start
        print(f"❌ Ошибка после {elapsed:.2f}s: {e}")
        return False


# ============================================================================
# МЕТОД 3: httpx библиотека (асинхронная)
# ============================================================================
def test_httpx():
    print("\n" + "=" * 80)
    print("🧪 МЕТОД 3: HTTP запрос через httpx (синхронный)")
    print("=" * 80)
    
    try:
        import httpx
        
        url = "https://api.deepseek.com/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }
        payload = {
            "model": "deepseek-chat",
            "messages": messages,
            "temperature": 0.7,
            "max_tokens": 500,
            "stream": False
        }
        
        print("👤 User:", user_query)
        print("🤔 Запрос через httpx.post()...")
        
        start = time.time()
        with httpx.Client(timeout=30.0) as client:
            response = client.post(url, headers=headers, json=payload)
        elapsed = time.time() - start
        
        response.raise_for_status()
        data = response.json()
        
        full_response = data["choices"][0]["message"]["content"]
        print(f"✅ Ответ за {elapsed:.2f}s ({len(full_response)} символов)")
        print(f"   Первые 150 символов: {full_response[:150]}...")
        return True
        
    except Exception as e:
        elapsed = time.time() - start
        print(f"❌ Ошибка после {elapsed:.2f}s: {e}")
        return False


# ============================================================================
# МЕТОД 4: curl через subprocess
# ============================================================================
def test_curl():
    print("\n" + "=" * 80)
    print("🧪 МЕТОД 4: curl через subprocess")
    print("=" * 80)
    
    try:
        import subprocess
        
        payload = {
            "model": "deepseek-chat",
            "messages": messages,
            "temperature": 0.7,
            "max_tokens": 500,
            "stream": False
        }
        
        print("👤 User:", user_query)
        print("🤔 Запрос через curl...")
        
        start = time.time()
        result = subprocess.run([
            'curl', '-s', '-X', 'POST',
            'https://api.deepseek.com/v1/chat/completions',
            '-H', f'Authorization: Bearer {api_key}',
            '-H', 'Content-Type: application/json',
            '-d', json.dumps(payload),
            '--max-time', '30'
        ], capture_output=True, text=True, timeout=35)
        elapsed = time.time() - start
        
        if result.returncode != 0:
            raise Exception(f"curl failed: {result.stderr}")
        
        data = json.loads(result.stdout)
        full_response = data["choices"][0]["message"]["content"]
        
        print(f"✅ Ответ за {elapsed:.2f}s ({len(full_response)} символов)")
        print(f"   Первые 150 символов: {full_response[:150]}...")
        return True
        
    except Exception as e:
        elapsed = time.time() - start
        print(f"❌ Ошибка после {elapsed:.2f}s: {e}")
        return False


# ============================================================================
# Запускаем все тесты
# ============================================================================
if __name__ == "__main__":
    print("\n🚀 Тестируем разные методы вызова DeepSeek API\n")
    
    results = []
    
    # Тест 1
    success = test_openai_sdk()
    results.append(("OpenAI SDK", success))
    if success:
        print("\n🎉 OpenAI SDK РАБОТАЕТ! Останавливаем тесты.\n")
    else:
        time.sleep(2)
        
        # Тест 2
        success = test_requests()
        results.append(("requests", success))
        if success:
            print("\n🎉 requests РАБОТАЕТ! Останавливаем тесты.\n")
        else:
            time.sleep(2)
            
            # Тест 3
            success = test_httpx()
            results.append(("httpx", success))
            if success:
                print("\n🎉 httpx РАБОТАЕТ! Останавливаем тесты.\n")
            else:
                time.sleep(2)
                
                # Тест 4
                success = test_curl()
                results.append(("curl", success))
    
    # Итоги
    print("\n" + "=" * 80)
    print("📊 ИТОГИ:")
    print("=" * 80)
    for method, success in results:
        status = "✅ РАБОТАЕТ" if success else "❌ НЕ РАБОТАЕТ"
        print(f"  {method}: {status}")
    print()
