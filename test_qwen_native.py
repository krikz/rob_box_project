#!/usr/bin/env python3
"""
Тестовый скрипт для Qwen API (нативный DashScope формат)
"""

import requests
import json
import time

API_KEY = "sk-bac6416afb4a4cb69f1a067ffa51db4b"

def test_qwen_native_api():
    """Тест DashScope нативного API"""
    
    url = "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation"
    
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json",
        "X-DashScope-SSE": "enable"
    }
    
    payload = {
        "model": "qwen-max",
        "input": {
            "messages": [
                {
                    "role": "system",
                    "content": "Ты ROBBOX - дружелюбный робот-ассистент."
                },
                {
                    "role": "user",
                    "content": "Привет! Расскажи короткую шутку."
                }
            ]
        },
        "parameters": {
            "result_format": "message",
            "incremental_output": True
        }
    }
    
    print("="*60)
    print("🚀 Тестирование DashScope Native API")
    print(f"📡 URL: {url}")
    print("="*60)
    print()
    
    try:
        response = requests.post(
            url,
            headers=headers,
            json=payload,
            stream=True,
            timeout=30
        )
        
        print(f"📊 Status Code: {response.status_code}")
        print(f"📋 Headers: {dict(response.headers)}")
        print()
        
        if response.status_code != 200:
            print(f"❌ Ошибка: {response.text}")
            return
        
        print("💬 Стриминг ответа:")
        print("-"*60)
        
        for line in response.iter_lines():
            if line:
                line_str = line.decode('utf-8')
                print(f"RAW: {line_str[:200]}")
                
                if line_str.startswith('data:'):
                    data_str = line_str[5:].strip()
                    
                    if data_str and data_str != '[DONE]':
                        try:
                            chunk = json.loads(data_str)
                            print(json.dumps(chunk, indent=2, ensure_ascii=False))
                        except:
                            pass
        
        print("-"*60)
    
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_qwen_native_api()
