#!/usr/bin/env python3
import json
import os
import time
from pathlib import Path

from openai import OpenAI

api_key = os.getenv("DEEPSEEK_API_KEY")
if not api_key:
    raise SystemExit("Set DEEPSEEK_API_KEY before running this script.")

# Загружаем БОЛЬШОЙ prompt
prompt_file = Path("src/rob_box_voice/prompts/master_prompt_simple.txt")
with open(prompt_file, "r", encoding="utf-8") as f:
    system_prompt = f.read()

print(f"✅ Prompt: {len(system_prompt)} байт (~{len(system_prompt)//4} токенов)")
print("🔑 API Key loaded from DEEPSEEK_API_KEY\n")

client = OpenAI(api_key=api_key, base_url="https://api.deepseek.com", timeout=30.0)

messages = [
    {"role": "system", "content": system_prompt},
    {"role": "user", "content": "расскажи анекдот"}
]

print("=" * 80)
print("🧪 ТЕСТ: Non-streaming с БОЛЬШИМ промптом и новым ключом")
print("=" * 80)
print("🤔 Запрос к DeepSeek...")

start = time.time()
try:
    response = client.chat.completions.create(
        model="deepseek-chat",
        messages=messages,
        temperature=0.7,
        max_tokens=500,
        stream=False
    )
    elapsed = time.time() - start
    answer = response.choices[0].message.content
    
    print(f"✅ Ответ за {elapsed:.2f}s ({len(answer)} символов)")
    print(f"\n{answer[:300]}...")
    
except Exception as e:
    elapsed = time.time() - start
    print(f"❌ Ошибка после {elapsed:.2f}s: {e}")
