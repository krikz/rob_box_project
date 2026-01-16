import json, time, requests

api_key = "sk-a9bb742844e344658e5987c0ccaeae07"
url = "https://api.deepseek.com/v1/chat/completions"

# КОРОТКИЙ промпт
messages = [
    {"role": "system", "content": "Ты робот. Отвечай JSON chunks: {\"chunk\": N, \"ssml\": \"<speak>текст</speak>\"}"},
    {"role": "user", "content": "расскажи анекдот"}
]

print("🤔 Запрос с КОРОТКИМ промптом...")
start = time.time()
r = requests.post(url, headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}, json={"model": "deepseek-chat", "messages": messages, "max_tokens": 500, "stream": False}, timeout=30)
elapsed = time.time() - start
print(f"✅ Ответ за {elapsed:.2f}s: {r.json()['choices'][0]['message']['content'][:200]}...")
