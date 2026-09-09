"""
mock_llm/server.py — Имитатор OpenAI API для тестирования dialogue_node.

Реализует:
  POST /v1/chat/completions  (streaming SSE, совместимо с openai python SDK)
  GET  /health               (healthcheck)
  POST /admin/set_responses  (задать скриптованные ответы для сценария)

Логика ответов (в порядке приоритета):
  1. Если задан scripted_queue — возвращает следующий ответ из очереди
  2. Иначе — ищет совпадение в rules (scenarios.yaml: rules[].trigger → response)
  3. Иначе — default_response

Формат ответа (поддерживается dialogue_node):
  Если SSML_MODE=true  → {"ssml": "<speak>...</speak>", "emotion": "neutral"}
  Иначе               → plain text (dialogue_node обернёт сам)
"""

import json
import os
import time
import uuid
from typing import AsyncIterator, Optional

import uvicorn
import yaml
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import StreamingResponse

app = FastAPI(title="Mock LLM for Rob Box Tests")

# ── Загрузка сценариев ────────────────────────────────────────────────────────

_scenarios_path = os.getenv("MOCK_LLM_SCENARIOS", "/scenarios/scenarios.yaml")
_rules: list[dict] = []
_default_response = "Я тестовый робот. Вопрос получен."

def _load_scenarios():
    global _rules, _default_response
    if not os.path.exists(_scenarios_path):
        print(f"[mock-llm] scenarios file not found: {_scenarios_path}, using defaults")
        return
    with open(_scenarios_path) as f:
        data = yaml.safe_load(f)
    _rules = data.get("llm_rules", [])
    _default_response = data.get("default_response", _default_response)
    print(f"[mock-llm] Loaded {len(_rules)} rules from {_scenarios_path}")

_load_scenarios()

# ── Scripted queue (для последовательных сценариев) ──────────────────────────
_scripted_queue: list[str] = []


def _pick_response(messages: list[dict]) -> str:
    """Выбрать ответ по сообщениям."""
    if _scripted_queue:
        return _scripted_queue.pop(0)

    # Последнее user-сообщение
    user_text = ""
    for m in reversed(messages):
        if m.get("role") == "user":
            user_text = (m.get("content") or "").lower()
            break

    for rule in _rules:
        trigger = str(rule.get("trigger", "")).lower()
        if trigger and trigger in user_text:
            return rule["response"]

    return _default_response


# ── SSE streaming helpers ─────────────────────────────────────────────────────

def _chunk_event(delta_content: str, finish_reason: Optional[str] = None) -> str:
    """Сформировать одну SSE строку как у OpenAI."""
    chunk = {
        "id": f"chatcmpl-test-{uuid.uuid4().hex[:8]}",
        "object": "chat.completion.chunk",
        "created": int(time.time()),
        "model": "mock-model",
        "choices": [{
            "index": 0,
            "delta": {"content": delta_content} if delta_content else {},
            "finish_reason": finish_reason,
        }],
    }
    if finish_reason:
        chunk["usage"] = {
            "prompt_tokens": 50,
            "completion_tokens": len(delta_content.split()),
            "total_tokens": 50 + len(delta_content.split()),
        }
    return f"data: {json.dumps(chunk)}\n\n"


async def _stream_response(text: str) -> AsyncIterator[str]:
    """Стримить ответ по токенам (имитация реального API)."""
    # Посылаем по словам с небольшой задержкой
    words = text.split(" ")
    for i, word in enumerate(words):
        token = word if i == 0 else " " + word
        yield _chunk_event(token)
        # Небольшая задержка для имитации реального стриминга
        # time.sleep(0.02) — закомментировано, чтобы тесты были быстрыми

    yield _chunk_event("", finish_reason="stop")
    yield "data: [DONE]\n\n"


# ── Endpoints ─────────────────────────────────────────────────────────────────

@app.get("/health")
async def health():
    return {"status": "ok", "rules": len(_rules)}


@app.post("/admin/set_responses")
async def set_responses(body: dict):
    """Задать очередь ответов для следующего сценария."""
    global _scripted_queue
    responses = body.get("responses", [])
    _scripted_queue = list(responses)
    return {"queued": len(_scripted_queue)}


@app.post("/admin/reload_scenarios")
async def reload_scenarios():
    _load_scenarios()
    return {"rules": len(_rules)}


@app.post("/v1/chat/completions")
async def chat_completions(request: Request):
    """OpenAI-compatible chat completions (streaming only для dialogue_node)."""
    body = await request.json()
    messages = body.get("messages", [])
    stream = body.get("stream", False)

    # Проверка на tool_calls request — возвращаем plain text (не tool_calls)
    # В тестах мы можем отдельно задать tool_calls ответ через scripted_queue
    response_text = _pick_response(messages)

    print(f"[mock-llm] Request: last_user={_last_user(messages)[:60]!r} → {response_text[:60]!r}")

    if stream:
        return StreamingResponse(
            _stream_response(response_text),
            media_type="text/event-stream",
        )
    else:
        # Non-streaming fallback
        return {
            "id": f"chatcmpl-test-{uuid.uuid4().hex[:8]}",
            "object": "chat.completion",
            "created": int(time.time()),
            "model": "mock-model",
            "choices": [{
                "index": 0,
                "message": {"role": "assistant", "content": response_text},
                "finish_reason": "stop",
            }],
            "usage": {"prompt_tokens": 50, "completion_tokens": 10, "total_tokens": 60},
        }


def _last_user(messages: list) -> str:
    for m in reversed(messages):
        if m.get("role") == "user":
            return m.get("content") or ""
    return ""


if __name__ == "__main__":
    port = int(os.getenv("MOCK_LLM_PORT", "8765"))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
