#!/usr/bin/env python3
"""vision_llm.py — вызов облачной vision-LLM по снимку лица (ADR-0164 §4.1, этап 6).

OpenAI-совместимый ``/chat/completions``: текст запроса и JPEG как ``image_url`` (data URL), ответ —
JSON-объект (``response_format=json_object``). Без сторонних зависимостей (urllib), чтобы образ
``vision-hailo`` не тянул SDK.

Включается только явной настройкой окружения контейнера ``vision-face``:

* ``FACE_PERSONA_VLM_API_KEY`` — ключ (пусто → облака нет, лицо получает словарную кличку);
* ``FACE_PERSONA_VLM_BASE_URL`` — адрес OpenAI-совместимого API;
* ``FACE_PERSONA_VLM_MODEL`` — модель с поддержкой изображений;
* ``FACE_PERSONA_VLM_MAX_PER_HOUR`` — потолок вызовов в час (пусто → ``persona.VISION_CALLS_PER_HOUR``).

Ключ в лог не пишется.
"""

from __future__ import annotations

import base64
import json
import urllib.request
from typing import Any, Callable, Dict, Mapping, Optional, Tuple

from rob_box_core import persona

ENV_API_KEY = 'FACE_PERSONA_VLM_API_KEY'
ENV_BASE_URL = 'FACE_PERSONA_VLM_BASE_URL'
ENV_MODEL = 'FACE_PERSONA_VLM_MODEL'
ENV_MAX_PER_HOUR = 'FACE_PERSONA_VLM_MAX_PER_HOUR'

DEFAULT_TIMEOUT_SEC = 20.0
MAX_TOKENS = 400


def make_describer(
    base_url: str,
    api_key: str,
    model: str,
    *,
    timeout: float = DEFAULT_TIMEOUT_SEC,
    opener: Callable[..., Any] = urllib.request.urlopen,
) -> Callable[[bytes, str, Dict[str, Any]], str]:
    """Вернуть ``describer(jpeg, prompt, schema) -> текст ответа модели``. Ошибки сети — исключением."""
    url = base_url.rstrip('/') + '/chat/completions'

    def describe(jpeg: bytes, prompt: str, schema: Dict[str, Any]) -> str:
        image = 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode('ascii')
        body = {
            'model': model,
            'temperature': 0.3,
            'max_tokens': MAX_TOKENS,
            'response_format': {'type': 'json_object'},
            'messages': [{
                'role': 'user',
                'content': [
                    {'type': 'text', 'text': prompt + '\nСхема ответа: '
                     + json.dumps(schema, ensure_ascii=False)},
                    {'type': 'image_url', 'image_url': {'url': image}},
                ],
            }],
        }
        request = urllib.request.Request(
            url, data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
            headers={'Content-Type': 'application/json', 'Authorization': f'Bearer {api_key}'},
            method='POST',
        )
        with opener(request, timeout=timeout) as resp:
            payload = json.loads(resp.read().decode('utf-8'))
        return str(payload['choices'][0]['message']['content'] or '')

    return describe


def describer_from_env(
    env: Mapping[str, str],
    *,
    opener: Callable[..., Any] = urllib.request.urlopen,
) -> Tuple[Optional[Callable[[bytes, str, Dict[str, Any]], str]], str]:
    """``(describer, пояснение)``; без ключа, адреса или модели — ``(None, почему)`` для честного лога."""
    key = (env.get(ENV_API_KEY) or '').strip()
    base = (env.get(ENV_BASE_URL) or '').strip()
    model = (env.get(ENV_MODEL) or '').strip()
    missing = [name for name, value in ((ENV_API_KEY, key), (ENV_BASE_URL, base), (ENV_MODEL, model))
               if not value]
    if missing:
        return None, f'vision-LLM не настроена (пусто: {", ".join(missing)}) — только словарные клички'
    return make_describer(base, key, model, opener=opener), f'vision-LLM: {model} @ {base}'


def budget_from_env(env: Mapping[str, str]) -> persona.CallBudget:
    """Потолок вызовов в час из окружения; пусто или мусор — значение по умолчанию из ``persona``."""
    raw = (env.get(ENV_MAX_PER_HOUR) or '').strip()
    try:
        per_hour = int(raw) if raw else persona.VISION_CALLS_PER_HOUR
    except ValueError:
        per_hour = persona.VISION_CALLS_PER_HOUR
    return persona.CallBudget(per_window=max(0, per_hour))


__all__ = [
    'ENV_API_KEY',
    'ENV_BASE_URL',
    'ENV_MAX_PER_HOUR',
    'ENV_MODEL',
    'budget_from_env',
    'describer_from_env',
    'make_describer',
]
