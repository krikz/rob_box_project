"""MiniMax-M3 по умолчанию: на провод уходит ровно то, что уходило ДО адаптера под Flash.

Решение Шифу 20.10: остаёмся на MiniMax-M3, код под M3.1-Flash сохраняем на будущее
(PR #3668). Значит, всё, что робот отправляет в API и что провайдер говорит о
моделях, при ``DEFAULT_MODEL = "MiniMax-M3"`` обязано остаться побайтно прежним.

``GOLDEN`` снят с кода ``origin/develop`` ДО PR (``ad2235406``): тот же набор
сценариев прогнан на develop, вывод вклеен сюда как есть. Этот файл проходит и
на develop (до PR), и на ветке PR — это и есть доказательство «ни на бит».
Сценарии: голосовой ход (complete/stream, настройки как в yaml), настройки по
умолчанию, ход с рассуждением (``TURN_REASONING``), запрос с картинкой,
``tool_choice="required"`` (путь DJ), ``extra={"thinking": None}``, чужие поля в
``extra``, и ``capabilities_for`` по именам моделей.

Намеренное отличие от develop одно и в этот снимок НЕ входит: ``image_input`` теперь
по явному списку проверенных моделей, а не по подстроке ``"minimax-m3"`` — так что
``MiniMax-M3-chat`` и будущие ``MiniMax-M3.*`` без пробы картинки его больше не
получают (в репозитории таких имён нет ни в одном конфиге).
"""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace
from typing import Any, Awaitable, Callable

import pytest

from rob_box_harness.providers.minimax import HarnessMiniMaxProvider, RetryPolicy
from rob_box_harness.providers.reasoning import TURN_REASONING
from rob_box_llm.provider import ImagePart, LLMMessage, LLMSettings, TextPart

TOOL = {
    "type": "function",
    "function": {"name": "set_voice", "parameters": {"type": "object", "properties": {"voice": {"type": "string"}}}},
}
VOICE = LLMSettings(temperature=0.7, max_tokens=3000)
JPEG = bytes([0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10]) + b"JFIF" + bytes([0x00, 0x01])
MODELS = [None, "MiniMax-M3", "MiniMax-M2.7", "MiniMax-M2-vision", "minimax-vision-x", "some-other-model"]


class _Completions:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        if kwargs.get("stream"):
            return self._stream()
        message = SimpleNamespace(content="ок", tool_calls=None)
        return SimpleNamespace(
            choices=[SimpleNamespace(message=message, finish_reason="stop")], usage=None, base_resp={}
        )

    async def _stream(self) -> Any:
        delta = SimpleNamespace(content="ок", tool_calls=None)
        yield SimpleNamespace(choices=[SimpleNamespace(delta=delta, finish_reason=None)], usage=None, base_resp={})
        delta = SimpleNamespace(content=None, tool_calls=None)
        yield SimpleNamespace(choices=[SimpleNamespace(delta=delta, finish_reason="stop")], usage=None, base_resp={})


def _provider() -> tuple[HarnessMiniMaxProvider, _Completions]:
    completions = _Completions()
    client = SimpleNamespace(chat=SimpleNamespace(completions=completions), is_closed=False)
    provider = HarnessMiniMaxProvider(
        api_key="sk-test",
        client=client,  # type: ignore[arg-type]
        first_chunk_timeout_s=10.0,
        retry=RetryPolicy(max_attempts=1),
    )
    return provider, completions


def _turn(user: Any = "привет") -> list[LLMMessage]:
    return [LLMMessage(role="system", content="ты робот"), LLMMessage(role="user", content=user)]


async def _stream_all(provider: HarnessMiniMaxProvider, messages: list[LLMMessage], **kwargs: Any) -> list[Any]:
    return [chunk async for chunk in provider.stream(messages, **kwargs)]


SCENARIOS: dict[str, tuple[Callable[[HarnessMiniMaxProvider], Awaitable[Any]], bool]] = {
    "voice_complete": (lambda p: p.complete(_turn(), settings=VOICE), False),
    "voice_complete_default_settings": (lambda p: p.complete(_turn()), False),
    "voice_stream_with_tools": (lambda p: _stream_all(p, _turn(), tools=[TOOL], settings=VOICE), False),
    "reasoning_complete": (lambda p: p.complete(_turn(), settings=VOICE), True),
    "reasoning_stream": (lambda p: _stream_all(p, _turn(), settings=VOICE), True),
    "image_complete": (
        lambda p: p.complete(
            _turn((TextPart("Что на картинке?"), ImagePart(source=JPEG, media_type="image/jpeg"))), settings=VOICE
        ),
        False,
    ),
    "tool_choice_required": (
        lambda p: p.complete(_turn(), tools=[TOOL], settings=LLMSettings(max_tokens=500, tool_choice="required")),
        False,
    ),
    "caller_thinking_none": (lambda p: p.complete(_turn(), settings=LLMSettings(extra={"thinking": None})), False),
    "caller_extra_top_k": (lambda p: p.complete(_turn(), settings=LLMSettings(extra={"top_k": 5})), False),
}


def _first_request(name: str) -> dict[str, Any]:
    run, reasoning = SCENARIOS[name]
    provider, completions = _provider()

    async def go() -> None:
        token = TURN_REASONING.set(reasoning)
        try:
            await run(provider)
        finally:
            TURN_REASONING.reset(token)

    asyncio.run(go())
    return json.loads(json.dumps(completions.calls[0], default=str))


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_request_sent_by_default_is_byte_for_byte_what_develop_sent(name: str) -> None:
    assert _first_request(name) == GOLDEN[name]


def test_default_instance_reports_the_same_capabilities() -> None:
    provider, _ = _provider()

    assert {str(m): provider.capabilities_for(m).image_input for m in MODELS} == GOLDEN["capabilities_for"]
    assert {
        k: getattr(provider.capabilities, k)
        for k in ("text", "streaming_text", "tools", "streaming_tools", "image_input")
    } == GOLDEN["default_capabilities"]
    assert provider._inner._default_model == GOLDEN["inner_default_model"] == "MiniMax-M3"


def test_picture_request_keeps_thinking_disabled() -> None:
    """Прямо то, что просил Шифу проверить: на картинку по-прежнему уходит ``thinking: disabled``."""
    sent = _first_request("image_complete")

    assert sent["extra_body"] == {"thinking": {"type": "disabled"}}
    assert [part["type"] for part in sent["messages"][1]["content"]] == ["text", "image_url"]


GOLDEN: dict[str, Any] = json.loads(
    r"""{
 "caller_extra_top_k": {
  "extra_body": {
   "thinking": {
    "type": "disabled"
   },
   "top_k": 5
  },
  "max_tokens": 4096,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": "привет",
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": false
 },
 "caller_thinking_none": {
  "max_tokens": 4096,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": "привет",
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": false
 },
 "capabilities_for": {
  "MiniMax-M2-vision": true,
  "MiniMax-M2.7": false,
  "MiniMax-M3": true,
  "None": true,
  "minimax-vision-x": true,
  "some-other-model": false
 },
 "default_capabilities": {
  "image_input": true,
  "streaming_text": true,
  "streaming_tools": true,
  "text": true,
  "tools": true
 },
 "image_complete": {
  "extra_body": {
   "thinking": {
    "type": "disabled"
   }
  },
  "max_tokens": 3000,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": [
     {
      "text": "Что на картинке?",
      "type": "text"
     },
     {
      "image_url": {
       "url": "data:image/jpeg;base64,/9j/4AAQSkZJRgAB"
      },
      "type": "image_url"
     }
    ],
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": false,
  "temperature": 0.7
 },
 "inner_default_model": "MiniMax-M3",
 "reasoning_complete": {
  "max_tokens": 7096,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": "привет",
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": false,
  "temperature": 0.7
 },
 "reasoning_stream": {
  "max_tokens": 7096,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": "привет",
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": true,
  "stream_options": {
   "include_usage": true
  },
  "temperature": 0.7
 },
 "tool_choice_required": {
  "extra_body": {
   "thinking": {
    "type": "disabled"
   }
  },
  "max_tokens": 500,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": "привет",
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": false,
  "tool_choice": "required",
  "tools": [
   {
    "function": {
     "name": "set_voice",
     "parameters": {
      "properties": {
       "voice": {
        "type": "string"
       }
      },
      "type": "object"
     }
    },
    "type": "function"
   }
  ]
 },
 "voice_complete": {
  "extra_body": {
   "thinking": {
    "type": "disabled"
   }
  },
  "max_tokens": 3000,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": "привет",
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": false,
  "temperature": 0.7
 },
 "voice_complete_default_settings": {
  "extra_body": {
   "thinking": {
    "type": "disabled"
   }
  },
  "max_tokens": 4096,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": "привет",
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": false
 },
 "voice_stream_with_tools": {
  "extra_body": {
   "thinking": {
    "type": "disabled"
   }
  },
  "max_tokens": 3000,
  "messages": [
   {
    "content": "ты робот",
    "role": "system"
   },
   {
    "content": "привет",
    "role": "user"
   }
  ],
  "model": "MiniMax-M3",
  "stream": true,
  "stream_options": {
   "include_usage": true
  },
  "temperature": 0.7,
  "tools": [
   {
    "function": {
     "name": "set_voice",
     "parameters": {
      "properties": {
       "voice": {
        "type": "string"
       }
      },
      "type": "object"
     }
    },
    "type": "function"
   }
  ]
 }
}"""
)
