"""Провайдер ``llm_harness``: OpenAI-совместимый ``chat/completions`` со строгой JSON-схемой (ADR-0157 §4, PR-7).

Тот же порт ``DecisionProvider``, что у Jev: отвечает только на ``ChoiceQuestion``. Схема ответа —
``{"choice": enum[id вариантов], "confidence": number}`` в режиме ``json_schema``/``strict``, ``temperature`` 0:
модель физически выбирает из перечня; проверка «вне перечня» всё равно остаётся у оркестратора (сервер без
поддержки strict схему может игнорировать). Ретраев нет; транспорт и ошибки — общие с ``systemone_http``.

Конфиг только из env (ключ читается в момент вызова и уходит лишь в ``Authorization``):
``LLM_HARNESS_BASE_URL`` (например ``https://api.deepseek.com``), ``LLM_HARNESS_MODEL``, ``LLM_HARNESS_API_KEY``.
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Callable, Mapping

from .systemone_http import ClientFactory, _default_client_factory, http_json
from .types import (
    ChoiceAnswer,
    ChoiceQuestion,
    Decision,
    ProviderConfigError,
    ProviderInvalidResponse,
    Question,
    State,
)

NAME = "llm_harness"
BASE_URL_ENV = "LLM_HARNESS_BASE_URL"
MODEL_ENV = "LLM_HARNESS_MODEL"
API_KEY_ENV = "LLM_HARNESS_API_KEY"
CHAT_PATH = "/chat/completions"
SYSTEM_PROMPT = ("Ты выбираешь ровно один вариант из закрытого перечня по описанию заказа. Отвечай только JSON "
                 "по схеме: id варианта из перечня и уверенность 0..1.")


class ChatChoiceProvider:
    """``chat/completions`` → ``ChoiceAnswer``; ``usage`` приводится к ``input_tokens``/``output_tokens``."""

    name = NAME

    def __init__(self, *, base_url: str, model: str, api_key_env: str = API_KEY_ENV, timeout_s: float = 2.0,
                 client_factory: ClientFactory = _default_client_factory,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.base_url = base_url.rstrip("/")
        self.model = model
        self._api_key_env = api_key_env
        self._timeout_s = timeout_s
        self._client_factory = client_factory
        self._clock = clock

    async def decide(self, state: State, questions: Mapping[str, Question]) -> Decision:
        started = self._clock()
        answers: dict[str, Any] = {}
        usage = {"input_tokens": 0, "output_tokens": 0}
        for qid, question in questions.items():
            if not isinstance(question, ChoiceQuestion):
                raise ProviderInvalidResponse(f"{NAME}: only choice questions are supported")
            payload = await http_json(NAME, self._client_factory, self._timeout_s, "POST",
                                      self.base_url + CHAT_PATH, build_request(self.model, state, question),
                                      self._headers())
            answers[qid] = parse_answer(payload)
            used = payload.get("usage") if isinstance(payload.get("usage"), Mapping) else {}
            usage["input_tokens"] += int(used.get("prompt_tokens") or 0)
            usage["output_tokens"] += int(used.get("completion_tokens") or 0)
        return Decision(answers=answers, provider=NAME, model=self.model,
                        latency_ms=(self._clock() - started) * 1000.0, usage=usage)

    def _headers(self) -> dict[str, str]:
        key = os.environ.get(self._api_key_env, "").strip()
        if not key:
            raise ProviderConfigError(f"{NAME}: env {self._api_key_env} is not set")
        return {"Accept": "application/json", "Authorization": f"Bearer {key}"}


def chat_choice_from_env(env: Mapping[str, str] | None = None, **kwargs: Any) -> ChatChoiceProvider | None:
    """Провайдер, если заданы base_url, модель и ключ; иначе ``None`` (провайдер опционален)."""
    env = os.environ if env is None else env
    base, model = env.get(BASE_URL_ENV, "").strip(), env.get(MODEL_ENV, "").strip()
    if not (base and model and env.get(API_KEY_ENV, "").strip()):
        return None
    return ChatChoiceProvider(base_url=base, model=model, **kwargs)


def build_request(model: str, state: State, question: ChoiceQuestion) -> dict[str, Any]:
    """Тело запроса: перечень с описаниями в сообщении, ``enum`` id в строгой схеме, ``temperature`` 0."""
    options = "\n".join(f"- {o}: {question.descriptions[o]}" if question.descriptions.get(o) else f"- {o}"
                        for o in question.options)
    wire_state = state if isinstance(state, str) else json.dumps(state, ensure_ascii=False, default=list)
    schema = {"type": "object", "additionalProperties": False, "required": ["choice", "confidence"],
              "properties": {"choice": {"type": "string", "enum": list(question.options)},
                             "confidence": {"type": "number", "minimum": 0, "maximum": 1}}}
    return {"model": model, "temperature": 0,
            "messages": [{"role": "system", "content": SYSTEM_PROMPT},
                         {"role": "user",
                          "content": f"{question.text}\n\nСостояние: {wire_state}\n\nВарианты:\n{options}"}],
            "response_format": {"type": "json_schema",
                                "json_schema": {"name": "choice", "strict": True, "schema": schema}}}


def parse_answer(payload: Mapping[str, Any]) -> ChoiceAnswer:
    try:
        raw = json.loads(payload["choices"][0]["message"]["content"])
        choice, confidence = str(raw["choice"]), float(raw["confidence"])
    except (KeyError, IndexError, TypeError, ValueError, AttributeError):
        raise ProviderInvalidResponse(f"{NAME}: malformed completion") from None
    return ChoiceAnswer(choice, {choice: 1.0}, confidence)
