"""Провайдер ``llm_harness``: OpenAI-совместимый ``chat/completions`` со строгой JSON-схемой (ADR-0157 §4, PR-7).

Тот же порт ``DecisionProvider``, что у Jev: отвечает только на ``ChoiceQuestion``. Схема ответа —
``{"choice": enum[id вариантов], "confidence": number}`` в режиме ``json_schema``/``strict``, ``temperature`` 0:
модель физически выбирает из перечня; проверка «вне перечня» всё равно остаётся у оркестратора (сервер без
поддержки strict схему может игнорировать). Ретраев нет; транспорт и ошибки — общие с ``systemone_http``.

Конфиг только из env (ключ читается в момент вызова и уходит лишь в ``Authorization``, в лог не пишется):
``LLM_HARNESS_PROVIDER`` — имя из ``providers.catalog.LLM_PROVIDER_REGISTRY`` (по умолчанию ``deepseek``: дефолты
``base_url``/модели и имя переменной ключа — оттуда же, где их берёт диалог робота); ``LLM_HARNESS_BASE_URL``,
``LLM_HARNESS_MODEL`` — переопределения; ключ — ``LLM_HARNESS_API_KEY`` или переменная, названная в
``LLM_HARNESS_API_KEY_ENV`` (например ``DEEPSEEK_API_KEY`` — новый секрет не нужен), иначе ``env_key_var`` провайдера.

Формат ответа (#3564, замер с робота 13.10): DeepSeek на ``json_schema`` отвечает 400 «This response_format type is
unavailable now», на ``json_object`` — 200 за ≈0.8 с; провайдерам из :data:`JSON_OBJECT_ONLY` схема с ``enum`` уходит
текстом в системное сообщение, а «вне перечня» отсекает оркестратор. MiniMax-M3 (основной LLM диалога) пишет
``<think>…</think>`` перед JSON — для выбора не годится, поэтому по умолчанию DeepSeek (запасной LLM диалога).
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
#: Имя переменной окружения, в которой лежит ключ (косвенность: ключ уже есть в env под своим именем).
API_KEY_ENV_ENV = "LLM_HARNESS_API_KEY_ENV"
PROVIDER_ENV = "LLM_HARNESS_PROVIDER"
#: Провайдер по умолчанию — запасной LLM диалога робота (``dialogue_node.yaml``: ``llm_providers: minimax,deepseek``).
DEFAULT_PROVIDER = "deepseek"
JSON_SCHEMA, JSON_OBJECT = "json_schema", "json_object"
#: Провайдеры без ``response_format=json_schema`` (замер 13.10: DeepSeek → 400).
JSON_OBJECT_ONLY = frozenset({"deepseek"})
CHAT_PATH = "/chat/completions"
SYSTEM_PROMPT = ("Ты выбираешь ровно один вариант из закрытого перечня по описанию заказа. Отвечай только JSON "
                 "по схеме: id варианта из перечня и уверенность 0..1.")


class ChatChoiceProvider:
    """``chat/completions`` → ``ChoiceAnswer``; ``usage`` приводится к ``input_tokens``/``output_tokens``."""

    name = NAME

    def __init__(self, *, base_url: str, model: str, api_key_env: str = API_KEY_ENV, timeout_s: float = 2.0,
                 response_format: str = JSON_SCHEMA, client_factory: ClientFactory = _default_client_factory,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if response_format not in (JSON_SCHEMA, JSON_OBJECT):
            raise ProviderConfigError(f"{NAME}: response_format {response_format!r} not in {JSON_SCHEMA}/{JSON_OBJECT}")
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.response_format = response_format
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
                                      self.base_url + CHAT_PATH,
                                      build_request(self.model, state, question, self.response_format),
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
    """Провайдер по env (см. модуль): ``base_url``/модель/ключ — переопределения или дефолты провайдера из каталога;
    нет ключа (или неизвестный провайдер без явных base_url/модели) — ``None`` (провайдер опционален)."""
    env = os.environ if env is None else env
    kind = env.get(PROVIDER_ENV, "").strip().lower() or DEFAULT_PROVIDER
    preset = _preset(kind)
    base = env.get(BASE_URL_ENV, "").strip() or preset.get("default_base_url", "")
    model = env.get(MODEL_ENV, "").strip() or preset.get("default_model", "")
    key_env = (env.get(API_KEY_ENV_ENV, "").strip() or (API_KEY_ENV if env.get(API_KEY_ENV, "").strip() else "")
               or preset.get("env_key_var", ""))
    if not (base and model and key_env and env.get(key_env, "").strip()):
        return None
    return ChatChoiceProvider(base_url=base, model=model, api_key_env=key_env,
                              response_format=JSON_OBJECT if kind in JSON_OBJECT_ONLY else JSON_SCHEMA, **kwargs)


def _preset(kind: str) -> Mapping[str, Any]:
    """Дефолты провайдера — из одного каталога с диалогом (импорт ленивый: каталог тянет SDK провайдеров)."""
    from rob_box_harness.providers.catalog import LLM_PROVIDER_REGISTRY

    return LLM_PROVIDER_REGISTRY.get(kind, {})


def build_request(model: str, state: State, question: ChoiceQuestion,
                  response_format: str = JSON_SCHEMA) -> dict[str, Any]:
    """Тело запроса: перечень с описаниями в сообщении, ``enum`` id в строгой схеме, ``temperature`` 0; без
    ``json_schema`` у сервера — ``json_object`` и та же схема текстом в системном сообщении."""
    options = "\n".join(f"- {o}: {question.descriptions[o]}" if question.descriptions.get(o) else f"- {o}"
                        for o in question.options)
    wire_state = state if isinstance(state, str) else json.dumps(state, ensure_ascii=False, default=list)
    schema = {"type": "object", "additionalProperties": False, "required": ["choice", "confidence"],
              "properties": {"choice": {"type": "string", "enum": list(question.options)},
                             "confidence": {"type": "number", "minimum": 0, "maximum": 1}}}
    system, fmt = SYSTEM_PROMPT, {"type": JSON_SCHEMA,
                                  "json_schema": {"name": "choice", "strict": True, "schema": schema}}
    if response_format == JSON_OBJECT:
        system = f"{SYSTEM_PROMPT} JSON-схема ответа: {json.dumps(schema, ensure_ascii=False)}"
        fmt = {"type": JSON_OBJECT}
    return {"model": model, "temperature": 0,
            "messages": [{"role": "system", "content": system},
                         {"role": "user",
                          "content": f"{question.text}\n\nСостояние: {wire_state}\n\nВарианты:\n{options}"}],
            "response_format": fmt}


def parse_answer(payload: Mapping[str, Any]) -> ChoiceAnswer:
    try:
        raw = json.loads(payload["choices"][0]["message"]["content"])
        choice, confidence = str(raw["choice"]), float(raw["confidence"])
    except (KeyError, IndexError, TypeError, ValueError, AttributeError):
        raise ProviderInvalidResponse(f"{NAME}: malformed completion") from None
    return ChoiceAnswer(choice, {choice: 1.0}, confidence)
