"""HTTP-провайдер System One: Jev (TypeSafe) и Laya (через Ollaya).

Один клиент на оба провайдера: Ollaya отдаёт Laya за TypeSafe-совместимым
``POST /v1/systemone`` (docs/research/jev/laya-survey.md §API), поэтому
Laya = тот же провайдер с другим ``base_url``/``model`` и без ключа.

Wire-контракт взят из исходников официального ``typesafe-sdk-python``
(0.7.2, f078f1e, см. docs/research/jev/README.md §«API contract»),
потому что api.typesafe.ai/docs недоступен из среды исследования:

* запрос ``{"model", "state", "questions": {id: {"type", "instructions",
  "criteria"}}}``; для ``choice`` criteria — ``{label: description|null}``,
  для ``score`` — упорядоченный список уровней (индекс = балл);
* ответ ``{"model", "answers": {id: ...}, "usage": {...}}``; ``noul`` —
  только ``{"noul": p}``; ``choice`` — ``choice/confidence/probabilities``;
  ``score`` — ``score/confidence/probabilities`` с ключами ``"0".."n"``.

Официальный SDK НЕ используется намеренно: по умолчанию он делает до 2
ретраев с бюджетом 30 с (``_core/retry.py``) — на критическом пути это
запрещено issue #3084. Здесь ретраев нет вовсе; дедлайн ставит
оркестратор, собственный ``timeout_s`` — только верхняя страховка.

Секреты: ключ читается из env в момент вызова, уходит только в заголовок
``Authorization`` и не попадает ни в исключения, ни в логи.
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Callable, Mapping

from .types import (
    ChoiceAnswer,
    ChoiceQuestion,
    Decision,
    NoulAnswer,
    NoulQuestion,
    ProviderAuthError,
    ProviderConfigError,
    ProviderInvalidResponse,
    ProviderRateLimited,
    ProviderTimeout,
    ProviderUnavailable,
    Question,
    ScoreQuestion,
    State,
)

SYSTEM_ONE_PATH = "/v1/systemone"
MODELS_PATH = "/v1/models"

JEV_BASE_URL = "https://api.typesafe.ai"
#: Версия пинуется (awesome-jev: «pin the version when comparing evaluations»);
#: ``jev-latest`` — плавающий алиас. Подтвердить через ``GET /v1/models``
#: на реальном ключе — пункт «Что осталось» в docs/research/jev/README.md.
JEV_DEFAULT_MODEL = "jev-1.13.0"
JEV_API_KEY_ENV = "JEV_API_KEY"
JEV_MODEL_ENV = "JEV_MODEL"
JEV_BASE_URL_ENV = "JEV_BASE_URL"

LAYA_BASE_URL = "http://127.0.0.1:11435"
#: ``laya`` в Ollaya — роутер: кириллица уходит в ``laya:multilingual``.
LAYA_DEFAULT_MODEL = "laya"
LAYA_BASE_URL_ENV = "LAYA_BASE_URL"
LAYA_MODEL_ENV = "LAYA_MODEL"

ClientFactory = Callable[[float], Any]


def _default_client_factory(timeout_s: float) -> Any:
    import httpx  # ленивый импорт: без httpx пакет импортируется, Jev просто не работает

    # trust_env=False: локальную Ollaya нельзя отправлять через HTTP(S)_PROXY.
    return httpx.AsyncClient(timeout=timeout_s, trust_env=False)


class SystemOneHttpProvider:
    """Провайдер поверх ``/v1/systemone`` (TypeSafe или совместимый сервер)."""

    def __init__(
        self,
        name: str,
        *,
        base_url: str,
        model: str,
        api_key_env: str | None = None,
        timeout_s: float = 2.0,
        client_factory: ClientFactory = _default_client_factory,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.name = name
        self.base_url = base_url.rstrip("/")
        self.model = model
        self._api_key_env = api_key_env
        self._timeout_s = timeout_s
        self._client_factory = client_factory
        self._clock = clock

    async def decide(self, state: State, questions: Mapping[str, Question]) -> Decision:
        body = build_request(self.model, state, questions)
        started = self._clock()
        payload = await self._request("POST", SYSTEM_ONE_PATH, body)
        answers = parse_answers(payload, questions)
        usage = payload.get("usage") if isinstance(payload.get("usage"), Mapping) else {}
        return Decision(
            answers=answers,
            provider=self.name,
            model=str(payload.get("model") or self.model),
            latency_ms=(self._clock() - started) * 1000.0,
            usage=dict(usage),
        )

    async def list_models(self) -> list[str]:
        """``GET /v1/models`` → имена моделей (для проверки model id)."""
        payload = await self._request("GET", MODELS_PATH, None)
        models = payload.get("models")
        if not isinstance(models, list):
            raise ProviderInvalidResponse("models: expected list")
        return [str(m.get("name")) for m in models if isinstance(m, Mapping)]

    async def _request(self, method: str, path: str, body: Mapping[str, Any] | None) -> Mapping[str, Any]:
        headers = self._headers()
        async with self._client_factory(self._timeout_s) as client:
            try:
                response = await client.request(method, self.base_url + path, json=body, headers=headers)
            except Exception as exc:  # noqa: BLE001 — маппинг транспортных ошибок ниже
                raise _transport_error(self.name, exc) from None
        _raise_for_status(self.name, response.status_code)
        try:
            payload = response.json()
        except (ValueError, json.JSONDecodeError):
            raise ProviderInvalidResponse(f"{self.name}: response is not JSON") from None
        if not isinstance(payload, Mapping):
            raise ProviderInvalidResponse(f"{self.name}: response is not an object")
        return payload

    def _headers(self) -> dict[str, str]:
        headers = {"Accept": "application/json"}
        if self._api_key_env is None:
            return headers
        key = os.environ.get(self._api_key_env, "").strip()
        if not key:
            raise ProviderConfigError(f"{self.name}: env {self._api_key_env} is not set")
        headers["Authorization"] = f"Bearer {key}"
        return headers


def jev_from_env(env: Mapping[str, str] | None = None, **kwargs: Any) -> SystemOneHttpProvider | None:
    """Jev-провайдер, если задан ``JEV_API_KEY``; иначе ``None`` (Jev опционален)."""
    env = os.environ if env is None else env
    if not env.get(JEV_API_KEY_ENV, "").strip():
        return None
    return SystemOneHttpProvider(
        "jev",
        base_url=env.get(JEV_BASE_URL_ENV) or JEV_BASE_URL,
        model=env.get(JEV_MODEL_ENV) or JEV_DEFAULT_MODEL,
        api_key_env=JEV_API_KEY_ENV,
        **kwargs,
    )


def laya_from_env(env: Mapping[str, str] | None = None, **kwargs: Any) -> SystemOneHttpProvider | None:
    """Laya через локальную Ollaya, если задан ``LAYA_BASE_URL``; иначе ``None``."""
    env = os.environ if env is None else env
    base_url = env.get(LAYA_BASE_URL_ENV, "").strip()
    if not base_url:
        return None
    return SystemOneHttpProvider(
        "laya", base_url=base_url, model=env.get(LAYA_MODEL_ENV) or LAYA_DEFAULT_MODEL, **kwargs
    )


# ---------------------------------------------------------------------------
# Сериализация
# ---------------------------------------------------------------------------


def build_request(model: str, state: State, questions: Mapping[str, Question]) -> dict[str, Any]:
    if not questions:
        raise ValueError("at least one question is required")
    wire_state: Any = state if isinstance(state, (str, Mapping)) else list(state)
    return {
        "model": model,
        "state": wire_state,
        "questions": {qid: _question_to_wire(q) for qid, q in questions.items()},
    }


def _question_to_wire(question: Question) -> dict[str, Any]:
    if isinstance(question, NoulQuestion):
        return {"type": "noul", "instructions": question.text}
    if isinstance(question, ChoiceQuestion):
        return {"type": "choice", "instructions": question.text, "criteria": {o: None for o in question.options}}
    if isinstance(question, ScoreQuestion):
        return {"type": "score", "instructions": question.text, "criteria": list(question.levels)}
    raise TypeError(f"unsupported question type: {type(question).__name__}")


def parse_answers(payload: Mapping[str, Any], questions: Mapping[str, Question]) -> dict[str, Any]:
    """Wire → типизированные ответы. Полная валидация — в оркестраторе."""
    raw = payload.get("answers")
    if not isinstance(raw, Mapping):
        raise ProviderInvalidResponse("answers: expected object")
    answers: dict[str, Any] = {}
    for qid, question in questions.items():
        item = raw.get(qid)
        if not isinstance(item, Mapping):
            raise ProviderInvalidResponse(f"{qid}: missing answer")
        answers[qid] = _parse_one(qid, question, item)
    return answers


def _parse_one(qid: str, question: Question, item: Mapping[str, Any]) -> Any:
    try:
        if isinstance(question, NoulQuestion):
            return NoulAnswer(probability=float(item["noul"]))
        if isinstance(question, ChoiceQuestion):
            probs = {str(k): float(v) for k, v in item["probabilities"].items()}
            return ChoiceAnswer(str(item["choice"]), probs, float(item["confidence"]))
        return _parse_score(question, item)
    except (KeyError, TypeError, ValueError, AttributeError) as exc:
        raise ProviderInvalidResponse(f"{qid}: malformed answer ({type(exc).__name__})") from None


def _parse_score(question: ScoreQuestion, item: Mapping[str, Any]) -> ChoiceAnswer:
    """Индексы уровней ``"0".."n"`` → имена уровней; ответ = самый вероятный уровень."""
    probs: dict[str, float] = {}
    for key, value in item["probabilities"].items():
        index = int(key)
        if not 0 <= index < len(question.levels):
            raise ValueError("score level out of range")
        probs[question.levels[index]] = float(value)
    best = max(probs, key=probs.__getitem__)
    return ChoiceAnswer(best, probs, float(item["confidence"]))


# ---------------------------------------------------------------------------
# Ошибки
# ---------------------------------------------------------------------------


def _raise_for_status(name: str, status: int) -> None:
    if 200 <= status < 300:
        return
    if status in (401, 403):
        raise ProviderAuthError(f"{name}: HTTP {status}")
    if status == 429:
        raise ProviderRateLimited(f"{name}: HTTP 429")
    if status in (400, 404, 422):
        # 404 MODEL_NOT_FOUND / 422 STATE_TRUNCATED — конфиг или запрос.
        raise ProviderConfigError(f"{name}: HTTP {status}")
    raise ProviderUnavailable(f"{name}: HTTP {status}")


def _transport_error(name: str, exc: Exception) -> Exception:
    kind = type(exc).__name__
    if "Timeout" in kind:
        return ProviderTimeout(f"{name}: {kind}")
    return ProviderUnavailable(f"{name}: {kind}")
