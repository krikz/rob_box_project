#!/usr/bin/env python3
"""judge.py — слушающий судья за интерфейсом (ADR-0157 §2.5, §3.6, J3).

Судья получает запись + промпт (с кодом/моделью трека, §1.4: без кода он выдумывает) и отвечает списком
утверждений фиксированной схемы ``[{t0, t1, kind, claim, confidence}]``. Он только **предлагает**: ни одно
утверждение не становится карточкой, пока ``verify.py`` его не подтвердил.

Провайдер выбирается env ``CRITIC_JUDGE`` = ``fake`` | ``qwen_text`` | ``qwen`` | ``gemini``. Не задан или неизвестен — ошибка
``JudgeUnavailable`` (тихой деградации «на фейк» нет). ``qwen``/``gemini`` без ключа тоже падают понятной ошибкой;
с ключом они пока каркас: реальный вызов API не написан и **не проверен** (ключа нет) — честный ``JudgeNotImplemented``.

Кеш: ответ лежит рядом с записью, имя ``<запись>.<sha256[:16] записи>.<sha256[:8] промпта>.<провайдер>.judge.json``.
Бюджет J3: env ``CRITIC_JUDGE_BUDGET_USD`` (по умолчанию 0.05 на прогон), счётчик стоимости — ``Budget``.
"""

from __future__ import annotations

import hashlib
import json
import os
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol

#: Перечень классов утверждений (ADR-0157 §2.5); ``param_reading`` — чтение числового параметра кода (``lpf``,
#: ``delay``, ``sus``, ``amp``, ``tempo``, ``pshift``, ``scale``): отдельный класс, потому что именно тут судья ошибался (§1.4).
KINDS = ("hole", "overlap", "level", "sustain", "phrasing", "out_of_key", "chord_clash", "mix", "masking",
         "param_reading")
#: Необязательные подсказки судье-«указателю» (роль, интервал, число…): без них верификатор честно ответит «не проверяемо».
ARG_KEYS = ("role", "other", "direction", "midi", "pcs", "interval", "param", "value", "seconds")
DEFAULT_BUDGET_USD = 0.05  # ADR-0157 J3: ≤ $0.05 на запись
COST_PER_CALL_USD = {"fake": 0.0, "qwen": 0.01, "qwen_text": 0.01, "gemini": 0.01}  # оценка ADR §3.6 по чужим ценам, не измерено


class JudgeError(RuntimeError):
    """База ошибок судьи."""


class JudgeUnavailable(JudgeError):
    """Провайдер не выбран / нет ключа / нет заготовки."""


class JudgeNotImplemented(JudgeError):
    """Каркас: вызов API не написан."""


class JudgeSchemaError(JudgeError):
    """Ответ не по схеме."""


class BudgetExceeded(JudgeError):
    """Лимит стоимости прогона (J3) исчерпан."""


@dataclass(frozen=True)
class Claim:
    t0: float
    t1: float
    kind: str
    claim: str
    confidence: float
    args: Mapping[str, Any] = field(default_factory=dict)

    def as_json(self) -> dict:
        out = {"t0": self.t0, "t1": self.t1, "kind": self.kind, "claim": self.claim, "confidence": self.confidence}
        return {**out, "args": dict(self.args)} if self.args else out


def parse_claims(raw: Any) -> list[Claim]:
    """Строгий разбор ответа: список объектов схемы, иначе ``JudgeSchemaError`` с путём до поля."""
    if not isinstance(raw, list):
        raise JudgeSchemaError(f"ответ должен быть списком, а не {type(raw).__name__}")
    out = []
    for i, row in enumerate(raw):
        if not isinstance(row, dict):
            raise JudgeSchemaError(f"[{i}]: не объект")
        missing = {"t0", "t1", "kind", "claim", "confidence"} - set(row)
        extra = set(row) - {"t0", "t1", "kind", "claim", "confidence", "args"}
        if missing or extra:
            raise JudgeSchemaError(f"[{i}]: нет полей {sorted(missing)}, лишние {sorted(extra)}")
        if row["kind"] not in KINDS:
            raise JudgeSchemaError(f"[{i}].kind={row['kind']!r}: вне перечня {KINDS}")
        for key in ("t0", "t1", "confidence"):
            if isinstance(row[key], bool) or not isinstance(row[key], (int, float)):
                raise JudgeSchemaError(f"[{i}].{key}: не число")
        if not 0 <= row["t0"] <= row["t1"]:
            raise JudgeSchemaError(f"[{i}]: t0={row['t0']} t1={row['t1']} — нужно 0 <= t0 <= t1")
        if not 0 <= row["confidence"] <= 1:
            raise JudgeSchemaError(f"[{i}].confidence={row['confidence']}: вне 0..1")
        if not isinstance(row["claim"], str) or not row["claim"].strip():
            raise JudgeSchemaError(f"[{i}].claim: пустой текст")
        args = row.get("args", {})
        if not isinstance(args, dict) or set(args) - set(ARG_KEYS):
            raise JudgeSchemaError(f"[{i}].args: ожидался объект с ключами из {ARG_KEYS}")
        out.append(Claim(float(row["t0"]), float(row["t1"]), row["kind"], row["claim"], float(row["confidence"]), args))
    return out


class Judge(Protocol):
    name: str

    def judge(self, recording: Path | None, prompt: str) -> list[Claim]: ...


class FakeJudge:
    """Читает заготовленный JSON (``{"claims": [...]}`` или голый список) — офлайн-прогон верификатора без API."""

    name = "fake"

    def __init__(self, fixture: Path) -> None:
        if not Path(fixture).is_file():
            raise JudgeUnavailable(f"FakeJudge: нет заготовки {fixture}")
        self.fixture = Path(fixture)

    def judge(self, recording: Path | None, prompt: str) -> list[Claim]:
        data = json.loads(self.fixture.read_text(encoding="utf-8"))
        return parse_claims(data["claims"] if isinstance(data, dict) else data)


class _KeyedJudge:
    name = ""
    env_keys: tuple[str, ...] = ()

    def __init__(self, env: Mapping[str, str] | None = None) -> None:
        env = os.environ if env is None else env
        self.key = next((env[k] for k in self.env_keys if env.get(k)), "")
        if not self.key:
            raise JudgeUnavailable(f"{self.name}: нет ключа API (env {' / '.join(self.env_keys)}); судья не вызван")

    def judge(self, recording: Path | None, prompt: str) -> list[Claim]:
        raise JudgeNotImplemented(f"{self.name}: вызов API не написан и не проверен (ключа на момент PR-4 не было)")


class QwenOmniJudge(_KeyedJudge):
    """Слушающий Qwen Omni: в подписке (проверено 12.10) нет модели со звуком — честная ошибка, а не имитация."""

    name = "qwen"
    env_keys = ("QWEN_API_KEY", "DASHSCOPE_API_KEY")

    def judge(self, recording: Path | None, prompt: str) -> list[Claim]:
        raise JudgeNotImplemented("qwen: нет audio-модели в подписке (input_audio -> 400 «incorrect modal audio»); "
                                  "используй CRITIC_JUDGE=qwen_text — он НЕ слушает запись")


QWEN_BASE_URL = "https://token-plan.maas.qwencloudapi.com/compatible-mode/v1"
QWEN_TEXT_MODEL = "qwen3.8-max"
TEXT_ONLY_RULES = ("Ответь ТОЛЬКО JSON-списком [{t0,t1,kind,claim,confidence,args?}], kind из: " + ", ".join(KINDS)
                   + ". Тебе дан код/модель трека и метрики записи текстом; звука ты не слышал — не утверждай слышимое.")
Transport = Callable[[str, dict, dict], dict]


def _post_json(url: str, headers: dict, body: dict) -> dict:
    req = urllib.request.Request(url, json.dumps(body).encode("utf-8"), {**headers, "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as resp:  # noqa: S310 — адрес задан константой/env
        return json.loads(resp.read().decode("utf-8"))


class QwenTextJudge:
    """OpenAI-совместимый chat/completions (qwen3.8-max): код + метрики записи текстом, звук не слушает.

    ``listens = False`` — отчёт помечает ``provider=text_only``. Имя env с ключом (по умолчанию ``QWEN``) и путь к ``.env``
    настраиваются; значение ключа нигде не печатается. Реальный вызов не проверен (только против заглушки transport).
    """

    name = "qwen_text"
    listens = False

    def __init__(self, env: Mapping[str, str] | None = None, key_var: str = "QWEN", env_file: Path | None = None,
                 transport: Transport = _post_json) -> None:
        env = os.environ if env is None else env
        self.key = env.get(key_var, "") or _dotenv_value(env_file, key_var)
        if not self.key:
            raise JudgeUnavailable(f"qwen_text: нет ключа (env {key_var}, .env {env_file}); судья не вызван")
        self.base = env.get("QWEN_BASE_URL", QWEN_BASE_URL).rstrip("/")
        self.model = env.get("QWEN_TEXT_MODEL", QWEN_TEXT_MODEL)
        self.transport = transport

    def judge(self, recording: Path | None, prompt: str) -> list[Claim]:
        body = {"model": self.model, "messages": [{"role": "system", "content": TEXT_ONLY_RULES},
                                                  {"role": "user", "content": prompt}]}
        reply = self.transport(f"{self.base}/chat/completions", {"Authorization": f"Bearer {self.key}"}, body)
        try:
            text = reply["choices"][0]["message"]["content"].strip()
            text = text.removeprefix("```json").removeprefix("```").removesuffix("```").strip()
            return parse_claims(json.loads(text))
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise JudgeSchemaError(f"qwen_text: ответ не разобран ({type(exc).__name__}: {exc})") from exc


def _dotenv_value(path: Path | None, var: str) -> str:
    if path is None or not Path(path).is_file():
        return ""
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        name, sep, value = line.partition("=")
        if sep and name.strip() == var:
            return value.strip().strip("\"'")
    return ""


class GeminiJudge(_KeyedJudge):
    name = "gemini"
    env_keys = ("GEMINI_API_KEY", "GOOGLE_API_KEY")


def make_judge(env: Mapping[str, str] | None = None) -> Judge:
    """Провайдер по env ``CRITIC_JUDGE``; заготовка ``fake`` — ``CRITIC_JUDGE_FIXTURE``."""
    env = os.environ if env is None else env
    name = env.get("CRITIC_JUDGE", "")
    if name == "fake":
        if not env.get("CRITIC_JUDGE_FIXTURE"):
            raise JudgeUnavailable("CRITIC_JUDGE=fake требует CRITIC_JUDGE_FIXTURE=<json>")
        return FakeJudge(Path(env["CRITIC_JUDGE_FIXTURE"]))
    if name == "qwen_text":
        env_file = Path(env["CRITIC_JUDGE_ENV_FILE"]) if env.get("CRITIC_JUDGE_ENV_FILE") else None
        return QwenTextJudge(env, env.get("CRITIC_JUDGE_KEY_VAR", "QWEN"), env_file)
    providers = {"qwen": QwenOmniJudge, "gemini": GeminiJudge}
    if name not in providers:
        raise JudgeUnavailable(f"CRITIC_JUDGE={name!r}: ожидалось fake|qwen_text|qwen|gemini (по умолчанию судьи нет)")
    return providers[name](env)


class Budget:
    """Счётчик стоимости прогона (J3): ``charge`` до вызова, чтобы лимит не перелетали."""

    def __init__(self, limit_usd: float | None = None, env: Mapping[str, str] | None = None) -> None:
        env = os.environ if env is None else env
        self.limit = float(env.get("CRITIC_JUDGE_BUDGET_USD", DEFAULT_BUDGET_USD)) if limit_usd is None else limit_usd
        self.spent = 0.0
        self.calls = 0
        self.cache_hits = 0

    def charge(self, cost: float, provider: str) -> None:
        if self.spent + cost > self.limit + 1e-12:
            raise BudgetExceeded(f"{provider}: {self.spent:.4f} + {cost:.4f} > лимит {self.limit:.4f} USD на прогон")
        self.spent += cost
        self.calls += 1


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def cache_path(recording: Path, prompt: str, provider: str) -> Path:
    prompt_sha = hashlib.sha256(prompt.encode("utf-8")).hexdigest()[:8]
    return recording.with_name(f"{recording.name}.{sha256_file(recording)[:16]}.{prompt_sha}.{provider}.judge.json")


def run_judge(judge: Judge, recording: Path, prompt: str, budget: Budget) -> list[Claim]:
    """Кеш рядом с записью → иначе бюджет → вызов судьи → проверка схемы → запись кеша."""
    path = cache_path(recording, prompt, judge.name)
    if path.is_file():
        budget.cache_hits += 1
        return parse_claims(json.loads(path.read_text(encoding="utf-8")))
    budget.charge(COST_PER_CALL_USD.get(judge.name, 0.0), judge.name)
    claims = judge.judge(recording, prompt)
    path.write_text(json.dumps([c.as_json() for c in claims], ensure_ascii=False, indent=1), encoding="utf-8")
    return claims
