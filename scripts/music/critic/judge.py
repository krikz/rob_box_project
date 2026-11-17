#!/usr/bin/env python3
"""judge.py — слушающий судья за интерфейсом (ADR-0157 §2.5, §3.6, J3).

Судья получает запись + промпт (с кодом/моделью трека, §1.4: без кода он выдумывает) и отвечает списком
утверждений фиксированной схемы ``[{t0, t1, kind, claim, confidence}]``. Он только **предлагает**: ни одно
утверждение не становится карточкой, пока ``verify.py`` его не подтвердил.

Провайдер выбирается env ``CRITIC_JUDGE`` = ``fake`` | ``qwen_text`` | ``qwen_omni`` | ``gemini``. Не задан или неизвестен — ошибка
``JudgeUnavailable`` (тихой деградации «на фейк» нет). ``qwen_omni``/``qwen_text``/``gemini`` без ключа тоже падают понятной ошибкой;
``gemini`` с ключом — каркас (``JudgeNotImplemented``). CLI: ``python judge.py --help``.

Кеш: ответ лежит рядом с записью, имя ``<запись>.<sha256[:16] записи>.<sha256[:8] промпта>.<провайдер>.judge.json``.
Бюджет J3: env ``CRITIC_JUDGE_BUDGET_USD`` (по умолчанию 0.25 на прогон), счётчик стоимости — ``Budget``.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import re
import sys
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Protocol

#: Перечень классов утверждений (ADR-0157 §2.5); ``param_reading`` — чтение числового параметра кода (``lpf``,
#: ``delay``, ``sus``, ``amp``, ``tempo``, ``pshift``, ``scale``): отдельный класс, потому что именно тут судья ошибался (§1.4).
KINDS = ("hole", "overlap", "level", "sustain", "phrasing", "out_of_key", "chord_clash", "mix", "masking",
         "param_reading")
#: Необязательные подсказки судье-«указателю» (роль, интервал, число…): без них верификатор честно ответит «не проверяемо».
ARG_KEYS = ("role", "other", "direction", "midi", "pcs", "interval", "param", "value", "seconds")
INVALID_KEY = "_invalid"  # {ключ: причина}: то, что судья прислал не в форме схемы; значение не выкидывается молча
STR_ARGS = ("role", "other", "direction", "interval", "param")
LIST_ARGS = ("midi", "pcs")
DEFAULT_BUDGET_USD = 0.25  # ADR-0157 J3 целил ≤ $0.05; omni на клубе вышел $0.08 по ценам-заглушкам — лимит поднят
COST_PER_CALL_USD = {"fake": 0.0, "qwen_omni": 0.01, "qwen_text": 0.01, "gemini": 0.01}  # оценка ADR §3.6 по чужим ценам, не измерено


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


def _num(v: Any) -> float | None:
    if isinstance(v, bool):
        return None
    if isinstance(v, (int, float)):
        return float(v)
    try:
        return float(str(v).strip().replace(",", "."))
    except ValueError:
        return None


def _norm_arg(key: str, v: Any) -> Any:
    """Нормализация одного значения args; ``ValueError`` — форма не распознана."""
    if key in STR_ARGS:
        if isinstance(v, str):
            return v
        raise ValueError(f"ожидалась строка, пришло {type(v).__name__}")
    if key in LIST_ARGS:  # midi=35 -> [35]; "35" -> [35]; [35, "36"] -> [35, 36]
        items = v if isinstance(v, (list, tuple)) else [v]
        nums = [_num(x) for x in items]
        if not nums or any(n is None for n in nums):
            raise ValueError(f"не список чисел: {v!r}")
        return [int(n) for n in nums]
    items = v if isinstance(v, (list, tuple)) else [v]  # value / seconds: число, строка-число или список
    nums = [_num(x) for x in items]
    if not nums or len(nums) > 2 or any(n is None for n in nums):
        raise ValueError(f"не число и не пара чисел: {v!r}")
    if len(nums) == 2 and key == "seconds":  # [127.06, 141.18] — границы интервала -> его длительность
        return nums[1] - nums[0] if nums[1] > nums[0] else nums[0]
    return nums[0]


def normalize_args(args: Any) -> dict:
    """Приводит args реального ответа судьи к схеме; нераспознанное не выкидывается, а помечается ``_invalid``."""
    if not isinstance(args, dict):
        return {INVALID_KEY: {"args": f"ожидался объект, пришло {type(args).__name__}"}}
    out, bad = {}, {}
    for key, v in args.items():
        if key not in ARG_KEYS:
            bad[str(key)] = "неизвестный ключ"
            continue
        try:
            out[key] = _norm_arg(key, v)
        except ValueError as exc:
            bad[key] = str(exc)
    return {**out, INVALID_KEY: bad} if bad else out


def parse_claims(raw: Any) -> list[Claim]:
    """Строгий разбор ответа: список объектов схемы, иначе ``JudgeSchemaError`` с путём до поля; args нормализуются."""
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
        args = normalize_args(row.get("args", {}))
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


OMNI_BASE_URL = "https://maas.qwencloudapi.com/compatible-mode/v1"
OMNI_MODEL = "qwen3.8-omni-flash"  # env CRITIC_OMNI_MODEL; запасная — qwen3.5-omni-plus (без reasoning)
#: USD за 1M токенов (текст-вход, аудио-вход, выход). НЕ СВЕРЕНО С ПРАЙСОМ — числа-заглушки, обновить по прайсу провайдера.
OMNI_PRICE_USD_PER_M = {"qwen3.8-omni-flash": (0.4, 3.0, 1.6), "qwen3.5-omni-plus": (0.8, 6.0, 3.2)}
OMNI_RULES = (TEXT_ONLY_RULES.split(". Тебе дан")[0] + ". Ты слушаешь приложенную запись И видишь код/модель трека: "
              "t0/t1 — секунды файла; для каждого утверждения заполни args (role из kick/hats/clap/bass/pad/lead/loop/fx, "
              "midi, param, seconds), когда это следует из кода.")
SseTransport = Callable[[str, dict, dict], Iterable[str]]


def _post_sse(url: str, headers: dict, body: dict) -> Iterable[str]:
    req = urllib.request.Request(url, json.dumps(body).encode("utf-8"), {**headers, "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=300) as resp:  # noqa: S310 — адрес задан константой/env
            for raw in resp:
                yield raw.decode("utf-8", errors="replace").rstrip("\r\n")
    except urllib.error.HTTPError as exc:
        raise JudgeError(f"HTTP {exc.code}: {exc.read().decode('utf-8', errors='replace')[:300]}") from exc


def parse_sse(lines: Iterable[str]) -> tuple[str, dict]:
    """SSE ``data:`` строки chat/completions -> (склеенный текст delta.content, usage последнего чанка)."""
    text, usage = [], {}
    for line in lines:
        if not line.startswith("data:"):
            continue
        payload = line[5:].strip()
        if payload == "[DONE]":
            break
        chunk = json.loads(payload)
        if chunk.get("error"):
            raise JudgeError(f"ошибка в SSE-чанке: {str(chunk['error'])[:300]}")
        for choice in chunk.get("choices") or ():
            text.append((choice.get("delta") or {}).get("content") or "")
        usage = chunk.get("usage") or usage
    return "".join(text), usage


def omni_cost_usd(model: str, usage: Mapping[str, Any]) -> float:
    """Стоимость по usage и таблице ``OMNI_PRICE_USD_PER_M`` (не сверена с прайсом)."""
    text_in, audio_in, out = OMNI_PRICE_USD_PER_M.get(model, (0.0, 0.0, 0.0))
    audio = (usage.get("prompt_tokens_details") or {}).get("audio_tokens", 0)
    plain = max(usage.get("prompt_tokens", 0) - audio, 0)
    return (plain * text_in + audio * audio_in + usage.get("completion_tokens", 0) * out) / 1e6


class QwenOmniJudge:
    """Слушающий Qwen Omni (pay-as-you-go, ключ ``QWEN_PSG``): запись (input_audio mp3) + модель трека текстом, SSE-поток.

    После ``judge`` в ``last_usage`` — usage, в ``last_cost_usd`` — стоимость по таблице цен. Проверено только против
    заглушки transport; реальные вызовы делает координатор.
    """

    name = "qwen_omni"
    listens = True

    def __init__(self, env: Mapping[str, str] | None = None, key_var: str = "QWEN_PSG", env_file: Path | None = None,
                 transport: SseTransport = _post_sse) -> None:
        env = os.environ if env is None else env
        self.key = env.get(key_var, "") or _dotenv_value(env_file, key_var)
        if not self.key:
            raise JudgeUnavailable(f"qwen_omni: нет ключа (env {key_var}, .env {env_file}); судья не вызван")
        self.base = env.get("CRITIC_OMNI_BASE_URL", OMNI_BASE_URL).rstrip("/")
        self.model = env.get("CRITIC_OMNI_MODEL", OMNI_MODEL)
        self.transport = transport
        self.last_usage: dict = {}
        self.last_cost_usd = 0.0

    def judge(self, recording: Path | None, prompt: str) -> list[Claim]:
        if recording is None or not Path(recording).is_file():
            raise JudgeUnavailable("qwen_omni: нужна запись (mp3) — судья без звука это qwen_text")
        audio = base64.b64encode(Path(recording).read_bytes()).decode("ascii")
        body = {"model": self.model, "stream": True, "stream_options": {"include_usage": True}, "modalities": ["text"],
                "messages": [{"role": "system", "content": OMNI_RULES},
                             {"role": "user", "content": [
                                 {"type": "input_audio", "input_audio": {"data": f"data:;base64,{audio}", "format": "mp3"}},
                                 {"type": "text", "text": prompt}]}]}
        text, usage = parse_sse(self.transport(f"{self.base}/chat/completions", {"Authorization": f"Bearer {self.key}"}, body))
        self.last_usage, self.last_cost_usd = usage, omni_cost_usd(self.model, usage)
        try:
            text = text.strip().removeprefix("```json").removeprefix("```").removesuffix("```").strip()
            return parse_claims(json.loads(text))
        except (TypeError, ValueError) as exc:
            raise JudgeSchemaError(f"qwen_omni: ответ не разобран ({type(exc).__name__}: {exc}): {text[:200]!r}") from exc


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
    if name == "qwen_omni":
        env_file = Path(env["CRITIC_JUDGE_ENV_FILE"]) if env.get("CRITIC_JUDGE_ENV_FILE") else None
        return QwenOmniJudge(env, env.get("CRITIC_JUDGE_KEY_VAR", "QWEN_PSG"), env_file)
    providers = {"gemini": GeminiJudge}
    if name not in providers:
        raise JudgeUnavailable(f"CRITIC_JUDGE={name!r}: ожидалось fake|qwen_text|qwen_omni|gemini (по умолчанию судьи нет)")
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
    estimate = COST_PER_CALL_USD.get(judge.name, 0.0)
    budget.charge(estimate, judge.name)
    claims = judge.judge(recording, prompt)
    if getattr(judge, "last_cost_usd", None) is not None:  # фактическая стоимость по usage вместо оценки
        budget.spent += judge.last_cost_usd - estimate
    path.write_text(json.dumps([c.as_json() for c in claims], ensure_ascii=False, indent=1), encoding="utf-8")
    return claims


# --- CLI: один стиль бандла -> судья -> фикстура для verify.py -------------------------------------------------

TRACK_HEAD = re.compile(r"^`(set\d+):(\d+):([A-Z]):([0-9a-f]+)` .*?≈([\d.]+) с в файле", re.M)


def style_inputs(bundle_dir: Path, style: str) -> tuple[Path, str, list[dict]]:
    """(mp3, промпт = md бандла до ответов судьи, tracks[{model,start_s}]) для стиля ``<bundle>/<style>/<style>.md``."""
    md = (bundle_dir / style / f"{style}.md").read_text(encoding="utf-8")
    head = md.split("\n## Ответ", 1)[0]
    tracks = [{"model": f"{s}_{n}_{d}_{h}.json", "start_s": float(off)} for s, n, d, h, off in TRACK_HEAD.findall(head)]
    if not tracks:
        raise JudgeError(f"{style}: в md нет блоков «Код аранжировщика» с позицией трека")
    return bundle_dir / style / f"{style}.mp3", head, tracks


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description="Судья на одном стиле бандла; результат — фикстура для verify.py")
    ap.add_argument("--provider", required=True, choices=["qwen_omni", "qwen_text", "fake"])
    ap.add_argument("--env-file", type=Path, default=None, help=".env с ключом (значение не печатается)")
    ap.add_argument("--bundle-dir", type=Path, required=True, help="style_critic бандл (<style>/<style>.md и .mp3)")
    ap.add_argument("--style", required=True)
    ap.add_argument("--out", type=Path, required=True, help="каталог результата: <style>.json")
    ap.add_argument("--fixture", type=Path, default=None, help="для provider=fake: заготовка")
    args = ap.parse_args(argv)
    env = {**os.environ, "CRITIC_JUDGE": args.provider}
    if args.env_file:
        env["CRITIC_JUDGE_ENV_FILE"] = str(args.env_file)
    if args.fixture:
        env["CRITIC_JUDGE_FIXTURE"] = str(args.fixture)
    judge = make_judge(env)
    mp3, prompt, tracks = style_inputs(args.bundle_dir, args.style)
    budget = Budget(env=env)
    claims = run_judge(judge, mp3, prompt, budget)
    args.out.mkdir(parents=True, exist_ok=True)
    out = args.out / f"{args.style}.json"
    doc = {"style": args.style, "provider": judge.name, "listens": getattr(judge, "listens", None),
           "model": getattr(judge, "model", None), "usage": getattr(judge, "last_usage", None),
           "cost_usd": round(budget.spent, 5), "tracks": tracks, "claims": [c.as_json() for c in claims]}
    out.write_text(json.dumps(doc, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    if judge.name == "qwen_omni":
        print("ВНИМАНИЕ: цены OMNI_PRICE_USD_PER_M не сверены с прайсом — cost ниже ориентировочный", file=sys.stderr)
    print(f"{args.style}: {len(claims)} утверждений, provider={judge.name}, cost=${budget.spent:.4f}, "
          f"cache_hits={budget.cache_hits} -> {out}")
    print(f"дальше: python scripts/music/critic/verify.py {out} --tracks <каталог моделей треков> --out {args.out / 'report.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
