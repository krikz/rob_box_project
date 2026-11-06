"""Генератор текста поста для канала: факты → MiniMax → JSON → проверка кодом.

Правила ролей (ADR-0148 «код решает, LLM говорит»):
  * факты собирает код (studio_facts.py) — у каждого есть источник;
  * текст пишет модель, но только по этим фактам и правилам канала;
  * проверяет код: числа и имена собственные в тексте есть среди фактов, лимиты и разметку
    проверяет channel_lib.validate, лиц/внутренних IP/ключей быть не должно.

Модель — MiniMax-M3: именно она умеет thinking: disabled (M3.1-Flash на этом отвечает HTTP 400,
см. src/rob_box_llm/rob_box_llm/providers/minimax.py), а выключенный thinking обязателен —
иначе в текст попадёт «<think>…».
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "telegram"))

import channel_lib as lib  # noqa: E402

DEFAULT_MODEL = "MiniMax-M3.1-Flash"
API_URL = "https://api.minimax.io/v1/chat/completions"

# Модели MiniMax и их «мышление» (проверено вживую 06.11.2026, см. также
# src/rob_box_llm/rob_box_llm/providers/minimax.py::apply_model_thinking):
#
#   MiniMax-M3           — thinking выключается: thinking={"type": "disabled"}
#   MiniMax-M3.1-Flash   — thinking выключить НЕЛЬЗЯ (HTTP 400), модель требует
#                          adaptive thinking и умеет брать «глубину» reasoning_effort.
#                          Без reasoning_effort размышления не ограничены: они съедают
#                          весь max_tokens, и content приходит ПУСТЫМ (finish_reason=length).
#                          Поэтому для Flash всегда ставим reasoning_effort.
THINKING_DISABLED_MODELS = ("MiniMax-M3",)
DEFAULT_REASONING_EFFORT = "low"


def thinking_params(model: str) -> dict:
    """Как выразить «думай меньше» для конкретной модели MiniMax."""
    if model in THINKING_DISABLED_MODELS:
        return {"thinking": {"type": "disabled"}}
    return {"reasoning_effort": DEFAULT_REASONING_EFFORT}

# Числа в тексте: «48 часов», «3 попытки», «99.5%». Дробные и проценты — отдельно.
NUMBER_RE = re.compile(r"\d+(?:[.,]\d+)?")
# Имена собственные — только технические: латиница в CamelCase (Nav2, RTABMap) или
# snake_case (done_claim). Русские слова с заглавной буквы (начало предложения) не проверяем:
# иначе валидатор ругается на «Проверили», «Иначе» и отбрасывает нормальные тексты.
PROPER_RE = re.compile(r"\b[A-Za-z][A-Za-z0-9]*[a-z][A-Za-z0-9]*\b|\b[a-z][a-z0-9]*(?:_[a-z0-9]+)+\b")

# Слова латиницей, которые встречаются в любом тексте и фактом не являются.
STOP_PROPER = {
    "PR",
    "PRs",
    "HTTP",
    "GitHub",
    "API",
    "HTML",
    "JSON",
    "ID",
    "OK",
    "MiniMax",
    "Telegram",
    "Robot",
    "Dev",
    "Ops",
    "Release",
    "Nightly",
    "CI",
    "CD",
    "TODO",
    "FIXME",
}


def extract_fact_text(facts: list) -> str:
    """Всё, что модель имеет право утверждать: утверждения, источники, пути."""
    parts = []
    for fact in facts:
        claim = fact.get("claim", "")
        source = fact.get("source", "")
        parts.append(f"- {claim} (источник: {source})")
        for evidence in fact.get("evidence", []) or []:
            parts.append(f"  - доказательство: {evidence}")
    return "\n".join(parts)


def load_rules(repo_root: Path) -> str:
    """Правила канала: тон, теги, лимиты, приватность."""
    path = repo_root / "docs" / "process" / "telegram-channel.md"
    if not path.is_file():
        return "Правила канала не найдены — пиши просто, по-русски, короткими абзацами."
    text = path.read_text(encoding="utf-8")
    return text[:12000]


def build_prompt(facts: list, rules: str, media_ref: str = "") -> str:
    """Промпт: только факты и правила, никаких просьб «придумай»."""
    return (
        "Ты пишешь посты для Telegram-канала проекта робота. У тебя есть только факты ниже "
        "и правила канала. НИЧЕГО, чего нет в фактах, ты не пишешь: ни новых чисел, ни новых "
        "имён, ни предположений. Если фактов мало — верни пустой массив ideas.\n\n"
        f"ПРАВИЛА КАНАЛА:\n{rules}\n\n"
        f"ФАКТЫ (источник каждого обязателен):\n{extract_fact_text(facts)}\n\n"
        f"МЕДИА: {media_ref or 'готового медиа нет — поле media_ref оставь пустым'}\n\n"
        "Верни СТРОГО JSON без пояснений:\n"
        '{"ideas": [{"id": "i1", "title": "тема поста одной строкой", '
        '"body_html": "<b>HTML</b> текст поста", "tags": ["#роббокс"], '
        '"media_ref": "путь или пустая строка", "facts": ["использованные факты"]}]}\n\n'
        "Требования к body_html: только теги b, i, code, pre из разрешённых; "
        "абзацы разделяй переводом строки, а не тегом <br> — Telegram HTML его не поддерживает; "
        "тег <a> со ссылкой на источник разрешён; "
        "цель 400–700 знаков; хештеги только из словаря канала; "
        "никаких внутренних IP, лиц людей и ключей."
    )


def call_minimax(prompt: str, model: str = DEFAULT_MODEL, api_key: Optional[str] = None, timeout: int = 240) -> dict:
    """MiniMax → JSON. Ошибка — не исключение наружу, а {"error": ...}."""
    key = api_key or os.environ.get("MINIMAX_API_KEY", "")
    if not key:
        return {"error": "MINIMAX_API_KEY не задан"}
    payload = {
        "model": model,
        "messages": [{"role": "user", "content": prompt}],
        "temperature": 0.3,
        "max_tokens": 8000,
        **thinking_params(model),
    }
    try:
        import requests

        response = requests.post(API_URL, json=payload, headers={"Authorization": f"Bearer {key}"}, timeout=timeout)
    except Exception as e:  # noqa: BLE001 — текст ошибки без ключа
        return {"error": f"сеть MiniMax: {type(e).__name__}"}
    if response.status_code != 200:
        return {"error": f"HTTP {response.status_code}: {response.text[:200]}"}
    try:
        data = response.json()
    except ValueError:
        return {"error": "ответ MiniMax не JSON"}
    choice = (data.get("choices") or [{}])[0]
    content = choice.get("message", {}).get("content", "") or ""
    if not content.strip():
        # Размышления съели весь бюджет токенов — у Flash это без reasoning_effort.
        finish = choice.get("finish_reason")
        usage = data.get("usage") or {}
        return {
            "error": f"модель вернула пустой ответ (finish_reason={finish}, "
            f"completion_tokens={usage.get('completion_tokens')}) — "
            "размышления съели max_tokens, нужен reasoning_effort"
        }
    if "<think>" in content:
        content = content.split("</think>")[-1]
    start, end = content.find("{"), content.rfind("}")
    if start < 0 or end <= start:
        return {"error": "в ответе нет JSON", "raw": content[:400]}
    try:
        return json.loads(content[start : end + 1])
    except ValueError as e:
        return {"error": f"JSON не разобрался: {e}", "raw": content[:400]}


def check_against_facts(body: str, facts: list) -> list:
    """Каждое число и имя собственное в тексте должно быть среди фактов."""
    problems = []
    parts = []
    for fact in facts:
        parts.append(fact.get("claim", ""))
        parts.append(fact.get("source", ""))
        parts.extend(fact.get("evidence", []) or [])
    blob = " ".join(parts).lower()
    # HTML убираем: атрибуты (<a href="...">) — не имена собственные, а синтаксис разметки.
    plain = re.sub(r"<[^>]+>", " ", body)
    # Числа: нормализованные (запятая → точка), чтобы «3,5» и «3.5» считались одним.
    allowed = set()
    for match in NUMBER_RE.findall(blob):
        allowed.add(match.replace(",", "."))
    for match in NUMBER_RE.findall(plain):
        normalized = match.replace(",", ".")
        if normalized not in allowed:
            problems.append(f"число «{match}» не найдено среди фактов")
    for match in PROPER_RE.findall(plain):
        if match in STOP_PROPER:
            continue
        if match.lower() not in blob:
            problems.append(f"имя «{match}» не найдено среди фактов")
    return problems


def extract_tags(body: str) -> list:
    """Хештеги из текста — по порядку появления, только из словаря канала.

    Теги достаёт код, а не модель: модель часто забывает поле tags, а хештег в тексте
    есть всегда. Так теги не могут разойтись с текстом и не могут быть чужими.
    """
    found = []
    for match in re.findall(r"#[^\W\d]\w*", body):
        if match in lib.TAG_DICTIONARY and match not in found:
            found.append(match)
    return found


def validate_draft(idea: dict, facts: list, repo_root: Path) -> list:
    """Проверка черновика кодом: факты, лимиты, разметка, теги."""
    problems = check_against_facts(idea.get("body_html", ""), facts)
    body = idea.get("body_html", "")
    tags = idea.get("tags", [])
    if not body.strip():
        problems.append("пустой текст")
    if not 2 <= len(tags) <= 5:
        problems.append(f"хештегов {len(tags)}, нужно 2–5")
    for tag in tags:
        if tag not in lib.TAG_DICTIONARY:
            problems.append(f"хештег {tag} не в словаре канала")
        if tag not in body:
            problems.append(f"хештег {tag} не упомянут в тексте")
    limit = lib.CAPTION_LIMIT if idea.get("media_ref") else lib.MESSAGE_LIMIT
    length = lib.visible_length(body)
    if length > limit:
        problems.append(f"текст {length} знаков, лимит {limit}")
    problems += lib.html_problems(body)
    for pattern, what in lib.FORBIDDEN:
        if pattern.search(body):
            problems.append(f"в тексте {what}")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--facts", type=Path, required=True, help="JSON с фактами (вывод studio_facts.py)")
    ap.add_argument("--out", type=Path, help="куда записать идеи (JSON)")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    args = ap.parse_args()

    data = json.loads(args.facts.read_text(encoding="utf-8"))
    facts = data.get("facts", data) if isinstance(data, dict) else data
    visible_facts = [f for f in facts if f.get("visible")]
    if not visible_facts:
        payload = {"ideas": [], "reason": "нет видимых фактов — показывать нечего"}
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    prompt = build_prompt(visible_facts, load_rules(args.repo_root))
    result = call_minimax(prompt, model=args.model)
    if "error" in result:
        print(json.dumps({"ideas": [], "error": result["error"]}, ensure_ascii=False, indent=2))
        return 1

    ideas = result.get("ideas") or []
    checked = []
    for idea in ideas:
        idea = dict(idea)
        # Теги — из текста (код решает), а не из ответа модели: модель их часто не возвращает.
        body = idea.get("body_html", "") or ""
        tags = extract_tags(body)
        if not tags:
            tags = list(idea.get("tags") or [])
        idea["tags"] = tags
        problems = validate_draft(idea, visible_facts, args.repo_root)
        idea["problems"] = problems
        idea["ok"] = not problems
        checked.append(idea)
    payload = {"ideas": checked, "fact_count": len(visible_facts)}
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
