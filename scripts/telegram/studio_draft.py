#!/usr/bin/env python3
"""Собрать готовый пост из фактов: тема → текст → проверка кодом.

По решению Шифу (06.11) на обсуждение уходит готовый пост, а не план
и не список идей. Модель пишет текст, код проверяет: числа и имена —
по фактам, разметку — по правилам Telegram HTML.

Вход:  facts.json (собран studio_facts.py)
Выход: post.json — заготовка поста, готовый к отправке и кнопкам правки.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import studio_text as st  # noqa: E402

# Какой вид факта взять за основу поста. Порядок — по «плотности» события:
# влитый PR говорит читателю больше, чем закрытый issue.
KIND_PRIORITY = ("merged_pr", "media", "adr", "decisions", "run", "closed_issue")


def pick_fact(facts: list) -> dict:
    """Выбрать факт-тему для поста."""
    by_kind = {}
    for fact in facts:
        by_kind.setdefault(fact.get("kind") or "", []).append(fact)
    for kind in KIND_PRIORITY:
        for fact in by_kind.get(kind, []):
            claim = (fact.get("claim") or "").strip()
            if len(claim) < 20:
                continue
            return fact
    for fact in facts:
        if len((fact.get("claim") or "").strip()) >= 20:
            return fact
    return facts[0] if facts else {}


def pick_media(facts: list) -> str:
    """Взять готовый материал из develop, если он есть."""
    for fact in facts:
        if fact.get("kind") == "media" and fact.get("source"):
            return fact["source"]
    return ""


def build_post(facts: list, model: str = st.DEFAULT_MODEL, repo_root: Path = st.REPO_ROOT) -> dict:
    """Собрать готовый пост: факт → идея → текст → проверка."""
    if not facts:
        return {"error": "нет фактов"}
    fact = pick_fact(facts)
    if not fact:
        return {"error": "нечего показывать"}
    claim = (fact.get("claim") or "").strip()
    title = claim[:90].rstrip(" .,:;") if claim else "Что нового в проекте"
    idea = {
        "title": title,
        # «Что показать» — сам факт: пост не должен выдумывать сюжет.
        "show": claim,
        "deadline": "",
        "media_ref": pick_media(facts),
        "fact_keys": [fact.get("key")] if fact.get("key") else [],
    }
    # Черновик уходит всегда с картинкой (решение Шифу 06.11): лимит — подпись 1024 знака.
    produced = st.produce_post(idea, [fact], model=model, repo_root=repo_root, has_media=True)
    attempts = 1
    if not produced.get("error") and not produced.get("ok"):
        # Бриф: «не прошло — один повтор с причиной». Причину строит код (problems).
        reason = "; ".join((produced.get("problems") or [])[:4])
        instruction = f"Прошлая версия не прошла проверку кодом: {reason}. Исправь именно это."
        idea["current_body"] = produced.get("body_html") or ""
        produced = st.produce_post(
            idea, [fact], model=model, repo_root=repo_root, instruction=instruction, has_media=True
        )
        attempts = 2
    if produced.get("error"):
        return {"error": produced["error"], "attempts": attempts}
    if not produced.get("ok"):
        return {"error": "не прошёл проверку", "problems": produced.get("problems") or [], "attempts": attempts}
    return {
        "title": title,
        "body_html": produced.get("body_html") or "",
        "tags": produced.get("tags") or [],
        "media_ref": idea["media_ref"],
        "fact_keys": idea["fact_keys"],
        "show": claim,
        "source": fact.get("source", ""),
        "why": produced.get("why") or "",
        "why_problems": produced.get("why_problems") or [],
        "checks": produced.get("checks") or {},
        "model": produced.get("model") or model,
        "attempts": attempts,
    }


def post_id_for(post: dict, day: str = "") -> str:
    """Идентификатор поста задаёт код: p<день>-<NNN> по стабильному хэшу источника и темы.

    hash() в Python рандомизируется между процессами — поэтому md5.
    """
    if not day:
        from datetime import datetime, timezone

        day = os.environ.get("STUDIO_POST_DATE") or datetime.now(timezone.utc).strftime("%Y%m%d")
    digest = hashlib.md5((post.get("source", "") + post.get("title", "")).encode("utf-8")).hexdigest()
    return f"p{day}-{int(digest[:6], 16) % 900 + 100}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--facts", type=Path, required=True, help="файл фактов сессии")
    ap.add_argument("--out", type=Path, required=True, help="куда писать заготовку поста")
    ap.add_argument("--model", default=st.DEFAULT_MODEL)
    ap.add_argument("--repo-root", type=Path, default=st.REPO_ROOT)
    args = ap.parse_args()

    try:
        data = json.loads(args.facts.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        print(f"факты не прочитаны: {exc}", file=sys.stderr)
        return 1
    facts = data.get("visible_facts") or data.get("facts") or []
    post = build_post(facts, model=args.model, repo_root=args.repo_root)
    if post.get("error"):
        print(f"пост не собран: {post.get('error')} {post.get('problems') or ''}", file=sys.stderr)
        return 1

    # Идентификатор задаёт код, а не модель: у поста должен быть один id.
    post_id = post_id_for(post)
    post["id"] = post_id
    post["status"] = "draft"

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"post": post}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"  пост собран: {post_id} ({len(post['body_html'])} знаков)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
