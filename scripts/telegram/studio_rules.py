"""Правила Шифу к постам: «в постах не пиши про …» — запоминает код, а не обещает модель.

Разбор 08.11: Шифу написал «в постах не нужно писать про пиары, ветки и детали из
репозитория и исью». Сообщение ушло в разговор, модель ответила «Приняли» — и ничего
не сохранилось: следующий черновик снова вышел с ``fix(dialog)``. По ADR-0148 фразу об
успехе строит код из результата: правило лежит в ``state["rules"]``, попадает в задание
каждого текста поста (``prompt_block``), а ответ Шифу пишет ``reply_text``.

Слова — подстроки в нижнем регистре, без регексов, как в ``studio_intent``.
"""

from __future__ import annotations

import time
from typing import Optional

RULE_WORDS = (
    "в постах не",
    "в постах пиши",
    "в постах всегда",
    "в постах больше",
    "в посты не",
    "не пиши про",
    "не пиши в пост",
    "больше не пиши",
    "не нужно писать",
    "не надо писать",
    "не упоминай",
    "впредь",
    "на будущее",
)
RULES_MAX = 20  # правил в задании модели: старые дальше не идут, чтобы не съесть окно
RULE_TEXT_MAX = 300  # знаков одного правила


def is_rule(text: str) -> bool:
    """Это правило к постам вообще, а не правка одного черновика."""
    low = " ".join((text or "").lower().split())
    return any(word in low for word in RULE_WORDS)


def remember(state: dict, text: str, now: Optional[float] = None) -> tuple:
    """Сохранить правило в состояние. (правило, новое ли). Тот же текст второй раз не дублируется."""
    rule = " ".join((text or "").split())[:RULE_TEXT_MAX]
    rules = state.setdefault("rules", [])
    if any(item.get("text") == rule for item in rules):
        return rule, False
    rules.append({"text": rule, "at": time.time() if now is None else now})
    return rule, True


def active(state: dict) -> list:
    """Тексты правил, свежие последними; не больше RULES_MAX."""
    return [item["text"] for item in (state.get("rules") or []) if item.get("text")][-RULES_MAX:]


def prompt_block(rules: list) -> str:
    """Блок задания модели. Пусто — правил нет."""
    if not rules:
        return ""
    lines = "\n".join(f"- {rule}" for rule in rules)
    return f"ПРАВИЛА ШИФУ К ПОСТАМ (обязательны, важнее общего стиля):\n{lines}\n\n"


def reply_text(rule: str, is_new: bool, post_id: str, count: int, several: bool = False) -> str:
    """Ответ Шифу строит код: что сохранено и что дальше будет сделано."""
    head = f"Записал правило к постам: «{rule}»." if is_new else f"Это правило уже записано: «{rule}»."
    head += (
        f" Правил сейчас: {count}; каждое идёт в задание каждого текста поста. Кодом проверяются только"
        " номера PR и issue, ветки, типы коммитов и файлы репозитория — остальное проверь глазами."
    )
    if post_id:
        return head + f" Черновик {post_id} переписываю по нему — индикатор ниже."
    if several:
        return head + " Черновиков на обсуждении несколько — ответь на нужный словом «перепиши», перепишу по правилам."
    return head + " Черновика на обсуждении нет — правило сработает на следующем."
