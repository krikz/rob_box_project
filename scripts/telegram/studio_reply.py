"""Ответ студии на вопрос Шифу в личке.

Фаза обсуждения: Шифу нажал «Обсудить» и задал вопрос. На вопрос должен отвечать агент,
а не скрипт. Агент запускается одноразово (hermes -z) в профиле studio: у него есть
инструменты, репозиторий и правила канала.

Контекст берём из state.json: идея на обсуждении и её факты — чтобы агент отвечал
по тому же, о чём идёт речь, и не выдумывал.

Запуск:
  scripts/telegram/studio_reply.py --message "текст вопроса" [--idea-id i20261106-1]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STATE = REPO_ROOT / "docker" / "studio" / "state.json"
DEFAULT_PROFILE = "studio"

SYSTEM_PROMPT = (
    "Ты — студия Telegram-канала @robbox_project (проект робота РОББОКС). "
    "Отвечаешь Шифу, хозяину проекта, в его личке.\n"
    "Правила ответа: по-русски, КОРОТКО (2–5 предложений, до 400 знаков), просто, "
    "как мастер в мастерской; без канцелярита и без рекламы; «мы» и «РОББОКС».\n"
    "Это переписка в Telegram: пиши как живой человек, а не как отчёт. "
    "Если вопрос длинный и по делу — ответь по сути и предложи один следующий шаг.\n"
    "Опирайся только на факты идеи и на репозиторий. Если данных не хватает — скажи прямо, "
    "чего не хватает. Не выдумывай цифры и имена.\n"
    "Ответ — только текст ответа, без вступлений вроде «Конечно» и без markdown-заголовков."
)


def conversation_history(state: dict, limit: int = 8) -> list:
    """Последние реплики диалога: без них каждый вопрос звучал бы с нуля."""
    notes = state.get("messages") or []
    history = []
    for note in notes[-limit:]:
        text = (note.get("text") or "").strip()
        if not text:
            continue
        role = "Шифу" if note.get("role", "shifu") == "shifu" else "Студия"
        history.append(f"{role}: {text}")
    return history


def load_state(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (ValueError, OSError):
        return {}


def discussed_idea(state: dict, idea_id: str = "") -> dict:
    """Идея, которая сейчас на обсуждении: указанная или последняя со статусом discuss."""
    ideas = state.get("ideas") or []
    if idea_id:
        for idea in ideas:
            if idea.get("id") == idea_id:
                return idea
        return {}
    for idea in reversed(ideas):
        if idea.get("status") == "discuss":
            return idea
    return {}


def build_prompt(message: str, idea: dict, state: dict) -> str:
    lines = [SYSTEM_PROMPT, ""]
    if idea:
        lines.append(f"Идея на обсуждении ({idea.get('id')}): {idea.get('title', '')}")
        facts = idea.get("facts") or []
        if facts:
            lines.append("Факты идеи (источники):")
            lines.extend(f"— {f}" for f in facts)
        body = (idea.get("body_html") or "").strip()
        if body:
            lines.append(f"Текущий черновик идеи: {body[:1200]}")
    else:
        lines.append("Идея на обсуждении не указана — ответь по общему состоянию студии.")
    history = conversation_history(state)
    if history:
        lines.append("")
        lines.append("Ход разговора:")
        lines.extend(history)
    lines.append("")
    lines.append(f"Вопрос Шифу: {message}")
    lines.append("")
    lines.append("Ответь.")
    return "\n".join(lines)


def ask_agent(prompt: str, profile: str = DEFAULT_PROFILE, timeout: int = 300) -> str:
    """Одноразовый запуск агента профиля. Пустой ответ — значит агент не справился."""
    cmd = ["hermes", "--profile", profile, "-z", prompt]
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return ""
    except OSError:
        return ""
    text = (done.stdout or "").strip()
    if not text:
        text = (done.stderr or "").strip()
    # Агент мог отдать хвост лога — берём последнюю непустую строку как ответ.
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    return lines[-1] if lines else ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--message", required=True, help="вопрос Шифу")
    ap.add_argument("--idea-id", default="", help="id идеи, если нужно обсудить конкретную")
    ap.add_argument("--state", type=Path, default=DEFAULT_STATE)
    ap.add_argument("--profile", default=DEFAULT_PROFILE)
    ap.add_argument("--timeout", type=int, default=300)
    args = ap.parse_args()

    state = load_state(args.state)
    idea = discussed_idea(state, args.idea_id)
    prompt = build_prompt(args.message, idea, state)
    answer = ask_agent(prompt, profile=args.profile, timeout=args.timeout)
    if not answer:
        print("Агент не ответил. Попробуем позже — идея осталась на обсуждении.", file=sys.stderr)
        return 1
    print(answer)
    return 0


if __name__ == "__main__":
    sys.exit(main())
