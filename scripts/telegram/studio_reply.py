"""Ответ студии на вопрос Шифу в личке.

Фаза обсуждения: Шифу нажал «Обсудить» и задал вопрос. На вопрос должен отвечать агент,
а не скрипт. Агент запускается одноразово (hermes -z) в профиле studio.

Безопасность ответа (ADR-0148: код решает, модель говорит):
  * агенту НЕ достаётся токен бота студии — иначе агент с инструментами мог бы
    сам опубликовать пост в канал (минуя одобрение Шифу);
  * инструменты ограничены: ни terminal, ни cron — только чтение файлов и веб;
  * ответ проверяется кодом: без html-разметки, без внутренних IP и ключей,
    ограниченной длины; из stdout берётся только текст ответа, не хвост лога.

Контекст берём из state.json: идея на обсуждении и её факты — чтобы агент отвечал
по тому же, о чём идёт речь, и не выдумывал.

Запуск:
  scripts/telegram/studio_reply.py --message "текст вопроса" [--idea-id i20261106-1]
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STATE = REPO_ROOT / "docker" / "studio" / "state.json"
DEFAULT_PROFILE = "studio"

# Тулсеты агента: чтение файлов и веб. Без terminal — иначе агент может выполнить
# публикацию в канал сам, минуя кнопку «Опубликовать».
AGENT_TOOLSETS = "file,web"

# Переменные, которых агент не должен видеть: с ними он может публиковать в канал.
SECRET_ENV_BLOCKLIST = (
    "TG_STUDIO_BOT_TOKEN",
    "TG_BOT_TOKEN",
    "TG_CHANNEL_TOKEN",
)

MAX_ANSWER_CHARS = 900
MIN_ANSWER_CHARS = 2

# Внутренние адреса и ключи в ответе — не место.
FORBIDDEN = (
    (re.compile(r"\b10\.1\.1\.\d{1,3}\b"), "внутренний IP"),
    (re.compile(r"\b192\.168\.\d{1,3}\.\d{1,3}\b"), "внутренний IP"),
    (re.compile(r"\bsk-[A-Za-z0-9_-]{20,}"), "ключ API"),
    (re.compile(r"\bgh[pousr]_[A-Za-z0-9]{20,}"), "токен GitHub"),
    (re.compile(r"\bAAG[A-Za-z0-9_-]{30,}"), "токен бота"),
)

SYSTEM_PROMPT = (
    "Ты — студия Telegram-канала @robbox_project (проект робота РОББОКС). "
    "Отвечаешь Шифу, хозяину проекта, в его личке.\n"
    "Правила ответа: по-русски, КОРОТКО (2–5 предложений, до 400 знаков), просто, "
    "как мастер в мастерской; без канцелярита и без рекламы; «мы» и «РОББОКС».\n"
    "Это переписка в Telegram: пиши как живой человек, а не как отчёт. "
    "Если вопрос длинный и по делу — ответь по сути и предложи один следующий шаг.\n"
    "Опирайся только на факты идеи и на репозиторий. Если данных не хватает — скажи прямо, "
    "чего не хватает. Не выдумывай цифры и имена.\n"
    "Ответ — только текст ответа, без вступлений вроде «Конечно» и без markdown-заголовков. "
    "Без разметки: Telegram сам покажет текст как есть."
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


def clean_env() -> dict:
    """Окружение для агента без токена бота студии."""
    import os

    env = dict(os.environ)
    for name in SECRET_ENV_BLOCKLIST:
        env.pop(name, None)
    return env


def ask_agent(prompt: str, profile: str = DEFAULT_PROFILE, timeout: int = 300) -> str:
    """Одноразовый запуск агента профиля. Пустой ответ — значит агент не справился."""
    cmd = ["hermes", "--profile", profile, "-t", AGENT_TOOLSETS, "-z", prompt]
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=clean_env())
    except subprocess.TimeoutExpired:
        return ""
    except OSError:
        return ""
    # Ответ — только stdout. stderr в чат не идёт: там может быть хвост лога
    # или предупреждение, а Шифу должен получить ответ, а не мусор.
    return (done.stdout or "").strip()


def strip_log_noise(text: str) -> str:
    """Убрать служебные строки, которые агент печатает вместе с ответом."""
    lines = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        lowered = line.lower()
        if lowered.startswith(("hermes:", "warning:", "error:", "traceback", "usage:", "session")):
            continue
        if line.startswith(("╭", "│", "╰", "─", "┌", "└")):
            continue
        lines.append(line)
    return "\n".join(lines).strip()


def clean_answer(text: str) -> str:
    """Привести ответ к виду, безопасному для отправки в Telegram.

    Убираем разметку (ответ идёт обычным текстом), режем лишнее и проверяем,
    что в нём нет внутренних адресов и ключей.
    """
    answer = strip_log_noise(text)
    # html-теги в ответе агента — мусор: текст отправляется как есть.
    answer = re.sub(r"<[^>]+>", "", answer)
    answer = re.sub(r"[`*_]{1,3}", "", answer)
    answer = answer.strip()
    if len(answer) > MAX_ANSWER_CHARS:
        answer = answer[:MAX_ANSWER_CHARS].rsplit(" ", 1)[0] + "…"
    return answer


def answer_problems(answer: str) -> list:
    """Проблемы ответа, из-за которых его нельзя отправлять Шифу."""
    problems = []
    if len(answer) < MIN_ANSWER_CHARS:
        problems.append("ответ слишком короткий")
    for pattern, what in FORBIDDEN:
        if pattern.search(answer):
            problems.append(f"в ответе {what}")
    return problems


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
    answer = clean_answer(ask_agent(prompt, profile=args.profile, timeout=args.timeout))
    if not answer:
        print("Агент не ответил. Попробуем позже — идея осталась на обсуждении.", file=sys.stderr)
        return 1
    problems = answer_problems(answer)
    if problems:
        # Плохой ответ в чат не отправляем: лучше тишина, чем внутренний IP в посте.
        print(f"Ответ не прошёл проверку: {'; '.join(problems)}", file=sys.stderr)
        return 2
    print(answer)
    return 0


if __name__ == "__main__":
    sys.exit(main())
