"""Ответ студии на вопрос Шифу в личке.

Фаза обсуждения: Шифу нажал «Обсудить» или написал в личку. Отвечает МОДЕЛЬ БЕЗ ИНСТРУМЕНТОВ:
прямой вызов MiniMax (studio_text.call_minimax) по узкой схеме {"answer": "..."}.

Почему не агент hermes: `hermes -t ""` НЕ выключает инструменты. В hermes-agent
(cli.py, _build_cli_from_args) пустой --toolsets означает «набор по умолчанию»
(coding posture) — с файлами и терминалом. Такой агент дотягивался бы до
docker/studio/.env.secrets (токен бота, ключ MiniMax). Проверено по коду hermes-agent
на builder 06.11, поэтому агента в этом пути нет вовсе.

Безопасность ответа (ADR-0148: код решает, модель говорит):
  * у модели нет ни инструментов, ни окружения — только текст промпта;
  * ответ проверяется кодом: без html-разметки, без внутренних IP и ключей
    (channel_lib.FORBIDDEN, в том числе JWT-ключ MiniMax), ограниченной длины.

Контекст берём из state.json: идея на обсуждении и её факты — чтобы агент отвечал
по тому же, о чём идёт речь, и не выдумывал.

Запуск:
  scripts/telegram/studio_reply.py --message "текст вопроса" [--idea-id i20261106-1]
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "telegram"))

import channel_lib as lib  # noqa: E402

DEFAULT_STATE = REPO_ROOT / "docker" / "studio" / "state.json"

# Признаки секретов в ответе. Ключ MiniMax — это JWT: "eyJ...". Без такой проверки
# он уехал бы в чат вместе с ответом.
SECRET_PATTERNS = (
    "eyJ",
    "sk-",
    "ghp_",
    "github_pat_",
    "AAG",
)

MAX_ANSWER_CHARS = 900
MIN_ANSWER_CHARS = 2

# Внутренние адреса и ключи в ответе — не место. Список один на канал: channel_lib.FORBIDDEN.
FORBIDDEN = lib.FORBIDDEN

SYSTEM_PROMPT = (
    "Ты — студия Telegram-канала @robbox_project (проект робота РОББОКС). "
    "Отвечаешь Шифу, хозяину проекта, в его личке.\n"
    "Правила ответа: по-русски, КОРОТКО (2–5 предложений, до 400 знаков), просто, "
    "как мастер в мастерской; без канцелярита и без рекламы; «мы» и «РОББОКС».\n"
    "Это переписка в Telegram: пиши как живой человек, а не как отчёт. "
    "Если вопрос длинный и по делу — ответь по сути и предложи один следующий шаг.\n"
    "Опирайся только на факты идеи и на репозиторий. Если данных не хватает — скажи прямо, "
    "чего не хватает. Не выдумывай цифры и имена.\n"
    "Ты только отвечаешь словами: ты НЕ ищешь, НЕ запускаешь, НЕ скачиваешь и НЕ обещаешь это сделать — "
    "таких возможностей у тебя нет. Нужны данные или файл — назови, какие именно; их кладёт координатор "
    "в docs/channel/media/ через PR, или Шифу присылает файл в личку.\n"
    "Действия над черновиком делает студия по кнопкам, а не ты. Если к вопросу подходит одно из них — "
    "предложи его полем offer (кнопку покажет код), а в тексте НЕ пиши «сделаю», «можно сделать», «запущу»: "
    "shorter — короче, simpler — проще, angle — другая подача, media — другая картинка, "
    "animate — анимация (GIF), video — ролик (MP4, например «до/после» по записям прогонов). "
    "Чего нет в этом списке, студия не умеет — так и скажи: «так пока не умею».\n"
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
    lines.append(
        'Верни СТРОГО JSON: {"answer": "текст ответа", "offer": "none|shorter|simpler|angle|media|animate|video"}'
    )
    return "\n".join(lines)


# Что модель может предложить кнопкой (узкая схема, ADR-0148): действие делает код, модель только предлагает.
OFFERS = ("shorter", "simpler", "angle", "media", "animate", "video")


def ask_model(prompt: str, timeout: int = 120, call=None) -> str:
    """Ответ модели без инструментов. Пустая строка — модель не ответила или схема не та."""
    return ask_model_full(prompt, timeout, call)[0]


def ask_model_full(prompt: str, timeout: int = 120, call=None) -> tuple:
    """(ответ, предложение кнопкой). Предложение вне OFFERS отбрасывается кодом."""
    if call is None:
        import studio_text

        call = studio_text.call_minimax
    result = call(prompt, timeout=timeout)
    answer = result.get("answer") if isinstance(result, dict) else None
    offer = result.get("offer") if isinstance(result, dict) else None
    return (answer.strip() if isinstance(answer, str) else ""), (offer if offer in OFFERS else "")


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
    if any(marker in answer for marker in SECRET_PATTERNS):
        problems.append("в ответе похоже на ключ")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--message", required=True, help="вопрос Шифу")
    ap.add_argument("--idea-id", default="", help="id идеи, если нужно обсудить конкретную")
    ap.add_argument("--state", type=Path, default=DEFAULT_STATE)
    ap.add_argument("--timeout", type=int, default=120)
    ap.add_argument("--json", action="store_true", help='вывод {"answer", "offer"}: кнопку строит вызывающий')
    args = ap.parse_args()

    state = load_state(args.state)
    idea = discussed_idea(state, args.idea_id)
    prompt = build_prompt(args.message, idea, state)
    raw, offer = ask_model_full(prompt, timeout=args.timeout)
    answer = clean_answer(raw)
    if not answer:
        print("Модель не ответила. Попробуем позже — идея осталась на обсуждении.", file=sys.stderr)
        return 1
    problems = answer_problems(answer)
    if problems:
        # Плохой ответ в чат не отправляем: лучше тишина, чем внутренний IP в посте.
        print(f"Ответ не прошёл проверку: {'; '.join(problems)}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps({"answer": answer, "offer": offer}, ensure_ascii=False))
        return 0
    print(answer)
    return 0


if __name__ == "__main__":
    sys.exit(main())
