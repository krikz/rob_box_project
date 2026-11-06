"""Сбор фактов для сессии анализа студии: что изменилось в проекте с прошлой сессии.

Факты собирает код (ADR-0148: модель не решает, что правда), модель потом пишет по ним текст.
Каждый факт — это утверждение плюс источник: PR, issue, run_id или файл:строка. Без источника
факт не попадает в выдачу: текст без проверяемого источника в канал не идёт.

Метка прошлой сессии лежит в state.json (last_session). Запуск без сети возвращает пустой список
и код 0 — сессия в этом случае просто скажет «материала нет».

Запуск:
  scripts/telegram/studio_facts.py [--state путь] [--repo krikz/rob_box_project]
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import studio_state as st

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPO = "krikz/rob_box_project"
DEFAULT_STATE = REPO_ROOT / "docker" / "studio" / "state.json"

# Сколько раз факт можно предложить, прежде чем он станет «отложенным».
# Три попытки — достаточно, чтобы Шифу успел ответить, и достаточно мало,
# чтобы непроизносимое событие не мусорило в ленте.
MAX_ATTEMPTS = 3

# Какие темы считаются «видимым изменением» — по ним есть что показать (бриф, шаг 2).
# Пишем и словами («навигация»), и scope'ами в скобках («fix(nav): …») — в заголовках
# PR тема живёт во втором, а в теле — в первом.
VISIBLE_TOPICS = (
    "навигац",
    "nav2",
    "nav)",
    "nav:",
    "лицо",
    "face)",
    "face:",
    "led)",
    "led:",
    "защит",
    "музык",
    "music)",
    "music:",
    "голос",
    "voice)",
    "voice:",
    "tts",
    "stt",
    "sim)",
    "sim:",
    "симуляц",
    "gazebo",
    "камер",
    "camera)",
    "микрофон",
    "wake",
    "dialog)",
    "e2e)",
)

# Технические PR не показываем читателю канала: чистка зависимостей, CI, документация,
# обновление сабмодуля. Их темы пересекаются с продуктовыми (путь voice/ у deps bump),
# поэтому тип правки отсекаем раньше, чем смотрим на темы.
BORING_PREFIXES = (
    "chore(",
    "ci:",
    "ci(",
    "docs(",
    "docs:",
    "build:",
    "test(",
    "test:",
    "refactor(",
    "style:",
)


def is_boring(title: str) -> bool:
    """Техническая правка без видимого для читателя смысла."""
    low = (title or "").strip().lower()
    return low.startswith(BORING_PREFIXES)


def run_gh(args: list, repo: str) -> list:
    """gh cli → список dict'ов. Ошибка или нет сети — пустой список, не исключение."""
    cmd = ["gh"] + args
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired):
        return []
    if proc.returncode != 0:
        return []
    try:
        data = json.loads(proc.stdout or "[]")
    except ValueError:
        return []
    return data if isinstance(data, list) else []


def visible(text: str) -> bool:
    low = (text or "").lower()
    return any(topic in low for topic in VISIBLE_TOPICS)


def fact_key(fact: dict) -> str:
    """Устойчивый ключ факта: по нему видно, что событие уже показано.

    Ключ переживает перезапуск сессии и пересборку фактов, в отличие от номера в списке.
    """
    kind = fact.get("kind", "")
    if kind == "merged_pr":
        match = re.search(r"PR #(\d+)", fact.get("claim", ""))
        return f"pr:{match.group(1)}" if match else f"pr:{fact.get('claim', '')[:40]}"
    if kind == "closed_issue":
        match = re.search(r"#(\d+)", fact.get("claim", ""))
        return f"issue:{match.group(1)}" if match else f"issue:{fact.get('claim', '')[:40]}"
    if kind == "run":
        match = re.search(r"run_id (\d+)", fact.get("source", ""))
        return f"run:{match.group(1)}" if match else f"run:{fact.get('source', '')[:40]}"
    if kind == "adr":
        return f"adr:{fact.get('source', '')[:60]}"
    if kind == "media":
        return f"media:{fact.get('source', '')[:80]}"
    if kind == "decisions":
        return "decisions:shifu"
    return f"{kind}:{fact.get('source', fact.get('claim', ''))[:50]}"


def covered_from_posts(queue_dir: Path, facts: list) -> set:
    """Ключи фактов, которые уже показаны в опубликованных постах.

    Пост может сослаться на правку («после правки #4487») или на файл
    («docs/research/…md»). Такой факт уже рассказан читателю — предлагать его снова нельзя.
    Ищем не по теме, а по прямой ссылке: тема может повторяться, а событие — нет.
    """
    if not queue_dir.is_dir():
        return set()
    cited_numbers = set()
    cited_paths = set()
    for post in sorted(queue_dir.glob("*.md")):
        text = post.read_text(encoding="utf-8", errors="ignore")
        head, _, body = text.partition("\n---\n")
        if "status: posted" not in head and "status: approved" not in head:
            continue  # черновик ещё не опубликован — он не закрывает событие
        cited_numbers.update(re.findall(r"#(\d{3,6})", body))
        cited_paths.update(re.findall(r"docs/[\w./-]+\.(?:md|py|yaml|yml|gif|png|jpe?g|mp4)", body))

    if not cited_numbers and not cited_paths:
        return set()

    # Номер в посте может быть номером PR или issue: PR и issue живут в разных
    # пространствах, а по названию не отличить. Различаем по тому, есть ли такой PR
    # среди собранных фактов: если PR есть — это PR, если нет — это issue.
    pr_numbers = {f["key"].split(":", 1)[1] for f in facts if f.get("key", "").startswith("pr:")}

    covered = set()
    for fact in facts:
        key = fact.get("key")
        if not key:
            continue
        source = fact.get("source", "") or ""
        if source and source in cited_paths:
            covered.add(key)
            continue
        match = re.search(r"#(\d{3,6})", fact.get("claim", ""))
        if match and match.group(1) in cited_numbers:
            number = match.group(1)
            if f"pr:{number}" == key or (f"pr:{number}" not in pr_numbers and f"issue:{number}" == key):
                covered.add(key)
    return covered


def collect_merged_prs(repo: str, since: str) -> list:
    """Влитые в develop PR с прошлой сессии: заголовок, тело, изменённые пути."""
    prs = run_gh(
        [
            "pr",
            "list",
            "--repo",
            repo,
            "--state",
            "merged",
            "--search",
            f"merged:>={since}",
            "--json",
            "number,title,body,mergedAt,files,url",
            "--limit",
            "40",
        ],
        repo,
    )
    facts = []
    for pr in prs:
        paths = [f.get("path", "") for f in (pr.get("files") or [])]
        title = pr.get("title", "")
        body = (pr.get("body") or "")[:600]
        facts.append(
            {
                "kind": "merged_pr",
                "claim": f"PR #{pr.get('number')} влит в develop: {title}",
                "source": f"PR #{pr.get('number')} ({pr.get('url', '')})",
                "merged_at": pr.get("mergedAt"),
                "paths": paths[:20],
                "visible": (not is_boring(title)) and visible(title + " " + body + " " + " ".join(paths)),
            }
        )
    return facts


def collect_closed_issues(repo: str, since: str) -> list:
    """Закрытые issue с комментариями-доказательствами: только те, где в комментариях есть цифры или run_id."""
    issues = run_gh(
        [
            "issue",
            "list",
            "--repo",
            repo,
            "--state",
            "closed",
            "--search",
            f"closed:>={since}",
            "--json",
            "number,title,body,closedAt,url,comments",
            "--limit",
            "40",
        ],
        repo,
    )
    facts = []
    for issue in issues:
        comments = issue.get("comments") or []
        evidence = []
        for comment in comments:
            body = comment.get("body") or ""
            if "run_id" in body or "run " in body.lower() or any(ch.isdigit() for ch in body):
                evidence.append(body[:300])
        if not evidence:
            continue
        facts.append(
            {
                "kind": "closed_issue",
                "claim": f"Issue #{issue.get('number')} закрыт: {issue.get('title')}",
                "source": f"issue #{issue.get('number')} ({issue.get('url', '')})",
                "closed_at": issue.get("closedAt"),
                "evidence": evidence[:3],
                "visible": visible(issue.get("title", "")),
            }
        )
    return facts


def collect_runs(repo: str, since: str) -> list:
    """Прогоны Actions: только итоговые цифры, без персональных данных."""
    runs = run_gh(
        [
            "run",
            "list",
            "--repo",
            repo,
            "--created",
            f">={since}",
            "--json",
            "databaseId,name,conclusion,status,createdAt,url",
            "--limit",
            "30",
        ],
        repo,
    )
    facts = []
    for run in runs:
        name = run.get("name", "")
        if not any(key in name for key in ("E2E", "Voice", "Build All", "Deploy", "Tests")):
            continue
        facts.append(
            {
                "kind": "run",
                "claim": f"Прогон «{name}»: {run.get('conclusion') or run.get('status')}",
                "source": f"run_id {run.get('databaseId')} ({run.get('url', '')})",
                "created_at": run.get("createdAt"),
                "visible": run.get("conclusion") in ("success", "failure"),
            }
        )
    return facts


def collect_new_files(repo_root: Path, since: str) -> list:
    """Новые ADR, записи в shifu-decisions.md и медиа в репозитории."""
    facts = []
    adr_dir = repo_root / "docs" / "adr"
    decisions = repo_root / "docs" / "shifu-decisions.md"
    cutoff = datetime.fromisoformat(since.replace("Z", "+00:00"))
    if adr_dir.is_dir():
        for path in sorted(adr_dir.glob("*.md")):
            mtime = datetime.fromtimestamp(path.stat().st_mtime, tz=timezone.utc)
            if mtime > cutoff:
                facts.append(
                    {
                        "kind": "adr",
                        "claim": f"Новый ADR: {path.name}",
                        "source": f"{path.relative_to(repo_root)}",
                        "visible": True,
                    }
                )
    if decisions.is_file():
        mtime = datetime.fromtimestamp(decisions.stat().st_mtime, tz=timezone.utc)
        if mtime > cutoff:
            facts.append(
                {
                    "kind": "decisions",
                    "claim": "Обновились решения Шифу (docs/shifu-decisions.md)",
                    "source": "docs/shifu-decisions.md",
                    "visible": True,
                }
            )
    for media_dir, patterns in (
        ("docs/assets/hil", ("*.gif", "*.mp4")),
        ("docs/channel/media", ("*.png", "*.jpg", "*.gif", "*.mp4")),
    ):
        folder = repo_root / media_dir
        if not folder.is_dir():
            continue
        for pattern in patterns:
            for path in sorted(folder.rglob(pattern)):
                mtime = datetime.fromtimestamp(path.stat().st_mtime, tz=timezone.utc)
                if mtime > cutoff:
                    facts.append(
                        {
                            "kind": "media",
                            "claim": f"Новое медиа: {path.relative_to(repo_root)}",
                            "source": str(path.relative_to(repo_root)),
                            "visible": True,
                        }
                    )
    return facts


def load_state(path: Path) -> dict:
    """Совместимость: состояние читает общий модуль studio_state."""
    return st.load_state(path)


def _unused_load_state(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (ValueError, OSError):
        return {}


def save_last_session(path: Path, when: str) -> None:
    state = st.load_state(path)
    state["last_session"] = when
    st.save_state(path, state)


def save_state(path: Path, state: dict) -> None:
    """Сохранить состояние атомарно."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default=os.environ.get("GH_REPO", DEFAULT_REPO))
    ap.add_argument("--state", type=Path, default=DEFAULT_STATE)
    ap.add_argument("--since", help="метка прошлой сессии (ISO); по умолчанию — из state.json или сутки назад")
    ap.add_argument("--no-save", action="store_true", help="не обновлять метку сессии (пробный запуск)")
    args = ap.parse_args()

    state = st.load_state(args.state)
    since = args.since or state.get("last_session")
    if not since:
        since = (datetime.now(timezone.utc) - timedelta(days=1)).strftime("%Y-%m-%dT%H:%M:%SZ")

    facts = []
    facts += collect_merged_prs(args.repo, since)
    facts += collect_closed_issues(args.repo, since)
    facts += collect_runs(args.repo, since[:10])
    facts += collect_new_files(REPO_ROOT, since)

    state = st.load_state(args.state)
    # Ключ каждому факту: по нему видно, что событие уже показано.
    for fact in facts:
        fact["key"] = fact_key(fact)

    # Что уже закрыто — не предлагаем снова.
    closed = st.covered_keys(state)
    # Что уже рассказано в опубликованных постах — тоже не предлагаем.
    closed |= covered_from_posts(REPO_ROOT / "docs" / "channel" / "queue", facts)
    # Что предлагали, но не показали — возвращаем: событие не должно потеряться.
    for fact in st.pending_facts(state):
        key = fact.get("key")
        if key and key not in closed:
            facts.append(fact)

    # Событие, которое предлагали три раза и не взяли, — помечаем как отложенное:
    # дальше оно не мусорит в ленте, но и не пропадает из учёта.
    fresh = []
    deferred = []
    for fact in facts:
        if int(fact.get("attempts", 0)) >= MAX_ATTEMPTS and fact.get("key") not in closed:
            deferred.append(fact.get("key"))
            continue
        fresh.append(fact)

    if not args.no_save:
        state["deferred"] = sorted(set(state.get("deferred", [])) | set(deferred))
        st.save_state(args.state, state)

    payload = {
        "since": since,
        "repo": args.repo,
        "facts": fresh,
        "visible_facts": [f for f in fresh if f.get("visible")],
        "fact_count": len(fresh),
        "closed_count": len(closed),
        "deferred": sorted(set(deferred)),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
