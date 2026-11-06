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
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPO = "krikz/rob_box_project"
DEFAULT_STATE = REPO_ROOT / "docker" / "studio" / "state.json"

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
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (ValueError, OSError):
        return {}


def save_last_session(path: Path, when: str) -> None:
    state = load_state(path)
    state["last_session"] = when
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default=os.environ.get("GH_REPO", DEFAULT_REPO))
    ap.add_argument("--state", type=Path, default=DEFAULT_STATE)
    ap.add_argument("--since", help="метка прошлой сессии (ISO); по умолчанию — из state.json или сутки назад")
    ap.add_argument("--no-save", action="store_true", help="не обновлять метку сессии (пробный запуск)")
    args = ap.parse_args()

    state = load_state(args.state)
    since = args.since or state.get("last_session")
    if not since:
        since = (datetime.now(timezone.utc) - timedelta(days=1)).strftime("%Y-%m-%dT%H:%M:%SZ")

    facts = []
    facts += collect_merged_prs(args.repo, since)
    facts += collect_closed_issues(args.repo, since)
    facts += collect_runs(args.repo, since[:10])
    facts += collect_new_files(REPO_ROOT, since)

    if not args.no_save:
        save_last_session(args.state, datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"))

    payload = {
        "since": since,
        "repo": args.repo,
        "facts": facts,
        "visible_facts": [f for f in facts if f.get("visible")],
        "fact_count": len(facts),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
