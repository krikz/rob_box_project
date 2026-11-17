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
from typing import Optional

import studio_develop as develop
import studio_material as material
import studio_media as smedia
import studio_state as st
import studio_topic as topic

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPO = "krikz/rob_box_project"
DEFAULT_STATE = REPO_ROOT / "docker" / "studio" / "state.json"

# Каталог кэша материалов студии: сюда складываются файлы, забранные с
# песочницы. Каталог внутри docker/studio/run — в git не попадает.
CACHE_DIR = Path(os.environ.get("STUDIO_TMP", REPO_ROOT / "docker" / "studio" / "run")) / "media" / "facts"

# Подпись прогона для читателя. Это же слово уходит в подвал инфографики
# («Источник: …»), поэтому оно обязано быть публичным текстом (channel_lib).
ROBOT_RUN_SOURCE = "журнал прогонов РОББОКС"

# Сколько раз факт показывали. Счётчик для учёта и для отчёта сессии.
#
# Раньше по нему факт уходил в «отложенное» после трёх показов. Теперь отказ —
# только явный («Мимо»); молчание снимает черновик по TTL (studio_state.
# expire_stale_drafts) и возвращает факт в pending. Поэтому число показов
# ограничивать нельзя: это не приговор факту, а счётчик внимания.
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
    # Склад и движение по нему — то, что читатель видит глазами (13.11:
    # «робот ездил по складу как пылесос» — а такие правки в фактах не было).
    "склад",
    "депо",
    "локализац",
    "карт",
    "маршрут",
    "заезд",
    "проезд",
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

# Незавершённая работа. Заголовок, начинающийся с «WIP », — это черновик:
# работу ещё не доделали, и показывать её читателю нельзя (ADR-0171, раздел 2а).
#
# Раньше отсев технических правок смотрел на начало названия, а «WIP
# refactor(e2e): …» начинается с «WIP », а не с «refactor(» — такая правка
# проходила в ленту как видимая, и семь изменённых файлов давали ей больше
# очков, чем содержательной правке о музыке в голосе. Отсев — здесь, на
# границе «что показывать», рядом с остальными техническими. Само правило
# живёт в studio_topic.is_draft: там же оно снимает очки черновику, чтобы
# такой факт не стал темой даже из старой ленты.


def is_boring(title: str) -> bool:
    """Техническая правка без видимого для читателя смысла."""
    low = (title or "").strip().lower()
    if topic.is_draft(title):
        return True
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
    if kind == "robot_run":
        # Префикс свой: «run:» уже занят прогонами Actions.
        run_id = str(fact.get("run_id") or "").strip()
        return f"robot_run:{run_id}" if run_id else f"robot_run:{fact.get('claim', '')[:40]}"
    if kind == "adr":
        return f"adr:{fact.get('source', '')[:60]}"
    if kind == "media":
        return f"media:{fact.get('source', '')[:80]}"
    if kind == "decisions":
        return "decisions:shifu"
    return f"{kind}:{fact.get('source', fact.get('claim', ''))[:50]}"


def lost_fact_keys(state: dict) -> set:
    """Ключи фактов, потерянных вместе с черновиком.

    Черновик протух (TTL) или был потерян, а его факт не вернулся в pending:
    в записи поста остался только ключ (`pr:4555`, разбор 11.11). Такой факт
    не закрыт (не показан, не отклонён) — значит событие ещё не рассказано
    и должно вернуться в ленту.
    """
    pending = {f.get("key") for f in state.get("pending", [])}
    closed = st.covered_keys(state)
    lost = set()
    for post_id, record in (state.get("posts") or {}).items():
        if not isinstance(record, dict):
            continue
        status = record.get("status")
        if status in ("approved", "rejected", "expired"):
            continue
        for key in record.get("fact_keys") or []:
            if key and key not in pending and key not in closed:
                lost.add(key)
    return lost


def refetch_fact_by_key(repo: str, key: str) -> dict:
    """Восстановить факт по ключу из GitHub.

    Нужен, когда событие потерялось: факт ушёл в черновик, а сам черновик
    протух или был потерян (разбор 11.11: `pr:4555` остался только ключом в
    записи поста — ни в pending, ни в covered, ни в rejected). По одному
    ключу `pr:4555` пост не собрать: нужен заголовок PR и его тело.
    """
    match = re.match(r"^(pr|issue):(\d+)$", key or "")
    if not match:
        return {}
    number = int(match.group(2))
    if match.group(1) == "pr":
        rows = run_gh(
            ["pr", "view", str(number), "--repo", repo, "--json", "number,title,body,mergedAt,files,url"],
            repo,
        )
        if not rows:
            return {}
        pr = rows[0] if isinstance(rows[0], dict) else {}
        paths = [f.get("path", "") for f in (pr.get("files") or [])]
        title = pr.get("title", "")
        body = (pr.get("body") or "")[:600]
        return {
            "kind": "merged_pr",
            "claim": f"PR #{pr.get('number')} влит в develop: {title}",
            "source": f"PR #{pr.get('number')} ({pr.get('url', '')})",
            "merged_at": pr.get("mergedAt"),
            "paths": paths[:20],
            "visible": (not is_boring(title)) and visible(title + " " + body + " " + " ".join(paths)),
            "key": key,
        }
    rows = run_gh(
        ["issue", "view", str(number), "--repo", repo, "--json", "number,title,body,closedAt,url,comments"],
        repo,
    )
    if not rows:
        return {}
    issue = rows[0] if isinstance(rows[0], dict) else {}
    comments = issue.get("comments") or []
    evidence = []
    for comment in comments:
        text = comment.get("body") or ""
        if "run_id" in text or "run " in text.lower() or any(ch.isdigit() for ch in text):
            evidence.append(text[:300])
    return {
        "kind": "closed_issue",
        "claim": f"Issue #{issue.get('number')} закрыт: {issue.get('title')}",
        "source": f"issue #{issue.get('number')} ({issue.get('url', '')})",
        "closed_at": issue.get("closedAt"),
        "evidence": evidence[:3],
        "visible": visible(issue.get("title", "")),
        "key": key,
    }


def partition_facts(facts: list, closed: set) -> tuple:
    """Разделить факты на показываемые и отложенные.

    Отложенным раньше становился факт, который показали `MAX_ATTEMPTS` раз и
    не взяли: молчание приравнивалось к отказу, и событие терялось (разбор
    11.11, факт `pr:4555` ушёл в deferred после трёх показов).

    Теперь отказ — только ЯВНЫЙ: товарищ Шифу нажал «Мимо», факт попал в
    `rejected`, а `rejected` входит в `closed` и отсекается здесь. Показ без
    ответа факт не наказывает: черновик протухает по TTL, факт возвращается
    в `pending`, и следующая сессия берётся за него снова.

    `deferred` остаётся в отчёте сессии для совместимости, но всегда пуст:
    отложенных по счётчику показов быть не должно.
    """
    fresh = [f for f in facts if not (f.get("key") and f.get("key") in closed)]
    return fresh, []


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


def _route_goals(text: str) -> int:
    """Сколько целей маршрута прошёл робот: строки журнала с меткой GOAL_START.

    Метка стоит вторым полем после времени: «08:24:17.873 GOAL_START M1 target=…».
    """
    goals = 0
    for line in (text or "").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[1] == "GOAL_START":
            goals += 1
    return goals


def _read_json(path: Path) -> dict:
    """Разбор json-файла. Нечитаемый файл — пустой словарь, а не исключение."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


# --- отладочные варианты одного замера ---------------------------------------
# Песочница держит рядом обычный прогон и его отладочные повторы:
# «depot4682_baseline_m1_20261111T224053Z» и «depot4682_baseline_m1_debug_…_r3».
# Отладочный повтор — это ТОТ ЖЕ замер, а не новое событие: как отдельные факты
# они дают шум (пять почти одинаковых тем «Робот прошёл склад: 1 цель…»).
#
# Поэтому событие — один замер: группируем прогоны по корню имени (без метки
# времени, номера ревизии и отладочной метки) и берём одного представителя
# группы. Настоящие прогоны depot4495 и depot4682 — разные корни, поэтому
# остаются каждый своим фактом (ADR-0171, раздел 2а).
DEBUG_MARKERS = ("debug", "diag", "recur")
DEBUG_SEGMENT_RE = re.compile(r"^(?:debug|diag|recur\d*)$")
RUN_TS_RE = re.compile(r"_\d{8}T\d{6}Z$")
RUN_REV_RE = re.compile(r"_r\d+$")

# Подпись события по типу прогона. Код знает, что за замер это
# (studio_material.KNOWN_RUNS), а не модель: у складского прогона подпись
# «Робот прошёл склад», у замера дрейфа локализации — своя. Жёсткое «склад»
# для любого прогона было бы враньём, если на песочнице появится не-складский
# замер (например shisyun_s1 — это дрейф, а не проход склада).
RUN_LEADS = {"drift": "Замер локализации на складе"}
RUN_LEAD_DEFAULT = "Робот прошёл склад"
# Роль прогона в паре «до/после» — слово из подписи прогона.
RUN_ROLE_WORDS = {"before": "до", "after": "после"}


def _run_root(run_id: str) -> tuple:
    """Корень замера прогона и признак отладочного повтора.

    «depot4682_baseline_m1_debug_20261111T230136Z_r3» и
    «depot4682_baseline_m1_20261111T224053Z» — один замер baseline_m1: разные
    только метка времени, номер ревизии и слово «debug».
    """
    name = RUN_REV_RE.sub("", run_id)
    name = RUN_TS_RE.sub("", name)
    parts = name.split("_")
    marked = False
    if parts and DEBUG_SEGMENT_RE.match(parts[-1]):
        parts.pop()
        marked = True
    return "_".join(parts), marked


def _run_lead(item: dict) -> str:
    """Подпись прогона для читателя — по типу замера, а не всегда «склад»."""
    topic = str(item.get("topic") or "")
    lead = RUN_LEADS.get(topic, RUN_LEAD_DEFAULT)
    role = RUN_ROLE_WORDS.get(str(item.get("role") or ""))
    if role:
        lead += f" ({role})"
    return lead


def _run_claim(numbers: dict, lead: str) -> str:
    """Утверждение о прогоне для читателя: только числа самого прогона.

    Идентификатор прогона сюда НЕ попадает: в посте восемь цифр подряд —
    это запрещённый идентификатор, модель перепишет его и проверка отбьёт текст.

    Время симуляции пишем ДВАЖДЫ — в секундах и в минутах («1406 с симуляции
    (23 минуты)»). Проверка чисел разрешает модели только числа из claim: без
    минут модель всё равно напишет «23 минуты симуляции» (так человечнее), и
    пост отклонится с «число 23 не найдено среди фактов», хотя по сути верен.
    """
    parts = []
    goals = numbers.get("goals")
    if goals:
        parts.append(f"{goals} {_plural(goals, 'цель маршрута', 'цели маршрута', 'целей маршрута')}")
    sim = numbers.get("sim_s")
    if sim:
        part = f"{int(sim)} с симуляции"
        minutes = _sim_minutes(int(sim))
        if minutes:
            part += f" ({minutes} {_plural(minutes, 'минута', 'минуты', 'минут')})"
        parts.append(part)
    snapshots = numbers.get("map_snapshots")
    if snapshots:
        parts.append(f"{snapshots} {_plural(snapshots, 'снимок карты', 'снимка карты', 'снимков карты')}")
    if not parts:
        return ""
    return f"{lead}: " + ", ".join(parts)


def _sim_minutes(seconds: int) -> int:
    """Секунды симуляции → минуты (округление до ближайшей минуты).

    1406 с — это 23 минуты: столько же напишет и модель, и человек.
    """
    if seconds < 60:
        return 0
    return int(seconds / 60 + 0.5)


def _plural(number: int, one: str, few: str, many: str) -> str:
    """Русское склонение числительного: 1 цель, 2 цели, 5 целей."""
    value = abs(int(number))
    if value % 10 == 1 and value % 100 != 11:
        return one
    if 2 <= value % 10 <= 4 and not 12 <= value % 100 <= 14:
        return few
    return many


def collect_robot_runs(gate=None, cache_dir: Optional[Path] = None) -> list:
    """Прогоны робота с песочницы — как факты студии (issue #4702).

    Разбор 13.11: за неделю робот ездил по складу, а в фактах прогонов не было
    вовсе — сборщики знали только про PR и issue. Теперь прогон с песочницы
    становится фактом: сколько целей маршрута пройдено, сколько секунд шла
    симуляция, сколько снимков карты снято.

    Песочница читается гейтом студии (решение Шифу 06.11: «студия сама берёт
    данные с хостов, только на чтение»). Ключа нет или хост не отвечает —
    это не поломка сессии: возвращаем пустой список и честно пишем причину.

    Числа берутся из файлов САМОГО прогона: ``maprec/meta.json`` (снимки карты,
    строки поз, секунды симуляции) и ``route.log`` (цели маршрута). Файлы
    кэшируются на диске, чтобы сессия не качала одно и то же каждый день.
    """
    facts = []
    source = ROBOT_RUN_SOURCE if smedia.is_public(ROBOT_RUN_SOURCE) else "журнал прогонов"
    try:
        catalog, reason = material.runs_catalog(gate or material.Gate())
    except Exception as exc:  # noqa: BLE001 — песочница не должна ронять сессию
        print(f"прогоны робота: каталог не прочитан ({type(exc).__name__}: {exc})", file=sys.stderr)
        return []
    if reason:
        print(f"прогоны робота: {reason}", file=sys.stderr)
        return []
    cache = Path(cache_dir or CACHE_DIR)
    gate_obj = gate or material.Gate()
    # Отладочные повторы одного замера — это один факт, а не пять. Группируем
    # прогоны по корню имени и оставляем по одному представителю на замер.
    groups: dict = {}
    for item in catalog:
        run_id = str(item.get("id") or "")
        kind = str(item.get("kind") or "")
        if not run_id or kind != "poses":
            # Каталог корня («depot») — это кадры камеры того же прогона, а не
            # отдельное событие: вторым фактом он был бы тем же прогоном дважды.
            continue
        if not material.NAME_RE.match(run_id):
            continue
        root, marked = _run_root(run_id)
        groups.setdefault(root, []).append((marked, run_id, item))
    for root in sorted(groups):
        members = groups[root]
        # Отладочные повторы отбрасываем: событие — один замер, а не его
        # повторы. Настоящий прогон (без меток debug/diag/recur) остаётся.
        clean = [member for member in members if not member[0]]
        if clean:
            # Из одного корня остаётся один представитель: самый свежий по
            # времени окончания, иначе порядок был бы случайным.
            chosen = sorted(clean, key=lambda member: member[1])[-1]
            facts.append(_robot_run_fact(chosen[2], gate_obj, cache, source))
            continue
        # Отладочных вариантов несколько: берём самый полный из них (по
        # времени симуляции), чтобы факт не потерялся, если обычного прогона
        # на песочнице уже нет.
        best = None
        for _marked, run_id, item in members:
            fact = _robot_run_fact(item, gate_obj, cache, source)
            if not fact:
                continue
            if best is None or _run_scale(fact) > _run_scale(best):
                best = fact
        if best:
            facts.append(best)
    return facts


def _run_scale(fact: dict) -> tuple:
    """Насколько прогон показателен: цели, секунды, снимки карты."""
    numbers = fact.get("numbers") or {}
    return (
        int(numbers.get("goals") or 0),
        int(numbers.get("sim_s") or 0),
        int(numbers.get("map_snapshots") or 0),
    )


def _robot_run_fact(item: dict, gate_obj, cache: Path, source: str) -> Optional[dict]:
    """Факт о прогоне с песочницы: числа САМОГО прогона, без его идентификатора.

    Пустой результат — прогон не удалось прочитать (нет meta.json и т.п.):
    сессия продолжает работать, причина уходит в stderr.
    """
    run_id = str(item.get("id") or "")
    local = cache / run_id
    meta_path = local / "meta.json"
    route_path = local / "route.log"
    end_path = local / "run_end.txt"
    base = f"{material.HIL_OUT}/{run_id}"
    try:
        if gate_obj.fetch(f"{base}/maprec/meta.json", meta_path):
            print(f"прогон {run_id}: meta.json не забрался", file=sys.stderr)
            return None
        meta = _read_json(meta_path)
        if not meta:
            print(f"прогон {run_id}: meta.json пустой или нечитаемый", file=sys.stderr)
            return None
        gate_obj.fetch(f"{base}/route.log", route_path)
        gate_obj.fetch(f"{base}/run_end.txt", end_path)
    except Exception as exc:  # noqa: BLE001 — один прогон не должен ронять сбор
        print(f"прогон {run_id}: файлы не забраны ({type(exc).__name__}: {exc})", file=sys.stderr)
        return None
    goals = 0
    if route_path.is_file():
        goals = _route_goals(route_path.read_text(encoding="utf-8", errors="ignore"))
    numbers = {
        "goals": goals,
        "map_snapshots": int(meta.get("map_snapshots") or 0),
        "pose_rows": int(meta.get("pose_rows") or 0),
        "sim_s": int(meta.get("sim_s") or 0),
    }
    claim = _run_claim(numbers, _run_lead(item))
    if not claim:
        return None
    ended = ""
    if end_path.is_file():
        ended = end_path.read_text(encoding="utf-8", errors="ignore").strip()
    fact = {
        "kind": "robot_run",
        "claim": claim,
        "source": source,
        # Идентификатор прогона и его числа живут ОТДЕЛЬНО от текста
        # факта: extract_fact_text показывает модели только claim,
        # source и evidence — идентификатор в пост не попадёт.
        "run_id": run_id,
        "run_label": str(item.get("label") or f"прогон {run_id}"),
        "numbers": numbers,
        "ended_at": ended,
        "visible": True,
    }
    # Ключ — здесь же, а не только в общем проходе main(): факт о прогоне
    # самодостаточен, и карточка цифр находит его по ключу поста.
    fact["key"] = fact_key(fact)
    return fact


PR_RE = re.compile(r"#(\d+)")


def pr_of(subject: str) -> str:
    """Номер PR из темы коммита.

    В конвенции репозитория номер PR — последний в скобках: «… (#4495) (#4508)»
    — тут PR это #4508, а #4495 — общий epic. Пусто — если PR не упомянут.
    """
    found = PR_RE.findall(subject or "")
    return found[-1] if found else ""


def collect_new_files_by_git(repo_root: Path, since: str) -> list:
    """Новые файлы по истории git, а не по mtime.

    В worktree на develop у всех файлов mtime равен моменту checkout — «новыми»
    выглядели бы все разом. История git отвечает честно: файл нов, если он
    добавлен после отметки прошлой сессии.
    """
    facts = []
    tracked = (
        ("docs/adr", ("*.md",)),
        ("docs/channel/media", ("*.png", "*.jpg", "*.gif", "*.mp4")),
        ("docs/assets/hil", ("*.gif", "*.mp4")),
    )
    done = subprocess.run(
        [
            "git",
            "log",
            f"--since={since}",
            "--name-status",
            "--pretty=format:%x00%s",
            "--diff-filter=A",
        ],
        cwd=str(repo_root),
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    if done.returncode != 0:
        return facts
    added = {}  # путь → тема коммита (из неё берём номер PR)
    subject = ""
    for line in done.stdout.splitlines():
        if "\x00" in line:  # начало коммита: тема сообщения
            subject = line.split("\x00", 1)[1].strip()
            continue
        parts = line.split("\t")
        if len(parts) >= 2 and parts[0].startswith("A"):
            path_text = parts[-1].strip()
            added.setdefault(path_text, subject)
    for path_text in sorted(added):
        path = repo_root / path_text
        if not path.is_file():
            continue
        if path_text == "docs/shifu-decisions.md":
            facts.append(
                {
                    "kind": "decisions",
                    "claim": "Обновились решения Шифу (docs/shifu-decisions.md)",
                    "source": "docs/shifu-decisions.md",
                    "visible": True,
                }
            )
            continue
        for media_dir, patterns in tracked:
            if not path_text.startswith(media_dir + "/"):
                continue
            if not any(path.match(pattern) for pattern in patterns):
                continue
            facts.append(
                {
                    "kind": "media",
                    "claim": f"Новое медиа: {path_text}",
                    "source": path_text,
                    "pr": pr_of(added.get(path_text, "")),
                    "visible": True,
                }
            )
            break
        else:
            if path_text.startswith("docs/adr/") and path_text.endswith(".md"):
                facts.append(
                    {
                        "kind": "adr",
                        "claim": f"Новый ADR: {Path(path_text).name}",
                        "source": path_text,
                        "visible": True,
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
    # Прогоны робота с песочницы (issue #4702): за неделю робот ездил по
    # складу, а в фактах прогонов не было — тема выбиралась из одних PR.
    # Без ключа или без песочницы сборщик молча отдаёт пустой список.
    facts += collect_robot_runs()
    # Факты о файлах и медиа читаются из worktree на origin/develop: рабочая ветка
    # студии свежих материалов не содержит (требование координатора, 06.11).
    source_root = develop.ensure_develop_worktree(REPO_ROOT, out=lambda _m: None) or REPO_ROOT
    facts += collect_new_files_by_git(source_root, since)
    if source_root == REPO_ROOT:
        # Без worktree develop приходится смотреть на mtime — но там у всех файлов
        # время выставлено заново, и «новыми» вышли бы все разом. Поэтому mtime-сбор
        # работает только когда worktree недоступен.
        facts += collect_new_files(source_root, since)

    state = st.load_state(args.state)
    # Ключ каждому факту: по нему видно, что событие уже показано.
    for fact in facts:
        fact["key"] = fact_key(fact)

    # Что уже закрыто — не предлагаем снова.
    closed = st.covered_keys(state)
    # Что уже рассказано в опубликованных постах — тоже не предлагаем.
    closed |= covered_from_posts(source_root / "docs" / "channel" / "queue", facts)
    # Что предлагали, но не показали — возвращаем: событие не должно потеряться.
    #
    # Прогоны робота собираются КАЖДЫЙ раз (список песочницы не меняется), поэтому
    # один и тот же факт приходит и свежим, и из ожидания. Дубликат в ленте — это
    # две одинаковые темы, а счётчик показов теряется: показанную тему студия
    # предложила бы снова. Поэтому факт из ожидания не добавляется вторым, а его
    # счётчик переносится на свежий.
    fresh_by_key = {fact.get("key"): fact for fact in facts}
    for fact in st.pending_facts(state):
        key = fact.get("key")
        if not key or key in closed:
            continue
        current = fresh_by_key.get(key)
        if current is None:
            facts.append(fact)
            fresh_by_key[key] = fact
            continue
        current["attempts"] = max(int(current.get("attempts") or 0), int(fact.get("attempts") or 0))

    # Факты, потерянные вместе с протухшим черновиком: в записи поста остался
    # только ключ (`pr:4555`), а без заголовка и тела PR пост не собрать.
    # Восстанавливаем такие факты из GitHub — событие возвращается в ленту.
    lost = lost_fact_keys(state)
    if lost:
        for key in sorted(lost):
            fact = refetch_fact_by_key(args.repo, key)
            if fact:
                facts.append(fact)
        print(f"восстановлено потерянных фактов: {len(lost)}", file=sys.stderr)

    fresh, deferred = partition_facts(facts, closed)

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
