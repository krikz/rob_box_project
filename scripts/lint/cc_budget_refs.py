#!/usr/bin/env python3
"""CC-budget baseline ref-tracker guard (ADR-0021 R1, issue #2186).

ADR-0021 R1 says: «при превышении воркер **обязан** открыть карточку
рефакторинга». ``scripts/lint/cc_budget.py`` грандфазит существующие
нарушения через ``cc_budget_baseline.json`` (ADR-0021 stage 4) — но
без отдельной проверки baseline пополняется молча, и рост CC
фиксируется сторожем вместо того, чтобы сдерживаться (issue #2186:
``WSSServer._on_json_cmd`` дожил до CC=107 до того, как его заметили).

Этот гард ловит «молчаливое пополнение» в двух слоях:

1. **Локальный (без сети)** — выполняется в CI. Каждая запись в
   ``exemptions`` должна иметь соответствующую запись в ``_refactor_cards``
   формата ``"<path>:<method>": "#NNNN (alias)"``. Без неё — FAIL.
   Это структурная проверка: «каждая grandfather-запись имеет явный
   контракт на удаление».

2. **Удалённый (опционально, ``--verify-remote``)** — для локального
   использования разработчиком / e2e-round. Проверяет, что указанные
   issue существуют и помечены ``type:tech-debt`` (через ``gh api
   /repos/{owner}/{repo}/issues/{n}`` с лейблами в ответе). Требует
   ``GH_REPO`` в env и ``gh`` CLI с авторизацией.

Сценарии, которые ловятся:

* baseline-файл изменён в PR, добавлен новый exemption **без**
  ``_refactor_cards`` записи → FAIL локально.
* ``_refactor_cards`` ссылается на issue без метки ``type:tech-debt``
  (закрыли, переоткрыли как bug, etc.) → FAIL при ``--verify-remote``.

Сценарии, которые **не** ловятся этим гардом (явно out of scope):

* Удаление exemption без рефакторинга — это уже ловит сам ``cc_budget.py``
  через «method not in baseline» FAIL.
* Рост CC поверх baseline-значения — тоже ``cc_budget.py`` (см. его
  ``violations.append(... 'grew past baseline'``).

Usage:
  python scripts/lint/cc_budget_refs.py                 # local check (CI default)
  python scripts/lint/cc_budget_refs.py --verify-remote # also hit GitHub API
  python scripts/lint/cc_budget_refs.py --baseline <path>  # override path
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BASELINE_FILE = REPO_ROOT / "scripts" / "lint" / "cc_budget_baseline.json"

# Issue refs в `_refactor_cards` пишутся как ``#NNNN (alias)``. Допускаем
# формат ``#NNNN``, ``#NNNN (anything)``, но требуем ровно один ``#NNNN``.
_ISSUE_REF_RE = re.compile(r"^#(\d+)(?:\s*\(([^)]*)\))?\s*$")

# GitHub label, обязательная для ref-карточек (ADR-0021 R1: «обязан открыть
# карточку рефакторинга»). Эта же метка используется issue triage для
# группировки tech-debt цикла.
REQUIRED_LABEL = "type:tech-debt"


def _load_baseline(path: Path) -> dict:
    if not path.exists():
        print(f"cc_budget_refs: missing baseline {path}", file=sys.stderr)
        sys.exit(2)
    return json.loads(path.read_text(encoding="utf-8"))


def _exempt_keys(baseline: dict) -> set[str]:
    """Множество ``"<path>:<method>"`` для каждого грандфазанного метода."""
    out: set[str] = set()
    for rel_path, methods in baseline.get("exemptions", {}).items():
        for method, cc in methods.items():
            if isinstance(cc, int):  # на случай мусора в JSON
                out.add(f"{rel_path}:{method}")
    return out


def _ref_cards(baseline: dict) -> dict[str, str]:
    """Словарь ``"<path>:<method>" -> "#NNNN (alias)"`` из baseline."""
    return dict(baseline.get("_refactor_cards", {}))


def _legacy_keys(baseline: dict) -> set[str]:
    """Множество ``"<path>:<method>"`` методов, grandfather'нутых ДО введения
    обязательной ref-card привязки (issue #2186, commit этого PR).

    Это существующий долг, накопленный к моменту активации гарда. Здесь
    перечислены ВСЕ ранее существовавшие entries; они не требуют
    ``_refactor_cards`` ссылки (т.к. карточек ещё нет, иначе бы они
    были в ``_refactor_cards``). Это даёт чистый запуск гарда без
    блокировки существующего кода.
    """
    out: set[str] = set()
    for entry in baseline.get("_legacy_acknowledged", []):
        path = entry.get("path", "")
        method = entry.get("method", "")
        if path and method:
            out.add(f"{path}:{method}")
    return out


def check_local(baseline: dict) -> list[str]:
    """Чисто структурная проверка. Возвращает список нарушений (текст)."""
    violations: list[str] = []
    exempt = _exempt_keys(baseline)
    refs = _ref_cards(baseline)
    legacy = _legacy_keys(baseline)
    legacy_paths = {entry.get("path") for entry in baseline.get("_legacy_acknowledged", [])}

    # Sanity-check: каждый entry в _legacy_acknowledged обязан реально
    # присутствовать в exemptions (это «фотография» существующего
    # долга; если кто-то удалил запись из exemptions, не почистив
    # legacy — это либо баг, либо рефакторинг завершился, либо враньё).
    for entry in baseline.get("_legacy_acknowledged", []):
        path = entry.get("path", "")
        method = entry.get("method", "")
        legacy_cc = entry.get("cc")
        if path not in baseline.get("exemptions", {}) or method not in baseline.get("exemptions", {}).get(path, {}):
            violations.append(
                f"[FAIL] _legacy_acknowledged: {path}:{method} упоминается "
                f"как legacy, но отсутствует в exemptions (висячая запись)"
            )
            continue
        actual_cc = baseline["exemptions"][path][method]
        if legacy_cc is not None and legacy_cc != actual_cc:
            violations.append(
                f"[FAIL] _legacy_acknowledged: {path}:{method} cc={legacy_cc} "
                f"не совпадает с exemptions cc={actual_cc}; правьте оба места"
            )

    # Каждый non-legacy exempt обязан иметь ref.
    for key in sorted(exempt):
        if key in legacy:
            continue
        if key not in refs:
            violations.append(
                f"[FAIL] {key} — нет записи в _refactor_cards; "
                f"добавьте \"{key}\": \"#NNNN (alias)\" и убедитесь, "
                f"что у issue есть метка `{REQUIRED_LABEL}`"
            )

    # Каждый ref должен ссылаться на существующий non-legacy exempt
    # (не на фантом). Refs на legacy-exempt — допустимы (полезно для
    # карточек, заведенных под уже существующий долг), но не обязательны.
    for key in sorted(refs.keys()):
        if key not in exempt:
            violations.append(
                f"[FAIL] {key} — есть в _refactor_cards, но отсутствует "
                f"в exemptions (висячая ссылка)"
            )
        # Внутри значения тоже проверим формат.
        match = _ISSUE_REF_RE.match(refs[key].strip())
        if not match:
            violations.append(
                f"[FAIL] {key} — значение {refs[key]!r} не похоже на "
                f"'#NNNN' или '#NNNN (alias)'"
            )

    # _legacy_acknowledged не должен расти бесконтрольно. Если там
    # появляются методы, которых ещё нет в exemptions — это ошибка
    # редактирования. Если там появляются методы, которые УЖЕ имеют
    # ref-card — это значит, рефакторинг завершился, ref-card
    # присутствует, и эту legacy-запись можно перенести в чистый
    # exempt+ref без legacy-пометки.
    legacy_with_refs = legacy & set(refs.keys())
    if legacy_with_refs:
        # Не fail-loud: это soft-warning, чтобы было видно при прогоне,
        # но не блокировать merge (если рефакторинг уже сделал запись
        # ref-пригодной, чистка — отдельная задача).
        for key in sorted(legacy_with_refs):
            print(
                f"  [info] {key} — есть в _legacy_acknowledged, но уже "
                f"имеет ref-card; на следующем refactor можно убрать "
                f"из legacy (cleanup task)"
            )

    # Если path есть в legacy и в exemptions для других методов
    # (например, dialogue_node.py имеет 8 grandfather-записей), это
    # нормально — каждый метод legacy-списка независим.
    _ = legacy_paths  # пусть линтер не ругается на неиспользование
    return violations


def _gh_api(repo: str, issue_num: int) -> dict | None:
    """Возвращает JSON issue через ``gh api`` или None при ошибке."""
    try:
        proc = subprocess.run(
            ["gh", "api", f"/repos/{repo}/issues/{issue_num}"],
            capture_output=True,
            text=True,
            check=False,
            timeout=20,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0 or not proc.stdout.strip():
        return None
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError:
        return None


def check_remote(baseline: dict, repo: str) -> list[str]:
    """Сетевая проверка: каждый ref существует и помечен ``type:tech-debt``."""
    violations: list[str] = []
    refs = _ref_cards(baseline)
    seen_issues: dict[int, str] = {}
    for key, ref in sorted(refs.items()):
        match = _ISSUE_REF_RE.match(ref.strip())
        if not match:
            continue  # уже поймано в local check
        issue_num = int(match.group(1))
        # Кэшируем запрос на каждый уникальный issue (несколько методов
        # могут ссылаться на одну карточку).
        if issue_num in seen_issues:
            continue
        data = _gh_api(repo, issue_num)
        if data is None:
            violations.append(
                f"[FAIL] {key} -> {ref} — не удалось получить "
                f"https://github.com/{repo}/issues/{issue_num} "
                f"через gh api (нет сети / нет auth / issue не существует)"
            )
            seen_issues[issue_num] = "ERROR"
            continue
        labels = [lbl.get("name", "") for lbl in data.get("labels", [])]
        if REQUIRED_LABEL not in labels:
            violations.append(
                f"[FAIL] {key} -> #{issue_num} существует, но без метки "
                f"`{REQUIRED_LABEL}` (фактические метки: {labels!r}); "
                f"ADR-0021 R1 требует, чтобы карточка рефакторинга была "
                f"tech-debt"
            )
        seen_issues[issue_num] = ",".join(labels) or "(no-labels)"

    return violations


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline",
        type=Path,
        default=BASELINE_FILE,
        help=f"Путь к cc_budget_baseline.json (default: {BASELINE_FILE})",
    )
    parser.add_argument(
        "--verify-remote",
        action="store_true",
        help="Также проверить, что issue в _refactor_cards существуют "
        "и помечены type:tech-debt (требует GH_REPO и gh CLI)",
    )
    args = parser.parse_args(argv)

    try:
        baseline_display = args.baseline.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        baseline_display = str(args.baseline)
    baseline = _load_baseline(args.baseline)
    exempt = _exempt_keys(baseline)
    refs = _ref_cards(baseline)
    print(
        f"cc_budget_refs: проверяю {baseline_display} "
        f"({len(exempt)} exemptions, {len(refs)} ref-cards)"
    )

    violations = check_local(baseline)
    for v in violations:
        print(v)

    if args.verify_remote:
        repo = os.environ.get("GH_REPO", "")
        if not repo:
            print(
                "[FAIL] --verify-remote требует GH_REPO в env "
                "(формат 'owner/repo')",
                file=sys.stderr,
            )
            return 2
        remote_violations = check_remote(baseline, repo)
        for v in remote_violations:
            print(v)
        violations.extend(remote_violations)

    if violations:
        print(
            f"cc_budget_refs: FAIL — {len(violations)} violation(s); "
            f"каждый exemption обязан иметь ref на type:tech-debt issue.",
            file=sys.stderr,
        )
        return 1
    print("cc_budget_refs: OK — все exemptions имеют ref-cards.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
