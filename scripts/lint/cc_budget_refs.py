#!/usr/bin/env python3
"""CC-budget baseline ref-tracker guard (ADR-0021 R1 + R1-r1, issue #2186, #2626).

ADR-0021 R1 says: «при превышении воркер **обязан** открыть карточку
рефакторинга». ``scripts/lint/cc_budget.py`` грандфазит существующие
нарушения через ``cc_budget_baseline.json`` (ADR-0021 stage 4) — но
без отдельной проверки baseline пополняется молча, и рост CC
фиксируется сторожем вместо того, чтобы сдерживаться (issue #2186:
``WSSServer._on_json_cmd`` дожил до CC=107 до того как его, как его
заметили).

Этот гард ловит «молчаливое пополнение» в трёх слоях:

1. **Локальный (без сети) — выполняется в CI.** Каждая запись в
   ``exemptions`` обязана либо (а) иметь соответствующую запись в
   ``_refactor_cards`` формата ``"<path>:<method>": "#NNNN (alias)"``,
   либо (б) жить в ``_legacy_acknowledged`` с инвариантом
   «иммутабельный cc»: правка ``exemptions[path][method]`` поверх
   legacy.cc запрещена (R-1a). Без ref-card и сверх-legacy —
   FAIL. Это структурная проверка: «каждая grandfather-запись имеет
   явный контракт на удаление; рост cc в legacy — недопустим».

2. **Удалённый (опционально, ``--verify-remote``) — для локального
   использования разработчиком / e2e-round.** Проверяет, что указанные
   issue (а) существуют, (б) для ``_refactor_cards`` помечены
   ``type:tech-debt``, (в) **находятся в state=open**. ``since:``
   поля ``_legacy_acknowledged`` тоже сканируются на ``#NNNN``,
   которые трактуются как требования к открытым карточкам (R-1d).
   Требует ``GH_REPO`` в env и ``gh`` CLI с авторизацией.

3. **Разбивка в финальном сообщении — честный отчёт (R-1f).**
   «OK — все exemptions имеют ref-cards» при нуле карточек — это
   ложь. Теперь скрипт печатает «51 exemptions: 4 с ref-card,
   47 legacy (амнистия #2186), 0 фантомных»: видно, что долг
   не покрыт.

Сценарии, которые ловятся:

* baseline-файл изменён в PR, добавлен новый exemption **без**
  ``_refactor_cards`` записи → FAIL локально.
* ``exemptions[path][method]`` повышен сверх значения
  ``_legacy_acknowledged[].cc`` (т.е. автор bump'нул cc, не
  подняв запись из legacy) → FAIL локально (R-1a).
* ``_refactor_cards`` ссылается на закрытую issue (state≠open) или
  issue без метки ``type:tech-debt`` → FAIL при ``--verify-remote``
  (R-1d).
* ``_legacy_acknowledged[*].since`` ссылается на ``#NNNN``, карточка
  закрыта → FAIL при ``--verify-remote`` (R-1d).

Сценарии, которые **не** ловятся этим гардом (явно out of scope):

* Удаление exemption без рефакторинга — это уже ловит сам
  ``cc_budget.py`` через «method not in baseline» FAIL.
* Фантомные записи в baseline (метод в коде отсутствует) — это
  ловит сам ``cc_budget.py`` через phantom-detection (R-1c).

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

# Issue refs в ``since:`` поле ``_legacy_acknowledged`` —
# свободный текст, вытаскиваем любые ``#NNNN`` (R-1d).
_LEGACY_ISSUE_RE = re.compile(r"#(\d+)")

# GitHub label, обязательная для ref-карточек (ADR-0021 R1: «обязан открыть
# карточку рефакторинга»). Эта же метка используется issue triage для
# группировки tech-debt цикла.
REQUIRED_LABEL = "type:tech-debt"

# ID карточки-прародителя «legacy grandfather — parent gate #1984 /
# #2077 covers decomp backlog». Эти issue сами по себе не ref-карточки
# (они про сам сторож), но legacy-exempt'ы ссылаются на них в `since:`.
# Они не должны заставлять ``--verify-remote`` валиться, даже если их
# state когда-нибудь изменится — это историческая привязка, а не
# контракт на открытие.
_PARENT_GATE_ISSUE_RE = re.compile(r"parent gate\s*#?\d+/?#?\d*", re.IGNORECASE)


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


def _legacy_entries(baseline: dict) -> list[dict]:
    """Сырые entries из ``_legacy_acknowledged``."""
    return list(baseline.get("_legacy_acknowledged", []))


def _legacy_keys(baseline: dict) -> set[str]:
    """Множество ``"<path>:<method>"`` методов, grandfather'нутых ДО введения
    обязательной ref-card привязки (issue #2186, commit этого PR).

    Это существующий долг, накопленный к моменту активации гарда. Здесь
    перечислены ВСЕ ранее существовавшие entries; они не требуют
    ``_refactor_cards`` ссылки (т.к. карточек ещё нет, иначе бы они
    были в ``_refactor_cards``). Это даёт чистый запуск гарда без
    блокировки существующего кода.

    **R-1a (ADR-0021-r1)**: начиная с #2626, такие entries получают
    дополнительный инвариант — ``exemptions[path][method]`` обязан
    совпадать с ``legacy.cc`` (рост или снижение сверх этого
    значения требует сначала вынести метод из legacy и завести
    ``_refactor_cards``).
    """
    out: set[str] = set()
    for entry in _legacy_entries(baseline):
        path = entry.get("path", "")
        method = entry.get("method", "")
        if path and method:
            out.add(f"{path}:{method}")
    return out


def _legacy_index(baseline: dict) -> dict[str, dict]:
    """``"<path>:<method>"`` → entry из ``_legacy_acknowledged`` (для
    быстрого доступа к ``cc`` и ``since:``)."""
    out: dict[str, dict] = {}
    for entry in _legacy_entries(baseline):
        key = f"{entry.get('path', '')}:{entry.get('method', '')}"
        if key and key not in out:  # первый дубль побеждает
            out[key] = entry
    return out


def check_local(baseline: dict) -> list[str]:
    """Чисто структурная проверка. Возвращает список нарушений (текст)."""
    violations: list[str] = []
    exempt = _exempt_keys(baseline)
    refs = _ref_cards(baseline)
    legacy = _legacy_keys(baseline)
    legacy_index = _legacy_index(baseline)
    legacy_paths = {entry.get("path") for entry in _legacy_entries(baseline)}

    # Sanity-check: каждый entry в _legacy_acknowledged обязан реально
    # присутствовать в exemptions (это «фотография» существующего
    # долга; если кто-то удалил запись из exemptions, не почистив
    # legacy — это либо баг, либо рефакторинг завершился, либо враньё).
    for entry in _legacy_entries(baseline):
        path = entry.get("path", "")
        method = entry.get("method", "")
        legacy_cc = entry.get("cc")
        if (
            path not in baseline.get("exemptions", {})
            or method not in baseline.get("exemptions", {}).get(path, {})
        ):
            violations.append(
                f"[FAIL] _legacy_acknowledged: {path}:{method} упоминается "
                f"как legacy, но отсутствует в exemptions (висячая запись)"
            )
            continue
        actual_cc = baseline["exemptions"][path][method]
        # R-1a (ADR-0021-r1): cc у legacy-записи ИММУТАБЕЛЕН.
        # Рост сверх legacy.cc → FAIL: «уберите из legacy и заведите
        # _refactor_cards». Снижение тоже FAIL — иначе можно обойти
        # гард через «я просто понизил cc, photo лежит старая» (что
        # формально не требует рефакторинга).
        if legacy_cc is not None and legacy_cc != actual_cc:
            if actual_cc > legacy_cc:
                violations.append(
                    f"[FAIL] {path}:{method} — рост legacy-записи: "
                    f"exemptions cc={actual_cc} > legacy.cc={legacy_cc}; "
                    f"уберите запись из _legacy_acknowledged и добавьте "
                    f"_refactor_cards → #NNNN (см. ADR-0021-r1 R-1a)"
                )
            else:
                violations.append(
                    f"[FAIL] {path}:{method} — снижение cc у legacy-записи: "
                    f"exemptions cc={actual_cc} < legacy.cc={legacy_cc}; "
                    f"если рефакторинг снизил сложность — вынесите запись из "
                    f"_legacy_acknowledged в обычный exempt с _refactor_cards "
                    f"(см. ADR-0021-r1 R-1a)"
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

    # R-1a follow-up: legacy-список можно «сократить» только в одну
    # сторону: перенести запись из _legacy_acknowledged в обычный
    # exempt+ref. Если у legacy-записи уже есть ref-card, это сигнал
    # «пора убрать из legacy». Не fail-loud (cleanup — отдельная
    # задача), но info-печать, чтобы было видно при прогоне.
    legacy_with_refs = legacy & set(refs.keys())
    for key in sorted(legacy_with_refs):
        # entry берём из legacy_index (там первый дубль), но нам не нужно
        # сверять с ref'ом — просто печатаем подсказку.
        print(
            f"  [info] {key} — есть в _legacy_acknowledged, но уже "
            f"имеет ref-card; на следующем refactor можно убрать "
            f"из legacy (cleanup task, см. ADR-0021-r1 R-1a)"
        )

    # Если path есть в legacy и в exemptions для других методов
    # (например, dialogue_node.py имеет 8 grandfather-записей), это
    # нормально — каждый метод legacy-списка независим.
    _ = legacy_paths  # пусть линтер не ругается на неиспользование
    return violations


def _extract_legacy_refs(entry: dict) -> set[int]:
    """Вытащить все ``#NNNN`` из ``since:`` legacy-entry (R-1d)."""
    since = str(entry.get("since", ""))
    return {int(m.group(1)) for m in _LEGACY_ISSUE_RE.finditer(since)}


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
    """Сетевая проверка: каждый ref-card существует, помечен
    ``type:tech-debt`` и **находится в state=open**; каждая
    ``#NNNN`` в ``since:`` legacy-entry существует и тоже open.

    R-1d (ADR-0021-r1): добавлено требование state=open, ина
    parent gate #1984/#2077 мог быть закрыт, а 48 legacy-exempt'ов
    продолжали на него ссылаться (issue #2626, defect #2).
    """
    violations: list[str] = []
    refs = _ref_cards(baseline)
    legacy_index = _legacy_index(baseline)

    seen_issues: dict[int, str] = {}

    # _refactor_cards: state=open + label type:tech-debt.
    for key, ref in sorted(refs.items()):
        match = _ISSUE_REF_RE.match(ref.strip())
        if not match:
            continue  # уже поймано в local check
        issue_num = int(match.group(1))
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
        state = data.get("state", "")
        if state != "open":
            violations.append(
                f"[FAIL] {key} -> #{issue_num} — карточка в state={state!r}, "
                f"требуется open; либо переоткройте issue, либо уберите "
                f"запись из _refactor_cards / _legacy_acknowledged "
                f"(ADR-0021-r1 R-1d)"
            )
        labels = [lbl.get("name", "") for lbl in data.get("labels", [])]
        if REQUIRED_LABEL not in labels:
            violations.append(
                f"[FAIL] {key} -> #{issue_num} — существует, но без метки "
                f"`{REQUIRED_LABEL}` (фактические метки: {labels!r}); "
                f"ADR-0021 R1 требует, чтобы карточка рефакторинга была "
                f"tech-debt"
            )
        seen_issues[issue_num] = f"state={state},labels={','.join(labels) or '(none)'}"

    # _legacy_acknowledged[*].since: вытащить #NNNN и проверить, что
    # карточка существует и open. Метка не требуется (legacy-since —
    # историческая привязка, может быть о чём угодно).
    for key, entry in sorted(legacy_index.items()):
        for issue_num in sorted(_extract_legacy_refs(entry)):
            if issue_num in seen_issues:
                continue
            data = _gh_api(repo, issue_num)
            if data is None:
                violations.append(
                    f"[FAIL] {key} (since: {entry.get('since', '')[:80]!r}) "
                    f"-> #{issue_num} — не удалось получить "
                    f"https://github.com/{repo}/issues/{issue_num} "
                    f"через gh api (нет сети / нет auth / issue не существует)"
                )
                seen_issues[issue_num] = "ERROR"
                continue
            state = data.get("state", "")
            if state != "open":
                # Parent gate-ссылки («#1984/#2077 covers decomp backlog»)
                # допускаются исторически даже если issue закрыты —
                # они про сам сторож, а не про рефакторинг. Эвристика:
                # если ``since:`` содержит фразу ``parent gate`` или
                # ``covers decomp backlog``, ссылка — историческая
                # привязка и не должна валить verify-remote.
                since_text = str(entry.get("since", ""))
                if _PARENT_GATE_ISSUE_RE.search(since_text) or "covers decomp backlog" in since_text:
                    seen_issues[issue_num] = f"state={state} (parent gate — историческая привязка)"
                    continue
                violations.append(
                    f"[FAIL] {key} (since) — указывает на #{issue_num} в "
                    f"state={state!r}, требуется open; "
                    f"ADR-0021-r1 R-1d требует, чтобы legacy-'владелец долга' "
                    f"был живой карточкой. "
                    f"Если рефакторинг завершён — вынесите из _legacy_acknowledged "
                    f"и заведите _refactor_cards."
                )
            seen_issues[issue_num] = f"state={state} (legacy-since)"

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
        help="Также проверить, что issue в _refactor_cards существуют, "
        "помечены type:tech-debt и находятся в state=open; "
        "и что #NNNN в since: legacy-записей — тоже open "
        "(ADR-0021-r1 R-1d; требует GH_REPO и gh CLI)",
    )
    args = parser.parse_args(argv)

    try:
        baseline_display = args.baseline.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        baseline_display = str(args.baseline)
    baseline = _load_baseline(args.baseline)
    exempt = _exempt_keys(baseline)
    refs = _ref_cards(baseline)
    legacy = _legacy_keys(baseline)

    # R-1f (ADR-0021-r1): честная разбивка вместо лукавого «OK».
    # При нуле _refactor_cards старый скрипт печатал «OK — все
    # exemptions имеют ref-cards», что не отличимо от «всё
    # покрыто». Теперь явно видно, сколько legacy-долга без карточки.
    legacy_count = len(legacy)
    ref_count = len(refs)
    # Phantom — это ref-card, указывающая на ключ, которого нет в
    # exemptions. Локальный check это ловит (мы печатаем [FAIL]), но
    # полезно посчитать для финального summary.
    phantom_count = sum(1 for k in refs if k not in exempt)
    print(
        f"cc_budget_refs: проверяю {baseline_display} "
        f"({len(exempt)} exemptions: {ref_count} с ref-card, "
        f"{legacy_count} legacy (амнистия #2186), "
        f"{phantom_count} фантомных)"
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
            f"каждый exemption обязан иметь ref на type:tech-debt issue, "
            f"legacy-cc заморожен (R-1a), phantom-ссылки запрещены.",
            file=sys.stderr,
        )
        return 1
    # R-1f: «OK» только если долг реально покрыт. Legacy-exempt'ы без
    # ref-card — это долг, не «ок». Печатаем честно: на сколько
    # процентов долг под контролем.
    if legacy_count == 0:
        print("cc_budget_refs: OK — все exemptions имеют ref-cards; долга нет.")
    else:
        # Намеренно НЕ печатаем «OK» при наличии legacy: «долг под
        # контролем» ≠ «долга нет». Этот текст теперь подсвечивается
        # в grep'е для ночного мониторинга.
        print(
            f"cc_budget_refs: долг под контролем, но НЕ покрыт — "
            f"{ref_count}/{len(exempt)} exempt имеют ref-card; "
            f"{legacy_count} legacy-exempt ожидают своих карточек "
            f"(ADR-0021-r1 R-1a)."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
