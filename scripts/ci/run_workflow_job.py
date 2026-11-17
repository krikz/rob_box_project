#!/usr/bin/env python3
"""Прогнать job'ы GitHub-workflow ЛОКАЛЬНО — те же `run:`-шаги, из того же YAML.

Зачем: проверки (G-Lint Code, G-Architecture Audit, G-Run Tests …) живут
в .github/workflows/*.yml, а локально их гоняли по памяти/по README —
команды расходились с CI (пример: G-Lint Code перезаписывал .yamllint.yml
своим heredoc'ом, и pre-commit линтил с другим конфигом). Этот раннер не
хранит ни одной команды: он читает workflow и исполняет его `run:`-шаги в
текущем чекауте. Поменяли шаг в CI — локальный прогон поменялся сам.

Что делает / не делает (честно):
  * исполняет `run:` через `bash --noprofile --norc -eo pipefail` (как
    GitHub), с `env:` workflow → job → step и `working-directory`;
  * `uses:`-шаги (checkout, setup-python, upload-artifact …) пропускает:
    чекаут — это ваш рабочий каталог, python — ваш;
  * шаги с `${{ ... }}` в теле пропускает (контекстов GitHub локально нет) —
    они печатаются как SKIP, а не выдаются за пройденные;
  * шаги `Install …` по умолчанию пропускает (не ставим пакеты в вашу
    систему без спроса) — `--with-install`, чтобы выполнить;
  * `continue-on-error: true` → WARN (как в CI: job не валится);
  * по умолчанию только job'ы на `ubuntu-*`: self-hosted job'ы (сборка,
    деплой, робот) зовут отдельными инструментами — scripts/build/build.py.

Примеры:
    scripts/ci/run_workflow_job.py --list
    scripts/ci/run_workflow_job.py "G-Lint Code"                 # все job'ы
    scripts/ci/run_workflow_job.py "G-Lint Code" -j python-lint
    scripts/ci/run_workflow_job.py "G-Lint Code" -j python-lint --step "CC-budget"
    scripts/ci/run_workflow_job.py "G-Architecture Audit" --dry-run
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
WORKFLOWS = REPO_ROOT / ".github" / "workflows"
EXPR = re.compile(r"\$\{\{.*?\}\}", re.S)
INSTALL = re.compile(r"^\s*install\b", re.I)


def resolve_workflow(name: str) -> Path:
    p = Path(name)
    if p.is_file():
        return p.resolve()
    for cand in (
        WORKFLOWS / name,
        WORKFLOWS / f"{name}.yml",
        WORKFLOWS / f"{name}.yaml",
    ):
        if cand.is_file():
            return cand
    matches = [f for f in WORKFLOWS.glob("*.y*ml") if name.lower() in f.stem.lower()]
    if len(matches) == 1:
        return matches[0]
    hint = ", ".join(sorted(m.stem for m in matches)) or "ничего"
    raise SystemExit(f"workflow {name!r} не найден однозначно (совпало: {hint})")


def load(path: Path) -> dict:
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def runs_on_hosted(job: dict) -> bool:
    ro = job.get("runs-on", "")
    labels = ro if isinstance(ro, list) else [ro]
    return any(str(label).startswith("ubuntu-") for label in labels)


def plain_env(mapping: dict | None) -> tuple[dict[str, str], list[str]]:
    env, skipped = {}, []
    for k, v in (mapping or {}).items():
        v = "" if v is None else str(v).lower() if isinstance(v, bool) else str(v)
        if EXPR.search(v):
            skipped.append(k)
        else:
            env[k] = v
    return env, skipped


def shell_argv(shell: str | None, script: Path) -> list[str]:
    if shell in (None, "bash"):
        return ["bash", "--noprofile", "--norc", "-eo", "pipefail", str(script)]
    if shell == "sh":
        return ["sh", "-e", str(script)]
    if shell == "python":
        return [sys.executable, str(script)]
    return [*shell.replace("{0}", str(script)).split()]


def list_workflows() -> None:
    for f in sorted(WORKFLOWS.glob("*.y*ml")):
        try:
            wf = load(f)
        except yaml.YAMLError:
            continue
        jobs = (wf or {}).get("jobs") or {}
        hosted = [j for j, d in jobs.items() if runs_on_hosted(d)]
        other = [j for j in jobs if j not in hosted]
        print(f"{f.stem}")
        if hosted:
            print(f"    локально: {' '.join(hosted)}")
        if other:
            print(f"    self-hosted/прочие (не гоняются): {' '.join(other)}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("\n\n", 1)[1],
    )
    ap.add_argument(
        "workflow", nargs="?", help="имя файла/часть имени в .github/workflows"
    )
    ap.add_argument(
        "-j", "--job", action="append", default=[], help="job id (можно несколько)"
    )
    ap.add_argument("--step", help="regex по имени шага — выполнить только совпавшие")
    ap.add_argument("--skip", help="regex по имени шага — пропустить совпавшие")
    ap.add_argument(
        "--with-install", action="store_true", help="выполнять шаги Install …"
    )
    ap.add_argument(
        "--all-runners", action="store_true", help="не фильтровать по runs-on: ubuntu-*"
    )
    ap.add_argument("--list", action="store_true", help="перечислить workflow и job'ы")
    ap.add_argument("--dry-run", action="store_true", help="только показать план")
    ap.add_argument(
        "--fail-fast", action="store_true", help="остановиться на первом FAIL"
    )
    args = ap.parse_args(argv)

    if args.list or not args.workflow:
        list_workflows()
        return 0

    path = resolve_workflow(args.workflow)
    wf = load(path)
    jobs: dict = wf.get("jobs") or {}
    unknown = set(args.job) - set(jobs)
    if unknown:
        raise SystemExit(f"нет таких job'ов в {path.name}: {' '.join(sorted(unknown))}")
    selected = args.job or [
        j for j, d in jobs.items() if args.all_runners or runs_on_hosted(d)
    ]
    wf_env, _ = plain_env(wf.get("env"))

    results: list[tuple[str, str, str, float]] = []
    tmp = Path(tempfile.mkdtemp(prefix="wfjob-"))
    shown = path.relative_to(REPO_ROOT) if path.is_relative_to(REPO_ROOT) else path
    print(f"workflow: {shown}   job'ы: {' '.join(selected)}")

    for job_id in selected:
        job = jobs[job_id]
        job_env, _ = plain_env(job.get("env"))
        defaults = ((job.get("defaults") or {}).get("run") or {}) or (
            (wf.get("defaults") or {}).get("run") or {}
        )
        print(f"\n════ job {job_id} ({job.get('name', job_id)}) ════")
        for idx, step in enumerate(job.get("steps") or [], 1):
            name = step.get("name") or step.get("uses") or f"step {idx}"
            label = f"{job_id} › {name}"

            def record(status: str, note: str = "", secs: float = 0.0) -> None:
                results.append((status, label, note, secs))
                print(f"  [{status}] {name}" + (f" — {note}" if note else ""))

            if args.step and not re.search(args.step, name):
                continue
            if args.skip and re.search(args.skip, name):
                record("SKIP", "--skip")
                continue
            if "uses" in step:
                record("SKIP", f"uses: {step['uses']}")
                continue
            body = step.get("run")
            if body is None:
                continue
            if INSTALL.search(name) and not args.with_install:
                record("SKIP", "установка пакетов (--with-install, чтобы выполнить)")
                continue
            if EXPR.search(body):
                record("SKIP", "в теле ${{ }} — контекстов GitHub локально нет")
                continue
            if step.get("if") and EXPR.sub("", str(step["if"])).strip() not in (
                "",
                "always()",
                "success()",
            ):
                record("SKIP", f"if: {step['if']}")
                continue
            step_env, skipped_env = plain_env(step.get("env"))
            if skipped_env:
                record("SKIP", f"env из ${{{{ }}}}: {' '.join(skipped_env)}")
                continue
            if args.dry_run:
                record("PLAN")
                continue

            script = tmp / f"{job_id}-{idx}.sh"
            script.write_text(body, encoding="utf-8")
            for f in (
                "GITHUB_OUTPUT",
                "GITHUB_ENV",
                "GITHUB_STEP_SUMMARY",
                "GITHUB_PATH",
            ):
                (tmp / f).touch()
            env = {
                **os.environ,
                **wf_env,
                **job_env,
                **step_env,
                "CI": "true",
                "GITHUB_WORKSPACE": str(REPO_ROOT),
                **{
                    f: str(tmp / f)
                    for f in (
                        "GITHUB_OUTPUT",
                        "GITHUB_ENV",
                        "GITHUB_STEP_SUMMARY",
                        "GITHUB_PATH",
                    )
                },
            }
            cwd = REPO_ROOT / (
                step.get("working-directory")
                or defaults.get("working-directory")
                or "."
            )
            print(f"  ▶ {name}", flush=True)
            t0 = time.monotonic()
            rc = subprocess.run(
                shell_argv(step.get("shell") or defaults.get("shell"), script),
                cwd=cwd,
                env=env,
            ).returncode
            secs = time.monotonic() - t0
            if rc == 0:
                record("PASS", "", secs)
            elif step.get("continue-on-error"):
                record("WARN", f"exit {rc}, continue-on-error", secs)
            else:
                record("FAIL", f"exit {rc}", secs)
                if args.fail_fast:
                    break
        else:
            continue
        break

    print("\n════ итог ════")
    for status, label, note, secs in results:
        t = f" {secs:.1f}s" if secs else ""
        print(
            f"  {status:4} {label}{t}"
            + (f"  ({note})" if note and status != "PASS" else "")
        )
    counts = {
        s: sum(1 for r in results if r[0] == s)
        for s in ("PASS", "FAIL", "WARN", "SKIP", "PLAN")
    }
    print("  " + "  ".join(f"{k}={v}" for k, v in counts.items() if v))
    return 1 if counts["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main())
