#!/usr/bin/env python3
"""Корпус замера понимания, часть 1: шаги ночного марафона (карточка #4163).

    python scripts/dialog/understanding_corpus/build_marathon.py            # собрать marathon.json
    python scripts/dialog/understanding_corpus/build_marathon.py --check    # сверить файл со сценариями (код 1 - разошлись)

Корпус собирается скриптом из сценариев ``.github/e2e/scenarios/night/*_v1.json`` (SSoT сюжета -
``scripts/e2e/gen_night_marathon.py``), руками не правится. Сценарии ``*_acceptance_v1.json`` в корпус не входят: это
не копии шагов, а акт-уровневые ворота (``expected_tool_calls`` / ``must_not_call`` на весь акт), шагов в них нет.

В файле все 132 шага марафона по порядку актов, у каждого метка ``dry_marathon.classify_step`` (``text`` / ``acoustic``)
и ``scored``. Оцениваются 90 шагов ``text``; 42 ``acoustic`` (биометрия, голос синтеза, тайминг перебивания) идут в
прогоне как контекст - после них у стенда нужное состояние (играет музыка, голос сменён, имя названо), - но в метрики
не входят: их вердикт на стенде справочный. Ожидания шага (``acceptance``, ``patterns``, ``expect``, повторы) взяты из
сценария как есть, без служебных ключей ``_*`` (комментарии сценария).

Заморожен корпус коммитом: ``corpus_sha256`` - хеш содержимого ``acts``; тест ``test_understanding_corpus.py`` сверяет
его, поэтому правка файла руками или перегенерация с другими шагами без нового коммита видны сразу. ``scenarios`` хранит
хеши исходных файлов и последний коммит, менявший каталог сценариев, - откуда корпус взят.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

import dry_marathon as dm  # noqa: E402

CORPUS = _HERE / "marathon.json"
SCENARIOS_REL = ".github/e2e/scenarios/night"
SCHEMA = 1


def clean(value: Any) -> Any:
    """Шаг сценария без служебных ключей ``_*`` (комментарии, ``_result_markers``): ни вердикт, ни прогон их не читают."""
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items() if not str(k).startswith("_")}
    if isinstance(value, list):
        return [clean(v) for v in value]
    return value


def digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def last_commit(path: Path) -> str:
    """Последний коммит, менявший каталог сценариев; пусто - вне git."""
    try:
        out = subprocess.run(["git", "log", "-1", "--format=%H", "--", str(path)], cwd=dm.REPO, check=True,
                             capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError):
        return ""
    return out.stdout.strip()


def build_acts() -> List[Dict[str, Any]]:
    manifest = json.loads(dm.MANIFEST.read_text(encoding="utf-8"))
    acts: List[Dict[str, Any]] = []
    for entry in manifest["acts"]:
        act = dm.load_act(str(entry["act"]))
        steps = []
        for step in act.steps:
            kind, reasons = dm.classify_step(step)
            steps.append({"id": f"{act.key}/{step['label']}", "kind": kind, "scored": kind == dm.TEXT,
                          "acoustic_reasons": reasons, "step": clean(step)})
        acts.append({"act": act.key, "title": act.title, "inherits": act.inherits,
                     "registers_speakers": act.registers_speakers, "writes_memory": act.writes_memory,
                     "steps": steps})
    return acts


def build() -> Dict[str, Any]:
    acts = build_acts()
    flat = [s for a in acts for s in a["steps"]]
    texts = [s["step"]["text"] for s in flat]
    files = sorted(dm.SCENARIOS.glob("night_marathon_*_v1.json"))
    sources = {p.name: sha256_file(p) for p in files if "_acceptance_" not in p.name}
    return {
        "name": "understanding_marathon_part1", "schema": SCHEMA, "part": 1,
        "generator": "scripts/dialog/understanding_corpus/build_marathon.py",
        "scenarios": {"dir": SCENARIOS_REL, "last_commit": last_commit(dm.SCENARIOS), "sha256": sources},
        "counts": {"steps": len(flat), "scored_text": sum(s["scored"] for s in flat),
                   "acoustic_context": sum(not s["scored"] for s in flat), "unique_phrases": len(set(texts))},
        "corpus_sha256": digest(acts),
        "acts": acts,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="сверить marathon.json со сценариями, ничего не писать")
    ap.add_argument("--out", default=str(CORPUS))
    args = ap.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    fresh = build()
    if args.check:
        have = json.loads(Path(args.out).read_text(encoding="utf-8"))
        same = have.get("corpus_sha256") == fresh["corpus_sha256"]
        print(f"корпус {'совпадает' if same else 'РАЗОШЁЛСЯ'} со сценариями: {have.get('corpus_sha256', '')[:12]} "
              f"против {fresh['corpus_sha256'][:12]}; {fresh['counts']}")
        return 0 if same else 1
    Path(args.out).write_text(json.dumps(fresh, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(f"записан {args.out}: {fresh['counts']} sha256={fresh['corpus_sha256'][:12]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
