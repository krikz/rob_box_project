#!/usr/bin/env python3
"""Музыкальные вопросы на размеченном наборе (ADR-0157 §4.1–§4.2, PR-3 #3560): режим, инструмент, стиль и выбор
произведения среди кандидатов, построенных кодом.

Режим/инструмент/стиль — грамматика роутера (``media_command_grammar``); произведение — тот же путь, что у
``request_music(melody)``: ``key_slot`` → ``engine.piece.PieceChain.candidates`` (поиск темы по каталогу RTTTL,
индекс партитур, если указан ``--scores``) → вопросы по уровням → ``Chooser``. Сеть не вызывается; модельный
провайдер — только подставной (``jev_eval.py --domain music --provider fake-*``), настоящий — PR-7.

  PYTHONPATH=src/rob_box_mcp_tools:src/rob_box_music:src/rob_box_voice:src/rob_box_core \\
    python scripts/research/music_eval.py [--scores /opt/rob_box/scores]
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

import yaml

REPO = Path(__file__).resolve().parents[2]
DATASET = REPO / "src" / "rob_box_music" / "test" / "data" / "music_requests.yaml"
#: Режим реплики по интенту грамматики; ``none`` — грамматика не закрыла реплику (хвост LLM).
_MODES = {"stop": "stop", "dj": "set", "play_named": "piece", "request_music": "piece"}


def load(path: Path = DATASET) -> List[Dict[str, Any]]:
    return list(yaml.safe_load(path.read_text(encoding="utf-8"))["requests"])


def offline_chain(scores_dir: Optional[str] = None, db_dir: Optional[str] = None) -> Any:
    """``PieceChain`` над каталогом RTTTL из пакета (SQLite во временном каталоге) и индексом партитур
    ``scores_dir`` (нет — пустой индекс, как на машине без ресурсного пака)."""
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
    from rob_box_mcp_tools.engine.classic import classic_picker
    from rob_box_mcp_tools.engine.piece import PieceChain
    from rob_box_mcp_tools.engine.score_library import ScoreLibrary
    from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, library_records, theme_finder, word_hooks

    box: Dict[str, Any] = {}

    def library() -> Any:
        if "lib" not in box:
            box["lib"] = RtttlLibrary(db_path=str(Path(db_dir or tempfile.mkdtemp()) / "music_eval.db"))
        return box["lib"]

    scores = ScoreLibrary(scores_dir or str(Path(tempfile.mkdtemp()) / "no_scores"))
    dj = DjSetTool(None, None, melodies=lambda ids: {}, finder=theme_finder(library), scores=scores, lines=False,
                   titles=lambda ids: {})
    return PieceChain(dj, library_records(library), word_hooks(library), classic_picker(library))


def analyze(text: str, chain: Any, chooser: Any) -> Dict[str, Any]:
    """Что код выделил из реплики и что выбрал бы (без звука)."""
    from rob_box_music.works import key_slot
    from rob_box_voice.core.media_command_grammar import MediaIntent, asks_for_set, instrument_in, parse_media_command

    command = parse_media_command(text)
    out: Dict[str, Any] = {"intent": command.intent.value, "mode": _MODES.get(command.intent.value, "none"),
                           "mode_rule": "set" if asks_for_set(text) else "piece", "instrument": instrument_in(text),
                           "style": command.style or ""}
    if command.intent is not MediaIntent.PLAY_NAMED:
        return out
    work, key = key_slot(command.name)
    started = time.perf_counter()
    ref, candidates, records = chain.candidates(work, key)
    questions = chain.questions(work, ref, candidates, records)
    choice = chooser.choose(questions[0]) if questions else None
    by_id = {c.id: c for c in candidates}
    top = by_id.get(choice.id) if choice else None
    out.update(work=work, key=key, composer=[list(w.spellings) for w in ref.composer],
               title=[list(w.spellings) for w in ref.title], candidates=len(candidates),
               ambiguous=bool(questions) and _gap(chooser, questions[0]) < _clear_gap(),
               choice=None if top is None else {"id": top.id, "kind": top.kind, "title": top.title,
                                                "composer": top.composer, "provider": choice.provider},
               ms=round((time.perf_counter() - started) * 1000, 1))
    return out


def _gap(chooser: Any, question: Any) -> float:
    ranked = chooser.rank(question)
    return ranked[0][0] - ranked[1][0] if len(ranked) > 1 else float("inf")


def _clear_gap() -> float:
    from rob_box_music import knowledge as kn
    return kn.CHOOSER_CLEAR_GAP


def check(item: Mapping[str, Any], got: Mapping[str, Any]) -> List[str]:
    """Расхождения ``got`` с разметкой ``item`` (пусто — совпало); неприменимое (контекст сета) — не проверяется."""
    if item.get("context"):
        return []
    bad = []
    mode = item.get("mode")
    if mode in ("piece", "set") and got["mode_rule"] != mode:
        bad.append(f"режим {got['mode_rule']} ≠ {mode}")
    if mode == "stop" and got["mode"] != "stop":
        bad.append(f"режим {got['mode']} ≠ stop")
    if (item.get("instrument") or "") != got["instrument"]:
        bad.append(f"инструмент {got['instrument']!r} ≠ {item.get('instrument')!r}")
    if item.get("style") and item["style"] != got["style"]:
        bad.append(f"стиль {got['style']!r} ≠ {item['style']!r}")
    work = item.get("work") or {}
    if work and "work" in got:
        bad += _check_work(work, item.get("top") or {}, got)
    return bad


def _check_work(work: Mapping[str, Any], top: Mapping[str, Any], got: Mapping[str, Any]) -> List[str]:
    bad = []
    if work.get("composer") and not any(work["composer"] in w for w in got["composer"]):
        bad.append(f"композитор {got['composer']} без {work['composer']!r}")
    for word in str(work.get("title") or "").split():
        if not any(any(s.startswith(word[:5]) for s in w) for w in got["title"]):
            bad.append(f"название {got['title']} без {word!r}")
    if (work.get("key") or "") != got["key"]:
        bad.append(f"тональность {got['key']!r} ≠ {work.get('key')!r}")
    choice = got.get("choice") or {}
    hay = {"title": str(choice.get("title", "")).lower(), "artist": str(choice.get("composer", "")).lower()}
    every = " ".join([*hay.values(), str(choice.get("id", "")).lower()])
    if top.get("title_has") and top["title_has"] not in hay["title"]:
        bad.append(f"выбрано {choice or '—'}: в названии нет {top['title_has']!r}")
    if top.get("artist_has") and top["artist_has"] not in hay["artist"]:
        bad.append(f"выбрано {choice or '—'}: автор не {top['artist_has']!r}")
    if top.get("has") and top["has"] not in every:
        bad.append(f"выбрано {choice or '—'}: нет {top['has']!r}")
    if top.get("not_title") and top["not_title"] in hay["title"]:
        bad.append(f"выбрано {choice}: паразит {top['not_title']!r}")
    return bad


def evaluate(items: List[Mapping[str, Any]], chain: Any, chooser: Any) -> Dict[str, Any]:
    """Построчно и сводка: точность по проверяемым строкам, неоднозначные, хвост LLM, время кандидатов."""
    rows, times = [], []
    for item in items:
        got = analyze(item["text"], chain, chooser)
        bad = check(item, got)
        rows.append({"id": item["id"], "text": item["text"], "ok": not bad, "bad": bad, "got": got,
                     "needs_shifu": bool(item.get("needs_shifu")), "context": item.get("context"),
                     "known_gap": item.get("known_gap")})
        if "ms" in got:
            times.append(got["ms"])
    checked = [r for r in rows if not r["context"]]
    return {"rows": rows, "summary": {
        "items": len(rows), "checked": len(checked), "ok": sum(r["ok"] for r in checked),
        "failed": [r["id"] for r in checked if not r["ok"] and not r["known_gap"]],
        "known_gap": [r["id"] for r in checked if not r["ok"] and r["known_gap"]],
        "known_gap_fixed": [r["id"] for r in checked if r["ok"] and r["known_gap"]],
        "ambiguous": [r["id"] for r in rows if r["got"].get("ambiguous")],
        "llm_tail": [r["id"] for r in rows if r["got"]["mode"] == "none" and r["context"] is None],
        "needs_shifu": [r["id"] for r in rows if r["needs_shifu"]],
        "skipped_context": [r["id"] for r in rows if r["context"]],
        "candidates_ms_p50": statistics.median(times) if times else None,
        "candidates_ms_max": max(times) if times else None}}


def main(argv: Optional[List[str]] = None) -> int:
    from rob_box_music.choice import DeterministicChooser

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", type=Path, default=DATASET)
    ap.add_argument("--scores", help="каталог библиотеки партитур (score_index.db + JSON)")
    args = ap.parse_args(argv)
    report = evaluate(load(args.dataset), offline_chain(args.scores), DeterministicChooser())
    json.dump(report, sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")
    summary = report["summary"]
    return 0 if not summary["failed"] and not summary["known_gap_fixed"] else 1


if __name__ == "__main__":
    sys.exit(main())
