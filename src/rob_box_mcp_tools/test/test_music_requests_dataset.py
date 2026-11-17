"""ADR-0157 §4.1 / §5 R1–R3 (PR-3, #3560): размеченный набор реплик ``rob_box_music/test/data/music_requests.yaml``.

Грамматика роутера (режим, инструмент, стиль) и путь произведения ``request_music(melody)`` — ``key_slot`` →
``PieceChain.candidates`` по каталогу RTTTL из пакета → ``DeterministicChooser``. Индекса партитур в CI нет (как на
машине без ресурсного пака) — композитор узнаётся по ``artist`` находок. Оценка — ``scripts/research/music_eval.py``
(тот же код читает ``jev_eval.py --domain music``).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.choice import DeterministicChooser

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "research"))
import music_eval  # noqa: E402


@pytest.fixture(scope="module")
def report(tmp_path_factory):
    chain = music_eval.offline_chain(db_dir=str(tmp_path_factory.mktemp("music_requests")))
    return music_eval.evaluate(music_eval.load(), chain, DeterministicChooser())


def test_dataset_covers_run_0910_s7_and_closed_issues():
    items = music_eval.load()
    sources = [i["source"] for i in items]
    assert sources.count("run_0910") == 20 and sources.count("s7") == 10
    assert {f"issue#{n}" for n in (3348, 3264, 3455, 3410, 3176)} <= set(sources)
    assert len({i["id"] for i in items}) == len(items)
    assert all(i.get("instrument") in (None, *kn.INSTRUMENTS) and i.get("style") in (None, *kn.STYLES) for i in items)


def test_grammar_and_chooser_match_the_labels(report):
    """R1 (режим), R2 (инструмент), стиль, R3 (произведение: композитор/название/тональность и первый кандидат —
    не «Mundian To Bach Ke», не «Minority»). Известные дыры (``known_gap``) обязаны расходиться именно там."""
    summary = report["summary"]
    bad = {r["id"]: r["bad"] for r in report["rows"] if r["id"] in summary["failed"]}
    assert bad == {}
    assert summary["known_gap_fixed"] == []  # дыру закрыли — снять метку в наборе
    print("неоднозначные (разрыв < CHOOSER_CLEAR_GAP — вопрос модели в PR-7):", summary["ambiguous"])
    print("хвост LLM (грамматика не закрыла):", summary["llm_tail"], "known_gap:", summary["known_gap"])


def test_bach_never_chooses_the_name_parasite_and_toccata_never_minority(report):
    rows = {r["id"]: r["got"] for r in report["rows"]}
    for rid in ("pr3-bach", "run0910-08", "run0910-11"):
        assert "bach" in rows[rid]["choice"]["composer"].lower(), rows[rid]
    for rid in ("pr3-toccata", "run0910-17"):
        assert rows[rid]["key"] == "D minor" and "toccata" in rows[rid]["choice"]["title"].lower(), rows[rid]
