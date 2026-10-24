"""#3919 (решение Шифу 23.10): v2 помнит, с кем говорит - профиль ведёт код, раздел снимка ``speaker_memory``."""

from __future__ import annotations

import asyncio
import sqlite3
from types import SimpleNamespace

from rob_box_dialog import knowledge as kn
from rob_box_dialog.context import build_context
from rob_box_dialog.session import SpeakerRef
from rob_box_voice.core.speaker_memory import SpeakerMemory

SURE = SpeakerRef(person_ref="spk-boris", name="Борис", name_state="confirmed")
LOG = SimpleNamespace(info=lambda _m: None, warning=lambda _m: None)


def _facts_db(path, rows):
    conn = sqlite3.connect(path)
    conn.execute("CREATE TABLE IF NOT EXISTS voice_facts (id INTEGER PRIMARY KEY, fact TEXT, category TEXT, "
                 "speaker_id TEXT, created_at REAL, updated_at REAL)")
    conn.executemany("INSERT INTO voice_facts (fact, category, speaker_id, created_at, updated_at) "
                     "VALUES (?, 'general', ?, 0, ?)", rows)
    conn.commit()
    conn.close()
    return str(path)


def _memory(tmp_path, rows=(), *, e2e=False, e2e_rows=()):
    return SpeakerMemory(str(tmp_path / "harness_voice.db"), _facts_db(tmp_path / "voice_memory.db", rows),
                         e2e_facts_db=_facts_db(tmp_path / "voice_memory.e2e.db", e2e_rows),
                         is_e2e=lambda: e2e, log=LOG)


def _section(memory, speaker=SURE):
    return asyncio.run(memory.section(speaker))


def test_sure_speaker_gets_dialog_count_and_own_facts(tmp_path):
    memory = _memory(tmp_path, [("любит чай", "spk-boris", 2), ("не ест лук", "spk-boris", 3),
                                ("любит кофе", "spk-sasha", 4), ("общий безымянный", None, 5)])
    block = _section(memory)
    assert block == {"dialog_count": 1, "facts": ["не ест лук", "любит чай"]}  # свежие первыми, чужих и общих нет
    assert "<speaker_memory>" in build_context({"speaker_memory": block})


def test_dialog_count_grows_per_turn_and_survives_restart(tmp_path):
    """Профиль - шов v1 (``note_seen``) в harness_voice.db: счётчик растёт и переживает новый экземпляр ноды."""
    memory = _memory(tmp_path)
    assert [_section(memory)["dialog_count"] for _ in range(3)] == [1, 2, 3]
    assert _section(_memory(tmp_path))["dialog_count"] == 4


def test_not_confirmed_speaker_is_not_profiled(tmp_path):
    memory = _memory(tmp_path, [("любит чай", "spk-boris", 2)])
    for ref in (None, SpeakerRef(), SpeakerRef(person_ref="spk-boris", name="Борис", name_state="tentative"),
                SpeakerRef(person_ref=None, name="Борис", name_state="confirmed")):
        assert _section(memory, ref) is None
    assert not (tmp_path / "harness_voice.db").exists()  # профиль не заводился


def test_facts_are_capped_by_knowledge(tmp_path):
    rows = [(f"факт {i} " + "я" * 500, "spk-boris", i) for i in range(kn.SPEAKER_FACTS_MAX + 5)]
    facts = _section(_memory(tmp_path, rows))["facts"]
    assert len(facts) == kn.SPEAKER_FACTS_MAX and all(len(f) <= kn.SPEAKER_FACT_MAX_CHARS for f in facts)


def test_e2e_leaves_the_live_databases_untouched(tmp_path):
    """e2e (#3904): профиль - в памяти процесса, боевой harness_voice.db не создаётся, боевые факты не читаются."""
    memory = _memory(tmp_path, [("боевой факт живого человека", "spk-boris", 2)], e2e=True,
                     e2e_rows=[("факт e2e", "spk-boris", 2)])
    live = tmp_path / "voice_memory.db"
    before = live.read_bytes()
    first, second = _section(memory), _section(memory)
    assert (first, second) == ({"dialog_count": 1, "facts": ["факт e2e"]}, {"dialog_count": 2, "facts": ["факт e2e"]})
    assert not (tmp_path / "harness_voice.db").exists() and live.read_bytes() == before


def test_memory_failure_does_not_break_the_turn(tmp_path):
    memory = SpeakerMemory(str(tmp_path), str(tmp_path / "absent.db"), log=LOG)  # «файл» профиля - каталог
    assert _section(memory) is None  # профиль не завёлся, фактов нет - раздела нет, исключения нет


def test_empty_paths_mean_no_memory_not_a_default_under_data():
    assert _section(SpeakerMemory("", "", log=LOG)) is None
