# -*- coding: utf-8 -*-
"""#4157 (просьба Шифу 28.10): шаг ``n1010_five_facts`` в финале марафона -- «назови пять фактов обо мне».

Шаг проверяет механизм, а не полноту (решение Шифу): память вызвана и вернула хит, в речи не меньше двух фактов Саши
из n203, общей заглушки каталога нет. Судит тот же код, что у живого харнесса: ``e2e_tool_match.step_verdict``
(acceptance) и ``patterns_missing`` (patterns). Строки лога -- формат строк робота; источник строки памяти
(f-строка ``MemorySearchTool.execute``) держит тест ниже.

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4157_five_facts_finale.py -v --no-cov
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, List

import pytest

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", ROOT / ".github" / "workflows" / "scripts"):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

LABEL = "n1010_five_facts"
DONE = (
    "📊 [turn] turn_id=t1 epoch=1 agent=main stage=done status=finished latency_ms=900 llm_calls=2 llm_provider=x "
    "tokens=10"
)
CLOSED_BY_CODE = "📊 [turn] turn_id=t1 epoch=1 agent=main stage=done status=done latency_ms=900 llm_calls=3"
STARTED = "🗣️ SpeechEvent started speech_id=ab12cd34 agent=main epoch=1 text_len=40"
TOOL_CALLED = (
    "📥 Запрос выполнения: memory_search с параметрами {'query': 'факт', 'limit': 10}"
)
#: Строка ``MemorySearchTool.execute`` ровно по его f-строке: ``[memory_search] query={…!r} speaker={…} → N results (vec=…)``.
HITS = "[memory_search] query='факт' speaker=fcdbb976 → 9 results (vec=yes)"
NO_HITS = "[memory_search] query='любить' speaker=fcdbb976 → 0 results (vec=yes)"
FACTS_SAID = "Ты не ешь лук, пьёшь зелёный чай без сахара и ночью чинишь мне мотор."


def _act10() -> List[Dict[str, Any]]:
    return next(a for a in gen.ACTS if a["n"] == 10)["steps"]


def _step(label: str = LABEL) -> Dict[str, Any]:
    return next(s for s in _act10() if s["label"] == label)


def _tts(text: str) -> str:
    return (
        "[tts_node-4] [INFO] [1792795079.69] [tts_node]: 🔊 TTS: speech_id=764009bb, dialogue_id=None, "
        "batch=None None/None, voice=default, lang=default, text='%s'" % text
    )


def _log(said: str, *, called: bool = True, hits: str = HITS, done: str = DONE) -> str:
    lines = (
        ([TOOL_CALLED] if called else [])
        + ([hits] if hits else [])
        + [_tts(said), STARTED, done]
    )
    return "\n".join(lines)


def _verdict(log: str) -> Dict[str, Any]:
    step = _step()
    missing = etm.patterns_missing(log, step["patterns"])
    acc = etm.step_verdict(step["acceptance"], log, label=LABEL)
    return {"missing": missing, "acc": acc, "pass": not missing and acc["pass"]}


# ── место и состав шага ─────────────────────────────────────────────────────────────────────────────────────


def test_n1010_sits_between_memory_after_reset_and_goodnight_with_sashas_voice():
    labels = [s["label"] for s in _act10()]
    at = labels.index(LABEL)
    assert (
        labels[at - 1] == "n1009_memory_context_after_reset"
        and labels[at + 1] == "n1011_goodnight"
    )
    assert at > labels.index(
        "n1006_session_reset"
    ), "факты обязаны идти после сброса сессии"
    assert _step()["voice"] == gen.SASHA


def test_n1010_checks_the_mechanism_not_the_count_of_facts():
    step = _step()
    acc = step["acceptance"]
    assert acc["expected_tool_calls"] == ["memory_search"]
    assert step["patterns"] == [gen.MEMORY_SEARCH_HITS]
    assert acc["expected_keywords"] == gen.SASHA_FACTS and len(gen.SASHA_FACTS) == 2
    assert (
        "#4157" in acc["_comment"]
        and "28.10" in acc["_comment"]
        and "n1006" in acc["_comment"]
    )
    text = step["text"].lower()
    assert "пять фактов" in text and len(text) <= 60
    assert not any(
        alt.strip() in text
        for kw in acc["expected_keywords"]
        for alt in kw.lower().split("|")
    ), "слово вопроса в ключе засчитывает эхо вопроса"


def test_n1010_bans_the_catalog_stub_through_the_step_wide_ban_not_a_copy():
    """Заглушку ``query.fallback_generic`` ставит ``step()`` каждому шагу из каталога фраз; своей копии у n1010 нет."""
    acc = _step()["acceptance"]
    generic = gen.robot_said("query.fallback_generic")
    assert generic and generic in gen.STUB_SPEECH
    assert acc["must_not_say"] == [gen.STUB_SPEECH]
    assert acc["must_not_call"] == [gen.TURN_FALLBACK]


def test_sasha_facts_are_one_table_shared_with_the_neighbours():
    """«Лук» и «зелёный» -- одна таблица: n1003/n1008 и n1001 берут слова из неё, а не пишут своё."""
    onion_steps = ("n1003_memory_search_food", "n1008_memory_survives_reset")
    for label in onion_steps:
        assert _step(label)["acceptance"]["expected_keywords"] == [
            gen.FACT_NO_ONION
        ], label
    n1001 = _step("n1001_recall_night")["acceptance"]["expected_keywords"]
    assert n1001 == [gen.FACT_GREEN_TEA + "|чай без сахара"]
    assert gen.SASHA_FACTS == [gen.FACT_NO_ONION, gen.FACT_GREEN_TEA]


def test_hits_pattern_is_the_one_n213_uses_and_matches_the_robot_log_format():
    n213 = next(
        s
        for a in gen.ACTS
        if a["n"] == 2
        for s in a["steps"]
        if s["label"].startswith("n213_")
    )
    assert n213["patterns"] == [gen.MEMORY_SEARCH_HITS]
    src = (ROOT / "src/rob_box_mcp_tools/rob_box_mcp_tools/tools/memory.py").read_text(
        encoding="utf-8"
    )
    assert 'f"[memory_search] query={query[:40]!r} speaker={speaker_id} "' in src
    assert 'f"→ {len(results)} results "' in src
    assert not etm.patterns_missing(HITS, [gen.MEMORY_SEARCH_HITS])
    assert etm.patterns_missing(
        NO_HITS, [gen.MEMORY_SEARCH_HITS]
    ), "0 results -- не хит"


def test_manifest_counts_n1010_and_json_on_disk_has_it():
    import json

    night = ROOT / ".github" / "e2e" / "scenarios" / "night"
    scenario = json.loads(
        (night / "night_marathon_act10_finale_memory_v1.json").read_text(
            encoding="utf-8"
        )
    )
    on_disk = next(s for s in scenario["steps"] if s["label"] == LABEL)
    assert on_disk == _step()
    manifest = gen.build()["night_marathon_manifest.json"]
    assert next(a for a in manifest["acts"] if a["act"] == 10)["steps"] == len(_act10())


# ── поведение: что краснит шаг, а что нет ───────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "said",
    [
        FACTS_SAID,
        "Не ешь лук. Пьёшь только зеленый чай.",  # «е» вместо «ё»: STT и LLM пишут по-разному
        "Первое: ты Саша. Второе: не ешь лук. Третье: зелёный чай. Больше пока не вспомню.",  # пять не требуем
    ],
)
def test_two_facts_from_memory_with_a_hit_are_green(said):
    got = _verdict(_log(said))
    assert got["pass"], got


@pytest.mark.parametrize(
    "said, why",
    [
        ("Ты не ешь лук.", "один факт из двух"),
        ("Ты пьёшь зелёный чай.", "один факт из двух"),
        ("Я многое о тебе помню, Саша.", "фактов нет"),
        ("Ты Саша, инженер и механик.", "факты, которых n1010 не проверяет"),
    ],
)
def test_fewer_than_two_facts_is_red(said, why):
    got = _verdict(_log(said))
    assert not got["pass"], why
    assert got["acc"]["missing_keywords"], why


def test_catalog_stub_in_speech_is_red_even_when_the_facts_follow_it():
    """#4157: код сказал заглушку вместо фактов, которые память вернула. Любой вариант фразы из каталога -- красный."""
    for stub in gen.ROBOT_PHRASES["query.fallback_generic"]:
        got = _verdict(_log(stub))
        assert not got["pass"] and got["acc"]["forbidden_said"], stub
        got = _verdict(_log(stub + " " + FACTS_SAID))
        assert not got["pass"] and got["acc"]["forbidden_said"], stub


def test_turn_closed_by_code_is_red_even_with_the_facts_said():
    got = _verdict(_log(FACTS_SAID, done=CLOSED_BY_CODE))
    assert not got["pass"] and got["acc"]["forbidden_calls"] == [gen.TURN_FALLBACK]


def test_memory_not_called_is_red():
    """Факты без вызова памяти -- из окна (ответы n1008/n1009 уже держат лук и чай): механизм не доказан."""
    got = _verdict(_log(FACTS_SAID, called=False, hits=""))
    assert not got["pass"] and got["acc"]["missing_expected_calls"] == ["memory_search"]


def test_memory_called_but_empty_is_red():
    got = _verdict(_log(FACTS_SAID, hits=NO_HITS))
    assert not got["pass"] and got["missing"] == [gen.MEMORY_SEARCH_HITS]


def test_silence_is_red():
    got = _verdict("\n".join([TOOL_CALLED, HITS, DONE]))
    assert not got["pass"] and got["acc"]["missing_keywords"] == gen.SASHA_FACTS
