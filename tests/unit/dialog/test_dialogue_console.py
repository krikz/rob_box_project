"""Консоль ``scripts/dialogue/chat.py`` на ходе v2 (карточка #4163): печать хода, флаги, постоянная память ``--db``.

Консоль получает готовый стенд, поэтому тесты дают ей стенд с фейковым провайдером за НАСТОЯЩИМ мостом команд
(``understanding_measure.fake_stand``). Старого ядра (``AgentCore``, ``local_tools``) в консоли нет.
"""

from __future__ import annotations

import ast
import asyncio
import importlib.util
import json
import sys
from datetime import datetime
from pathlib import Path

import pytest

pytest.importorskip("openai", reason="сухому стенду нужен настоящий мост команд, а ему - клиент openai")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import understanding_measure as umeasure  # noqa: E402

_SPEC = importlib.util.spec_from_file_location("dialogue_chat", REPO / "scripts" / "dialogue" / "chat.py")
chat = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(chat)


def rule(msgs, tools):
    """Фейковая «модель»: заряд - тул, затем слова по данным; музыка - действие; нечто без владельца - тул без фейка."""
    text = next((str(m.content) for m in reversed(msgs) if m.role == "user"), "").split("\n\n")[-1].lower()  # после снимка
    if msgs[-1].role == "tool":
        return {"speech": "Заряд восемьдесят семь процентов."}
    if "заряд" in text:
        return {"actions": [{"tool": "get_battery_level", "args": {}}]}
    if "музык" in text:
        return {"actions": [{"tool": "request_music", "args": {"intent": "track", "text": "лёгкий джаз"}}]}
    if "слушай" in text:
        return {"actions": [{"tool": "listen_for_response", "args": {}}]}
    if "запомни" in text:
        return {"actions": [{"tool": "memory_save", "args": {"fact": "любит чай", "category": "preference"}}]}
    return {"speech": "Хорошо."}


def args_of(*argv):
    chat.disable_colour()
    return chat.build_arg_parser().parse_args(list(argv))


@pytest.fixture
def console(tmp_path):
    made = []

    def make(*argv, memory=None, variant="A"):
        stand = umeasure.fake_stand(rule, tmp_path / f"w{len(made)}", variant, None)
        if memory is not None:
            stand.close()
            stand = umeasure.fake_stand(rule, tmp_path / f"w{len(made)}m", variant, None)
        lines = []
        out = chat.Console(stand, args_of(*argv), lines.append)
        made.append(stand)
        return out, lines

    yield make
    for stand in made:
        stand.close()


def send(con, text):
    return asyncio.run(con.send(text))


def test_ordinary_question_prints_said_tools_outcomes_and_stats(console):
    con, lines = console()
    result = send(con, "Сколько у тебя заряда?")
    text = "\n".join(lines)
    assert "🤖 Заряд восемьдесят семь процентов." in text
    assert "тулы: get_battery_level  →  исходы: get_battery_level: done" in text
    assert "Tier-1: не сработал (ход к модели)" in text
    assert "вызовов LLM 2" in text and "касался: robot_status" in text and "вариант A" in text
    assert result.llm_calls == 2 and result.tools == ["get_battery_level"]


def test_stop_is_tier1_without_llm(console):
    con, lines = console()
    result = send(con, "Робот, стоп")
    text = "\n".join(lines)
    assert "Tier-1: stop_all (код решил сам, без LLM)" in text and "вызовов LLM 0" in text
    assert result.llm_calls == 0 and result.tier1 == "stop_all"


def test_action_outcome_is_shown_and_a_call_is_not_an_execution(console):
    con, lines = console()
    send(con, "Включи музыку")
    send(con, "Слушай меня")  # listen_for_response: у стенда нет фейкового владельца
    text = "\n".join(lines)
    assert "request_music: done" in text and "касался: music" in text
    assert "listen_for_response: rejected(no_fake_owner)" in text  # «вызван» ещё не «исполнен»


def test_debug_shows_the_slice_and_variant_b_a_wider_one(console):
    a, a_lines = console("--debug")
    b, b_lines = console("--debug", variant="B")
    send(a, "Сколько у тебя заряда?")
    send(b, "Сколько у тебя заряда?")
    width = lambda lines: int(next(ln for ln in lines if "срез:" in ln).split("срез:")[1].split("тулов")[0])  # noqa: E731
    assert width(b_lines) > width(a_lines)


def test_without_wake_flag_console_phrase_is_addressed_with_it_it_is_not(console):
    con, lines = console()
    assert send(con, "Привет").addressed
    wake, wake_lines = console("--wake-word")
    assert not send(wake, "Привет").addressed and "не адресовано" in wake_lines[-1]
    assert send(wake, "Робот, привет").addressed


def test_commands(console):
    con, lines = console()
    send(con, "Включи музыку")
    for command in ("/help", "/state", "/tools", "/variant B", "/reset", "/nope"):
        assert asyncio.run(con.command(command)) is True
    text = "\n".join(lines)
    assert "музыка играет=True" in text and "вариант понимания: B" in text and "эпоха 1" in text.replace("эпоха=1", "эпоха 1")
    assert "неизвестная команда /nope" in text
    assert asyncio.run(con.command("/exit")) is False


def test_db_memory_survives_a_restart_and_is_listed_by_facts(tmp_path):
    db = tmp_path / "memory.e2e.db"
    lines = []
    for run in (1, 2):  # два запуска консоли на одном файле памяти (флаг ``--db``)
        stand = umeasure.fake_stand(rule, tmp_path / f"w{run}", "A", None, memory_path=db)
        stand.speakers.register(chat.CONSOLE_VOICE, "Кирилл")  # память привязана к узнанному собеседнику
        con = chat.Console(stand, args_of("--db", str(db)), lines.append)
        if run == 1:
            send(con, "Робот, запомни, что я люблю чай")
        asyncio.run(con.command("/facts"))
        stand.close()
    assert [ln.strip() for ln in lines if ln.startswith("  ")] == ["preference = любит чай"] * 2
    assert db.exists()


def test_db_inside_robot_data_is_refused(tmp_path):
    with pytest.raises(SystemExit):
        chat.build_stand(args_of("--db", "/data/voice_memory.db"), tmp_path / "w")


def time_reply(tmp_path, *argv):
    """Что тул ``get_current_time`` отдал модели в стенде консоли с данными флагами: данные тула из ответа ``tool``."""
    seen = []

    def ask_time(msgs, tools):
        if msgs[-1].role == "tool":
            seen.append(json.loads(msgs[-1].content)["query_results"][0]["data"])
            return {"speech": "Время."}
        return {"actions": [{"tool": "get_current_time", "args": {}}]}

    args = args_of(*argv)
    stand = umeasure.fake_stand(ask_time, tmp_path, "A", None, clock=chat.console_clock(args))
    try:
        before = datetime.now().astimezone()
        asyncio.run(chat.Console(stand, args, lambda line: None).send("Который час?"))
        after = datetime.now().astimezone()
    finally:
        stand.close()
    return seen[0], before, after


def test_console_clock_is_the_real_time_by_default(tmp_path):
    """«Который час?» в консоли - настоящее время суток, а не ночь мастерской из сухого марафона (поручение координатора)."""
    data, before, after = time_reply(tmp_path)
    reported = datetime.fromisoformat(data["iso"])
    assert before.replace(microsecond=0) <= reported <= after
    assert data["date"].startswith(str(before.day)) and str(before.year) in data["date"]
    hour_period = {range(0, 6): "ночь", range(6, 12): "утро", range(12, 18): "день", range(18, 24): "вечер"}
    assert data["period"] in {p for r, p in hour_period.items() if before.hour in r or after.hour in r}
    assert not data["iso"].startswith("2026-10-24T01:4")  # не NIGHT_START


def test_night_clock_flag_brings_back_the_marathon_night(tmp_path):
    data, _, _ = time_reply(tmp_path, "--night-clock")
    assert data["iso"].startswith("2026-10-24T01:4") and data["period"] == "ночь"
    assert data["weekday"] == "суббота" and data["date"] == "24 октября 2026"


def test_console_clock_choice_follows_the_flag():
    from dry_stand import FakeClock, LiveClock

    assert type(chat.console_clock(args_of())) is LiveClock
    assert type(chat.console_clock(args_of("--night-clock"))) is FakeClock


def test_marathon_stand_keeps_the_night_clock_and_the_real_time_format(tmp_path):
    """Сухой марафон не тронут: часы стенда - ночь мастерской; время суток считает настоящий помощник тула, а не строка."""
    stand = umeasure.fake_stand(lambda msgs, tools: {"speech": "ок"}, tmp_path, "A", None)
    try:
        ok, data = stand.tools._t_get_current_time({})  # noqa: SLF001
    finally:
        stand.close()
    assert ok and data["iso"].startswith("2026-10-24T01:4") and data["period"] == "ночь"
    assert data["formatted_time"].startswith("один сорок") or data["formatted_time"].startswith("один ")


def test_flags():
    args = args_of("--once", "раз", "--once", "два", "--variant", "B", "--db", "x.db", "--model", "m", "--debug")
    assert (args.once, args.variant, args.db, args.model, args.debug) == (["раз", "два"], "B", "x.db", "m", True)
    assert args_of().providers == "deepseek" and args_of().variant == "A"
    with pytest.raises(SystemExit):
        args_of("--variant", "C")


def test_the_old_core_is_gone():
    assert not (REPO / "scripts" / "dialogue" / "local_tools.py").exists()
    tree = ast.parse((REPO / "scripts" / "dialogue" / "chat.py").read_text(encoding="utf-8"))
    imported = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module} | {
        a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert not [m for m in imported if "agent_core" in m or "dialogue_state_machine" in m or "local_tools" in m]
    assert not hasattr(chat, "ChatSession")
    assert chat.dm.__name__ == "dry_marathon"
