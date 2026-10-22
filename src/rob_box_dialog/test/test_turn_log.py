"""Окно истории LLM — только озвученное и структурные результаты (ADR-0150 §7.3, §7.4; PR-8 #3702)."""

from rob_box_dialog import knowledge as kn
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.turn_log import ASSISTANT, TOOL, USER, TurnAggregate, TurnLog, WindowEntry


def _result(tool="play_sound", status="done", reason=None, data=None):
    return ActionResult(turn_id="t1/0", epoch=0, agent="personality", tool=tool, cls="action.express",
                        status=status, reason=reason, data=data or {})


def test_assistant_is_only_the_spoken_part():
    log = TurnLog()
    log.begin("t1", 0, "расскажи сказку")
    log.said("t1", "s1", "Жил-был робот. Он пел песни.")
    assert log.window() == (WindowEntry(USER, "расскажи сказку"),), "до события речи ответа в окне нет"
    assert log.spoken("s1", 14) == "Жил-был робот."
    assert log.window() == (WindowEntry(USER, "расскажи сказку"), WindowEntry(ASSISTANT, "Жил-был робот."))


def test_unspoken_reply_never_enters_window():
    log = TurnLog()
    log.begin("t1", 0, "привет")
    log.said("t1", "s1", "Привет!")
    assert log.spoken("s1", 0) is None
    assert [e.role for e in log.window()] == [USER]


def test_foreign_or_late_speech_event_is_ignored():
    log = TurnLog()
    log.begin("t1", 0, "привет")
    log.said("t1", "s1", "Привет!")
    assert log.spoken("чужая", 7) is None
    log.reset()
    assert log.spoken("s1", 7) is None, "событие речи прошлой сессии ничего не добавляет"
    assert log.window() == ()


def test_text_channel_reply_is_delivered_whole():
    log = TurnLog()
    log.begin("t1", 0, "как дела")
    log.said("t1", "", "Нормально.", delivered=True)
    assert log.window()[-1] == WindowEntry(ASSISTANT, "Нормально.")


def test_tool_result_is_structure_not_text():
    log = TurnLog()
    log.begin("t1", 0, "звук")
    log.result("t1", _result(status="rejected", reason="busy", data={"name": "boop"}))
    entry = log.window()[-1]
    assert entry.role == TOOL
    assert dict(entry.content) == {"tool": "play_sound", "status": "rejected", "reason": "busy",
                                   "data": {"name": "boop"}}
    assert entry.text() == '{"tool":"play_sound","status":"rejected","reason":"busy","data":{"name":"boop"}}'


def test_rejected_command_leaves_only_user_text():
    """Невалидная команда (#3145) — ни речи, ни результата: в окне только реплика человека."""
    log = TurnLog()
    log.begin("t1", 0, "сделай что-нибудь")
    assert log.window() == (WindowEntry(USER, "сделай что-нибудь"),)


def test_user_text_has_no_markers_added():
    log = TurnLog()
    log.begin("t1", 0, "  включи свет  ")
    assert log.window()[0].content == "включи свет"


def test_window_keeps_last_turns():
    log = TurnLog()
    for n in range(kn.TURN_WINDOW_MAX_TURNS + 5):
        log.begin(f"t{n}", 0, f"реплика {n}")
    window = log.window()
    assert len(window) == kn.TURN_WINDOW_MAX_TURNS
    assert window[0].content == "реплика 5" and window[-1].content == f"реплика {kn.TURN_WINDOW_MAX_TURNS + 4}"


def test_window_token_budget_drops_whole_old_turns():
    log = TurnLog(max_tokens=10, estimate=len)
    log.begin("t1", 0, "aaaa")
    log.said("t1", "s1", "bbbb")
    log.spoken("s1", 4)
    log.begin("t2", 0, "cccc")
    log.said("t2", "s2", "dd")
    log.spoken("s2", 2)
    assert [e.content for e in log.window()] == ["cccc", "dd"], "ход t1 выпал целиком, не по половине"
    assert log.tokens() == 6


def test_newest_turn_is_kept_even_over_budget():
    log = TurnLog(max_tokens=3, estimate=len)
    log.begin("t1", 0, "очень длинная реплика")
    assert [e.content for e in log.window()] == ["очень длинная реплика"]


def test_turns_beyond_window_are_forgotten_with_their_speech():
    log = TurnLog(max_turns=1)
    log.begin("t1", 0, "раз")
    log.said("t1", "s1", "Первый.")
    log.begin("t2", 0, "два")
    assert log.spoken("s1", 7) is None


def test_reset_clears_window():
    log = TurnLog()
    log.begin("t1", 0, "запомни")
    log.result("t1", _result())
    log.reset()
    assert log.window() == ()


def test_aggregate_line_has_no_text():
    agg = TurnAggregate("t1", 2, "personality", "done", "ok", latency_ms=1834, llm_calls=1,
                        llm_provider="deepseek", tokens=5120)
    line = agg.log_line()
    assert line == ("📊 [turn] turn_id=t1 epoch=2 agent=personality stage=done status=ok latency_ms=1834 "
                    "llm_calls=1 llm_provider=deepseek tokens=5120")
    assert set(vars(agg)) == {"turn_id", "epoch", "agent", "stage", "status", "latency_ms", "llm_calls",
                              "llm_provider", "tokens"}
