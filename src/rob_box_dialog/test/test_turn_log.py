"""Окно истории LLM — только озвученное и структурные результаты (ADR-0150 §7.3, §7.4; PR-8 #3702)."""

from rob_box_dialog import knowledge as kn
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.turn_log import ASSISTANT, TOOL, USER, TurnAggregate, TurnLog, WindowEntry, perform_status


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
    (entry,) = [e for e in log.window() if e.role == TOOL]
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


# -- исход выступления закрывает просьбу (#3973) -------------------------------------------------------------------

def _tale_log():
    """Акт 4: «расскажи сказку» → выступление стартовало (сегменты звучат у владельца выступлений, не в репликах)."""
    log = TurnLog()
    log.begin("t1", 0, "расскажи сказку про красную шапочку", "Борис")
    return log


def test_perform_outcome_closes_the_request_in_window():
    log = _tale_log()
    assert [e.role for e in log.window()] == [USER], "без исхода просьба висит одна - её исполнят заново (#3973)"
    assert log.performed("t1", "tale", "started")
    assert log.performed("t1", "tale", perform_status("halt"), segments_done=3)
    window = log.window()
    assert [e.role for e in window] == [USER, TOOL, ASSISTANT], "одна запись на выступление, статус обновлён на месте"
    assert dict(window[1].content) == {"tool": "perform", "status": "stopped_by_user",
                                       "data": {"kind": "tale", "segments_done": 3}}
    assert window[2] == WindowEntry(ASSISTANT, "[ответ действием: perform tale — остановил человек]"), (
        "в истории модели просьба закрыта репликой кода, а не висит без ответа (#4066)")


def test_perform_end_reasons_map_to_honest_statuses():
    assert perform_status("done") == "done"
    assert perform_status("stopped") == perform_status("halt") == "stopped_by_user"
    assert perform_status("lost") == "interrupted"
    assert perform_status("что-то новое") == "interrupted", "неизвестная причина - не «досказано»"


def test_perform_end_keeps_kind_of_the_start_entry():
    log = _tale_log()
    log.performed("t1", "rap", "started")
    log.performed("t1", "", "done")
    assert dict(log.window()[1].content)["data"]["kind"] == "rap"


def test_perform_outcome_of_unknown_or_reset_turn_writes_nothing():
    log = _tale_log()
    assert not log.performed("чужой", "tale", "done")
    log.reset()
    assert not log.performed("t1", "tale", "done"), "конец выступления прошлой сессии ничего не добавляет"
    assert log.window() == ()


def test_perform_outcome_is_forgotten_with_its_turn():
    log = TurnLog(max_turns=2)
    for n in range(1, 4):
        log.begin(f"t{n}", 0, f"реплика {n}")
        log.performed(f"t{n}", "tale", "started")
    assert not log.performed("t1", "tale", "done"), "ход выпал из окна - записывать некуда"
    assert [dict(e.content)["status"] for e in log.window() if e.role == TOOL] == ["started", "started"]


# -- поздний итог действия исправляет журнал хода (#3960 п.2, марафон 23.10 n809) ------------------------------------

N809 = "поставь фоном ровный ритм на Ренардо, только не громко"


def _phrase(result):
    """Фраза кода об исходе, как её строит ход (``turn.outcome_phrase``)."""
    from rob_box_dialog.turn import outcome_phrase

    return outcome_phrase([result])


def _music(status, ref="t9/0", data=None):
    return ActionResult(turn_id=ref, epoch=0, agent="personality", tool="request_music", cls="action.media",
                        status=status, data=data or {})


def _n809_log():
    """n809: ход заказал музыку, исполнитель закрыл ожидание ``timeout`` на сроке класса, робот сказал фразу кода."""
    log = TurnLog()
    log.begin("t9", 0, N809)
    log.result("t9", _music("timeout"))
    log.said("t9", "s1", _phrase(_music("timeout")))
    return log


def test_late_outcome_replaces_the_timeout_record_and_its_phrase():
    """Трек заиграл через 1,4 с после срока: в окне следующего хода - «done» и «Включаю музыку.», а не таймаут."""
    log = _n809_log()
    assert log.spoken("s1", 24) == "Музыка пока не заиграла."
    assert log.late(_music("done", data={"track_id": "req95964:01:A"}), _phrase)
    log.said("t9", "s2", _phrase(_music("done")))
    log.spoken("s2", 15)
    window = log.window()
    assert [(e.role, e.content) for e in window if e.role != TOOL] == [(USER, N809), (ASSISTANT, "Включаю музыку.")]
    assert [dict(e.content) for e in window if e.role == TOOL] == [
        {"tool": "request_music", "status": "done", "data": {"track_id": "req95964:01:A"}}]


def test_late_outcome_drops_the_timeout_phrase_that_is_still_being_spoken():
    """n809: поздний итог пришёл, пока «Музыка пока не заиграла.» ещё звучала: событие речи - как прозвучало (e2e),
    а в окно модели опровергнутая фраза не идёт."""
    log = _n809_log()
    assert log.late(_music("done"), _phrase)
    assert log.spoken("s1", 24) == "Музыка пока не заиграла."
    window = log.window()
    assert [e.role for e in window] == [USER, TOOL, ASSISTANT]
    assert window[-1].content == "[ответ действием: request_music — сделано]", (
        "опровергнутой фразы в окне нет; ход закрыт репликой кода из позднего итога (#4066)")


def test_late_outcome_keeps_other_words_of_the_robot():
    log = _n809_log()
    log.spoken("s1", 24)
    log.said("t9", "s3", "Ренардо уже прогревается.")
    log.spoken("s3", 25)
    assert log.late(_music("done"), _phrase)
    assert [e.content for e in log.window() if e.role == ASSISTANT] == ["Ренардо уже прогревается."]


def test_late_outcome_of_unknown_or_forgotten_action_changes_nothing():
    log = _n809_log()
    log.spoken("s1", 24)
    before = log.window()
    assert not log.late(_music("done", ref="t8/0"), _phrase), "чужое действие"
    assert log.window() == before
    log.reset()
    assert not log.late(_music("done"), _phrase), "итог действия прошлой сессии ничего не добавляет"


def test_late_outcome_after_a_promise_keeps_the_promise_phrase():
    """Итог после ``pending`` («Еду к…») обновляет запись на месте; обещание было правдой - фраза остаётся."""
    log = TurnLog()
    log.begin("t5", 0, "езжай на кухню")
    nav = dict(turn_id="t5/0", epoch=0, agent="personality", tool="navigate_to_waypoint", cls="action.motion")
    log.result("t5", ActionResult(status="pending", **nav))
    log.said("t5", "s1", "Еду на кухню.")
    log.spoken("s1", 13)
    assert log.late(ActionResult(status="done", data={"waypoint": "кухня"}, **nav), _phrase)
    window = log.window()
    assert [e.content for e in window if e.role == ASSISTANT] == ["Еду на кухню."]
    assert [dict(e.content)["status"] for e in window if e.role == TOOL] == ["done"]
