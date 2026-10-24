"""ADR-0150 PR-3b (#3690): стадии хода v1 -> TurnEvent + один ExpressionRequest по рефлексу, без LLM."""

import pytest
from rob_box_dialog import knowledge as kn

from rob_box_voice.core import turn_reflexes as tr
from rob_box_voice.core import turn_stages as ts
from rob_box_voice.core import turn_tools as tt
from rob_box_voice.core.turn_stages import TurnStages

PERSONALITY = "personality"


class _Rig:
    """TurnStages с записью всего, что ушло наружу, и ручными часами."""

    def __init__(self, fail_express=False):
        self.now, self.events, self.requests, self.logs = 100.0, [], [], []
        self.queue, self.next_id = [], 0
        self.fail_express = fail_express
        self.stages = TurnStages(
            self._express, lambda **kw: self.events.append(kw), epoch=lambda: 7,
            normalize=lambda name: {"happy": "happy", "excited": "happy", "sleep": "sleep"}.get(name, ""),
            clock=lambda: self.now, log=self.logs.append)

    def _express(self, **kw):
        if self.fail_express:
            raise RuntimeError("zenoh down")
        self.requests.append(kw)

    def stage_names(self):
        return [(e["stage"], e["status"]) for e in self.events]

    def speech(self, kind, agent=PERSONALITY, reason="", sid=None):
        """Событие речи; без ``sid`` реплики нумеруются сами: queued заводит новую, конец снимает самую старую."""
        if sid is None:
            if kind == ts.SPEECH_QUEUED:
                self.next_id += 1
                self.queue.append(f"s{self.next_id}")
                sid = self.queue[-1]
            elif kind in ts.SPEECH_DONE_KINDS and self.queue:
                sid = self.queue.pop(0)
            else:
                sid = self.queue[0] if self.queue else "s0"
        self.stages.on_speech(kind, agent, reason, sid)


@pytest.fixture
def rig():
    return _Rig()


def _talking_turn(rig, uid="u1", animation="happy"):
    rig.stages.addressed(uid)
    rig.stages.thinking("deepseek")
    rig.stages.tool_call("speak_text", {"text": "привет", "animation": animation})
    rig.stages.finish(rig.stages.turn_id_for(uid))


def test_addressed_gives_listening_face_and_the_heard_earcon_before_any_llm(rig):
    assert rig.stages.addressed("u1") is True
    (req,) = rig.requests
    assert (req["emotion"], req["earcon"], req["agent"], req["epoch"]) == ("listening", "heard", PERSONALITY, 7)
    assert req["ttl_ms"] == kn.REFLEXES["addressed"].ttl_ms
    assert rig.stage_names() == [("addressed", "started")]
    assert rig.events[0]["turn_id"] == req["turn_id"] and rig.events[0]["source"] == "respeaker"


def test_early_signal_and_admission_of_one_phrase_open_one_turn(rig):
    assert rig.stages.addressed("u1") is True
    assert rig.stages.addressed("u1") is False
    assert len(rig.requests) == 1 and rig.stage_names() == [("addressed", "started")]


def test_phrase_without_id_is_one_phrase_inside_the_window_and_two_after_it(rig):
    rig.stages.addressed(None)
    assert rig.stages.addressed("u9") is False          # допуск привязал id к раннему сигналу
    assert rig.stages.turn_id_for("u9") == rig.events[0]["turn_id"]
    rig.now += tr.SAME_PHRASE_S + 0.1
    assert rig.stages.addressed(None) is True


def test_telegram_is_listened_to_but_not_heard_by_the_robot(rig):
    rig.stages.addressed(None, "telegram")
    assert rig.requests[0]["emotion"] == "listening" and rig.requests[0]["earcon"] == ""


def test_full_spoken_turn_walks_all_stages_in_order_and_ends_done(rig):
    _talking_turn(rig)
    rig.speech(ts.SPEECH_QUEUED)
    rig.speech(ts.SPEECH_STARTED)
    assert rig.stage_names() == [("addressed", "started"), ("thinking", "started"), ("speaking", "started")]
    rig.speech(ts.SPEECH_FINISHED)
    assert rig.stage_names()[-1] == ("done", "finished")
    assert [r["emotion"] for r in rig.requests][-2:] == ["happy", "happy"]   # speaking и done окрашены эмоцией команды
    assert rig.requests[-1]["earcon"] == "done"
    assert len({e["turn_id"] for e in rig.events}) == 1


def test_done_waits_for_the_last_chunk_of_the_speech(rig):
    _talking_turn(rig)
    for _ in range(3):
        rig.speech(ts.SPEECH_QUEUED)
    rig.speech(ts.SPEECH_STARTED)
    rig.speech(ts.SPEECH_FINISHED)
    rig.speech(ts.SPEECH_FINISHED)
    assert rig.stage_names()[-1] == ("speaking", "started")
    rig.speech(ts.SPEECH_FINISHED)
    assert rig.stage_names()[-1] == ("done", "finished")


def test_other_agents_speech_does_not_move_the_turn(rig):
    _talking_turn(rig)
    rig.speech(ts.SPEECH_QUEUED, agent="operator")
    rig.speech(ts.SPEECH_STARTED, agent="operator")
    assert ("speaking", "started") not in rig.stage_names()


def test_promised_speech_that_never_starts_ends_in_timeout_not_in_done(rig):
    _talking_turn(rig)
    rig.now += ts.SPEECH_GRACE_S - 1
    rig.stages.tick()
    assert rig.stage_names()[-1] == ("thinking", "started")
    rig.now += 2
    rig.stages.tick()
    assert rig.stage_names()[-1] == ("timeout", "failed") and rig.events[-1]["reason"] == "speech_not_started"
    assert (rig.requests[-1]["emotion"], rig.requests[-1]["earcon"]) == ("sad", "fail")


def test_silent_turn_ends_done_at_once_and_waits_for_foreign_speech(rig):
    rig.stages.addressed("u1")
    rig.stages.thinking()
    rig.speech(ts.SPEECH_QUEUED)                         # речь прошлого хода ещё в очереди
    rig.stages.finish(rig.stages.turn_id_for("u1"))
    assert rig.stage_names()[-1] == ("thinking", "started")
    rig.speech(ts.SPEECH_FINISHED)
    assert rig.stage_names()[-1] == ("done", "finished")


def test_failed_tool_makes_failed_and_the_reflex_beats_the_command_emotion(rig):
    rig.stages.addressed("u1")
    rig.stages.thinking()
    rig.stages.tool_call("speak_text", {"animation": "happy"})
    rig.stages.tool_call("navigate_to_waypoint", {"name": "нет такой"})
    rig.stages.tool_result("navigate_to_waypoint", True, "waypoint_not_found")
    rig.stages.finish(rig.stages.turn_id_for("u1"))
    assert rig.stage_names()[-1] == ("failed", "failed")
    assert rig.events[-1]["reason"] == "waypoint_not_found"
    assert (rig.requests[-1]["emotion"], rig.requests[-1]["earcon"]) == ("error", "fail")


def test_later_success_of_the_same_tool_clears_its_error(rig):
    rig.stages.addressed("u1")
    rig.stages.tool_call("search_web", {"query": "x"})
    rig.stages.tool_result("search_web", True, "timeout")
    rig.stages.tool_call("search_web", {"query": "x"})
    rig.stages.tool_result("search_web", False)
    rig.stages.finish(rig.stages.turn_id_for("u1"))
    assert rig.stage_names()[-1] == ("done", "finished")


def test_llm_failure_is_failed_with_the_reason(rig):
    rig.stages.addressed("u1")
    rig.stages.thinking()
    rig.stages.finish(rig.stages.turn_id_for("u1"), ok=False, reason="llm_error")
    assert rig.stage_names()[-1] == ("failed", "failed") and rig.events[-1]["reason"] == "llm_error"


def test_rejected_speech_is_a_failure(rig):
    _talking_turn(rig)
    rig.speech(ts.SPEECH_QUEUED)
    rig.speech(ts.SPEECH_REJECTED, reason="unsupported_script")
    assert rig.stage_names()[-1] == ("failed", "failed") and "rejected" in rig.events[-1]["reason"]


def test_executing_carries_the_class_of_the_tool_and_the_motion_direction(rig):
    rig.stages.addressed("u1")
    rig.stages.tool_call("search_web", {"query": "погода"})
    assert (rig.stage_names()[-1], rig.requests[-1]["emotion"]) == (("executing", "started"), "thinking_teeth")
    rig.stages.tool_call("move_direction", {"direction": "left"})
    assert rig.requests[-1]["emotion"] == "turn_left"
    rig.stages.tool_call("move_direction", {"direction": "forward"})
    assert rig.requests[-1]["emotion"] == "accelerating"


def test_speak_text_and_unknown_tools_open_no_executing_stage(rig):
    rig.stages.addressed("u1")
    rig.stages.tool_call("speak_text", {"text": "привет"})
    rig.stages.tool_call("no_such_tool", {})
    assert rig.stage_names() == [("addressed", "started")]


def test_play_animation_shows_the_scene_now_for_the_ordered_time(rig, monkeypatch):
    monkeypatch.setattr(tt, "show_seconds", float)
    rig.stages.addressed("u1")
    rig.stages.tool_call("play_animation", {"animation": "sleep", "duration": 6})
    assert (rig.requests[-1]["emotion"], rig.requests[-1]["ttl_ms"]) == ("sleep", 6000)
    assert rig.stage_names() == [("addressed", "started")]     # стадия та же, рефлекс executing лицо не перебил


def test_play_animation_face_goes_to_the_face_as_validated_slots_with_the_ordered_duration(rig, monkeypatch):
    monkeypatch.setattr(tt, "show_seconds", lambda d: float(d or 0))
    rig.stages.addressed("u1")
    rig.stages.tool_call("play_animation", {"face": {"eyes": "wide", "mouth": "teeth", "tint": "red"}, "duration": 6})
    assert rig.requests[-1]["face"] == {"eyes": "wide", "mouth": "teeth", "tint": "red", "duration_s": 6.0}
    assert (rig.requests[-1]["emotion"], rig.requests[-1]["ttl_ms"]) == ("", 6000)
    rig.stages.tool_call("play_animation", {"animation": "sleep", "face": '{"blink": "wink_left"}'})
    assert rig.requests[-1]["face"] == {"blink": "wink_left", "duration_s": 5.0}
    assert rig.requests[-1]["emotion"] == "sleep"          # анимация и сцена лица - вместе


def test_play_animation_with_an_invalid_face_drops_the_face_not_the_animation(rig):
    rig.stages.addressed("u1")
    before = len(rig.requests)
    rig.stages.tool_call("play_animation", {"face": {"eyes": "neon"}})
    assert len(rig.requests) == before                        # ни сцены, ни лица - нечего показывать
    rig.stages.tool_call("play_animation", {"animation": "sleep", "face": {"color": "#f00"}})
    assert rig.requests[-1]["face"] is None and rig.requests[-1]["emotion"] == "sleep"


def test_unknown_animation_is_ignored_not_shown(rig):
    rig.stages.addressed("u1")
    rig.stages.tool_call("play_animation", {"animation": "moonwalk"})
    rig.stages.tool_call("speak_text", {"animation": "moonwalk"})
    assert len(rig.requests) == 1


def test_neutral_animation_does_not_paint_the_turn(rig):
    _talking_turn(rig, animation="neutral")
    rig.speech(ts.SPEECH_QUEUED)
    rig.speech(ts.SPEECH_STARTED)
    assert rig.requests[-1]["emotion"] == "talking"


def test_new_phrase_supersedes_the_turn_in_flight_with_a_skipped_event(rig):
    rig.stages.addressed("u1")
    rig.stages.thinking()
    first = rig.events[0]["turn_id"]
    rig.stages.addressed("u2")
    assert (rig.events[-2]["turn_id"], rig.events[-2]["status"], rig.events[-2]["reason"]) == (first, "skipped", "superseded")
    assert rig.events[-1]["turn_id"] != first
    rig.stages.finish(first)                                     # опоздавший конец старого хода нового не трогает
    assert rig.stage_names()[-1] == ("addressed", "started")


def test_dropped_phrase_calms_the_face_and_closes_the_turn(rig):
    rig.stages.addressed("u1")
    rig.stages.dropped("u1", "silenced")
    assert (rig.events[-1]["status"], rig.events[-1]["reason"]) == ("skipped", "silenced")
    assert (rig.requests[-1]["emotion"], rig.requests[-1]["ttl_ms"]) == (ts.CALM_SCENE, ts.CALM_TTL_MS)
    rig.stages.dropped("u1", "again")
    assert len(rig.events) == 2


def test_cancelled_turn_ends_without_a_reflex(rig):
    rig.stages.addressed("u1")
    rig.stages.thinking()
    count = len(rig.requests)
    rig.stages.finish(rig.stages.turn_id_for("u1"), cancelled=True)
    assert (rig.events[-1]["status"], rig.events[-1]["reason"]) == ("skipped", "cancelled")
    assert len(rig.requests) == count


def test_turn_handled_by_code_settles_then_done(rig):
    rig.stages.addressed("u1")
    rig.stages.handled("u1")
    assert rig.stage_names()[-1] == ("addressed", "started")
    rig.now += ts.HANDLED_SETTLE_S + 0.1
    rig.stages.tick()
    assert rig.stage_names()[-1] == ("done", "finished")


def test_turn_handled_by_code_waits_for_its_phrase(rig):
    rig.stages.addressed("u1")
    rig.speech(ts.SPEECH_QUEUED)
    rig.stages.handled("u1")
    rig.speech(ts.SPEECH_STARTED)
    rig.now += ts.HANDLED_SETTLE_S + 0.1
    rig.stages.tick()
    assert rig.stage_names()[-1] == ("speaking", "started")
    rig.speech(ts.SPEECH_FINISHED)
    assert rig.stage_names()[-1] == ("done", "finished")


def _fails(rig):
    return [r for r in rig.requests if r["earcon"] == "fail"]


def test_router_lookup_miss_returned_to_llm_is_not_a_failed_turn(rig):
    """#3794: медиароутер забрал «включи звук X», поиск промахнулся, фраза ушла в LLM: earcon fail не звучит."""
    rig.stages.addressed(None)
    rig.stages.hold()
    rig.stages.handled(None)
    rig.stages.tool_call("music_library", {})
    rig.stages.tool_result("music_library", True, "Мелодия не найдена")
    rig.now += ts.HANDLED_SETTLE_S + 0.1
    rig.stages.tick()                                   # пока ищем, ход не закрывается
    assert not _fails(rig) and rig.stage_names()[-1][0] == "executing"
    rig.stages.hold(reopen=True)
    assert rig.stages.turn_id_for(None) == rig.events[0]["turn_id"]   # фраза без id (инъекция) - тот же ход
    rig.stages.thinking("deepseek")
    rig.stages.tool_call("speak_text", {"text": "нет такого", "animation": "happy"})
    rig.stages.finish(rig.stages.turn_id_for(None))
    rig.speech(ts.SPEECH_QUEUED)
    rig.speech(ts.SPEECH_STARTED)
    rig.speech(ts.SPEECH_FINISHED)
    assert not _fails(rig) and rig.stage_names()[-1] == ("done", "finished")


def test_router_found_tune_closes_the_held_turn_after_its_phrase(rig):
    rig.stages.addressed("u1")
    rig.stages.hold()
    rig.stages.handled("u1")
    rig.now += ts.HANDLED_SETTLE_S + 0.1
    rig.stages.hold(reopen=False)
    rig.stages.tick()
    assert rig.stage_names()[-1] == ("done", "finished")


def test_really_failed_tool_still_plays_the_fail_earcon(rig):
    rig.stages.addressed("u1")
    rig.stages.thinking()
    rig.stages.tool_call("music_library", {})
    rig.stages.tool_result("music_library", True, "boom")
    rig.stages.finish(rig.stages.turn_id_for("u1"))
    assert len(_fails(rig)) == 1 and rig.stage_names()[-1][0] == "failed"


def test_done_alternates_faces_between_turns(rig):
    faces = []
    for n in range(2):
        rig.stages.addressed(f"u{n}")
        rig.stages.finish(rig.stages.turn_id_for(f"u{n}"))
        faces.append(rig.requests[-1]["emotion"])
    assert sorted(faces) == sorted(kn.REFLEXES["done"].face)


def test_publish_failures_never_break_the_dialogue():
    rig = _Rig(fail_express=True)
    rig.stages.addressed("u1")
    rig.stages.thinking()
    rig.stages.finish(rig.stages.turn_id_for("u1"))
    assert rig.stage_names()[-1] == ("done", "finished")
    assert any("не ушёл" in line for line in rig.logs)


def test_executor_answers_are_logged_and_skips_are_visible(rig):
    rig.stages.addressed("u1")
    tid = rig.events[0]["turn_id"]
    assert rig.stages.on_expression_event("face", tid, True, "").startswith("[reflex] executor=face")
    assert "started +" in rig.logs[-1]
    rig.stages.on_expression_event("ring", tid, False, "no_device")
    assert rig.logs[-1].endswith("skipped reason=no_device")


def test_events_without_a_turn_are_ignored(rig):
    rig.stages.thinking()
    rig.stages.tool_call("search_web", {})
    rig.stages.tool_result("search_web", True)
    rig.stages.expect_speech()
    rig.stages.finish(None)
    rig.stages.tick()
    rig.speech(ts.SPEECH_STARTED)
    assert rig.events == [] and rig.requests == []


@pytest.fixture
def real_tool_animations(monkeypatch):
    """Настоящий ``rob_box_mcp_tools/animations.py`` по пути: тест не зависит от заглушек пакета в чужих conftest."""
    import importlib.util
    import sys
    import types
    from pathlib import Path

    path = Path(ts.__file__).resolve().parents[3] / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "animations.py"
    spec = importlib.util.spec_from_file_location("rob_box_mcp_tools.animations", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools", types.ModuleType("rob_box_mcp_tools"))
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.animations", module)
    return module


def test_show_seconds_is_the_rule_of_the_tool(real_tool_animations):
    assert tt.show_seconds(None) == 0.0
    assert tt.show_seconds(6) == 6.0 and tt.show_seconds("12") == 12.0
    assert tt.show_seconds(1) == 2.0 and tt.show_seconds(300) == 2.0     # вне [2, 30] - минимум, как в туле
    assert tt.show_seconds("долго") == 0.0


def test_scene_for_animation_uses_the_tool_vocabulary(real_tool_animations):
    assert tt.scene_for_animation("happy") == "happy"
    assert tt.scene_for_animation("Excited") == "happy"          # псевдоним словаря тулов
    assert tt.scene_for_animation("police_lights") == "police_lights"
    assert tt.scene_for_animation("moonwalk") == ""
    assert tt.scene_for_animation("") == ""


def test_without_the_tool_package_there_is_no_scene_and_no_crash(monkeypatch):
    import sys

    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.animations", None)      # import -> ImportError
    assert tt.scene_for_animation("happy") == "" and tt.show_seconds(6) == 0.0


def test_error_reason_takes_the_error_field_or_the_start_of_the_text():
    assert tt.error_reason('{"success": false, "error": "waypoint_not_found", "message": "x"}') == "waypoint_not_found"
    assert tt.error_reason('{"message": "точка не найдена"}') == "точка не найдена"
    assert tt.error_reason("просто   текст\nошибки") == "просто текст ошибки"
    assert tt.error_reason("x" * 200) == "x" * 60
    assert tt.error_reason(None) == ""


def test_lost_terminal_event_does_not_hold_the_turn_forever(rig):
    rig.stages.addressed("u1")
    rig.speech(ts.SPEECH_QUEUED, sid="lost")             # конец этой реплики так и не пришёл
    rig.stages.finish(rig.stages.turn_id_for("u1"))
    assert rig.stage_names()[-1] == ("addressed", "started")
    rig.now += ts.SPEECH_ID_TTL_S + 1
    rig.stages.tick()
    assert rig.stage_names()[-1] == ("done", "finished")


def test_end_event_of_an_unknown_speech_does_not_cancel_a_queued_one(rig):
    rig.stages.addressed("u1")
    rig.speech(ts.SPEECH_QUEUED, sid="a")
    rig.speech(ts.SPEECH_FINISHED, sid="zzz")            # чужой/повторный конец
    rig.stages.finish(rig.stages.turn_id_for("u1"))
    assert rig.stage_names()[-1] == ("addressed", "started")
    rig.speech(ts.SPEECH_FINISHED, sid="a")
    assert rig.stage_names()[-1] == ("done", "finished")


def test_phrase_without_id_gets_the_turn_opened_for_it_but_a_retry_does_not(rig):
    """Telegram / оператор / инъекция без utterance_id: ход LLM берёт id рефлексов, иначе finish() не найдёт ход."""
    rig.stages.addressed(None, "telegram")
    tid = rig.events[0]["turn_id"]
    assert rig.stages.turn_id_for(None) == tid
    rig.stages.thinking()
    assert rig.stages.turn_id_for(None) is None            # синтетический ретрай продолжает ход, нового id не берёт
    rig.stages.finish(tid)
    assert rig.stage_names()[-1] == ("done", "finished")


def test_phrase_with_id_never_takes_a_turn_of_another_phrase(rig):
    rig.stages.addressed("u1")
    assert rig.stages.turn_id_for("u2") is None and rig.stages.turn_id_for(None) is None
