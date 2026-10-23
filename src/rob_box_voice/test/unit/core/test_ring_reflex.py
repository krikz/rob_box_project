"""ADR-0150 PR-3b (#3690): кольцо LED живёт по стадиям хода и отвечает событием на каждое включение."""

from rob_box_dialog import knowledge as kn
from rob_box_voice.core import ring_reflex as rr


class _Ring:
    """Кольцо-заглушка: помнит вызовы; ``alive=False`` - USB-кольца нет."""

    def __init__(self, alive=True):
        self.alive, self.calls = alive, []

    def _do(self, name, *args):
        self.calls.append((name, *args))
        return self.alive

    def __getattr__(self, name):
        return lambda *args: self._do(name, *args)


class _Timers:
    def __init__(self):
        self.pending = []

    def call_later(self, delay, fn):
        timer = type("T", (), {"delay": delay, "fn": fn, "cancelled": False})()
        timer.cancel = lambda: setattr(timer, "cancelled", True)
        self.pending.append(timer)
        return timer

    def fire_last(self):
        timer = self.pending[-1]
        if not timer.cancelled:
            timer.fn()


def _make(alive=True):
    ring, timers, answers = _Ring(alive), _Timers(), []
    reflex = rr.RingReflex(ring, timers.call_later, lambda *a: answers.append(a))
    return reflex, ring, timers, answers


def test_every_ring_mode_of_the_table_has_hardware():
    modes = {row.ring for row in kn.REFLEXES.values() if row.ring}
    assert modes <= set(rr.HARDWARE_MODES) | set(rr.FLASHES)
    assert set(rr.HARDWARE_MODES) | set(rr.FLASHES) <= set(kn.RING_MODES)


def test_addressed_lights_the_speaker_sector_and_answers_started():
    reflex, ring, _timers, answers = _make()
    assert reflex.on_turn_event("t1", "addressed", "started") == "doa_sector"
    assert ring.calls == [("listen",)]
    assert answers == [("t1", True, "")]


def test_each_stage_goes_to_the_hardware_mode_of_its_row():
    for stage in ("addressed", "thinking", "speaking"):
        reflex, ring, _t, _a = _make()
        mode = kn.REFLEXES[stage].ring
        reflex.on_turn_event("t", stage, "started")
        assert ring.calls == [(rr.HARDWARE_MODES[mode],)], stage


def test_done_is_a_green_flash_then_rest_and_failed_a_red_one():
    reflex, ring, timers, _a = _make()
    reflex.on_turn_event("t", "done", "finished")
    assert ring.calls == [("mono", 0, 255, 0)] and timers.pending[-1].delay == rr.FLASH_S
    timers.fire_last()
    assert ring.calls[-1] == ("trace",) and reflex.mode == rr.REST
    reflex.on_turn_event("t", "failed", "failed")
    assert ring.calls[-1] == ("mono", 255, 0, 0)
    reflex.on_turn_event("t", "timeout", "failed")
    assert ring.calls[-1] == ("mono", 255, 0, 0)


def test_a_mode_without_the_next_stage_does_not_hang_forever():
    reflex, ring, timers, _a = _make()
    reflex.on_turn_event("t", "thinking", "started")
    assert timers.pending[-1].delay == rr.HOLD_MAX_S
    timers.fire_last()
    assert ring.calls[-1] == ("trace",)


def test_next_stage_cancels_the_pending_return_to_rest():
    reflex, _ring, timers, _a = _make()
    reflex.on_turn_event("t", "thinking", "started")
    first = timers.pending[-1]
    reflex.on_turn_event("t", "speaking", "started")
    assert first.cancelled and not timers.pending[-1].cancelled


def test_no_device_is_answered_with_skipped_not_with_silence():
    reflex, _ring, _t, answers = _make(alive=False)
    reflex.on_turn_event("t9", "addressed", "started")
    assert answers == [("t9", False, rr.NO_DEVICE)]


def test_stage_without_ring_and_skipped_stage_do_nothing():
    reflex, ring, _t, answers = _make()
    assert reflex.on_turn_event("t", "executing", "started") is None
    assert reflex.on_turn_event("t", "addressed", "skipped") is None
    assert reflex.on_turn_event("t", "no_such_stage", "started") is None
    assert ring.calls == [] and answers == []


def test_silence_turns_the_ring_off_and_stages_answer_skipped_until_it_ends():
    reflex, ring, _t, answers = _make()
    reflex.on_state("SILENCED")
    assert ring.calls == [("off",)] and reflex.mode == kn.REFLEXES["silenced"].ring
    assert reflex.on_turn_event("t", "addressed", "started") is None
    assert answers == [("t", False, rr.SILENCED)]
    reflex.on_state("silenced")
    assert ring.calls == [("off",)]
    reflex.on_state("DIALOGUE")
    assert ring.calls[-1] == ("trace",)


def test_dialogue_states_other_than_silence_never_touch_the_ring():
    reflex, ring, _t, _a = _make()
    for state in ("LISTENING", "DIALOGUE", "IDLE"):
        reflex.on_state(state)
    assert ring.calls == []


def test_silence_scene_of_session_v2_drives_the_ring_like_led_node_does():
    """issue #3911: цепочка ``led_node.session_callback`` - SessionState -> VoiceStateTracker -> RingReflex.

    На ``dialog_engine=v2`` топик ``/voice/dialogue/state`` пуст, тишину даёт сцена ``Silence``.
    """
    from rob_box_core import voice_state
    from rob_box_core.msg_stub import install_dialog_msgs_stub

    msgs = install_dialog_msgs_stub()
    reflex, ring, _t, answers = _make()
    tracker = voice_state.VoiceStateTracker()

    def session_callback(msg):  # тело led_node.LEDNode.session_callback без ROS
        changed = tracker.on_session(msg)
        if changed is not None:
            reflex.on_state("silenced" if changed.detail == voice_state.SILENCED_DETAIL else "idle")

    session_callback(msgs.SessionState(agent="personality", scenes=[msgs.SceneFrame(kind="Silence")]))
    assert ring.calls == [("off",)]
    assert reflex.on_turn_event("t", "addressed", "started") is None and answers == [("t", False, rr.SILENCED)]
    session_callback(msgs.SessionState(agent="personality", scenes=[]))
    assert ring.calls[-1] == ("trace",)
