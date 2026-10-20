"""ProviderBoard: доска писателей /voice/providers/state, устаревшее = unknown (ADR-0150 §8, PR-4 ч.2, #3693)."""

from __future__ import annotations

from rob_box_dialog.degrade import ProviderAnnouncer, ProviderBoard, ProviderState, provider_order, phrase_on_change
from rob_box_dialog.knowledge import PROVIDER_STALE_S


def test_board_merges_writers_by_kind_and_name():
    b = ProviderBoard()
    b.update([ProviderState("minimax", "tts", "ok")], now=0.0)
    b.update([ProviderState("yandex", "stt", "quota")], now=1.0)
    got = {(s.kind, s.name): s.health for s in b.states(now=2.0)}
    assert got == {("tts", "minimax"): "ok", ("stt", "yandex"): "quota"}


def test_board_republish_replaces_previous_state_of_same_key():
    b = ProviderBoard()
    b.update([ProviderState("minimax", "tts", "ok")], now=0.0)
    b.update([ProviderState("minimax", "tts", "quota")], now=2.0)
    assert [s.health for s in b.states(now=3.0)] == ["quota"]


def test_board_entry_without_updates_becomes_unknown_after_stale_window():
    b = ProviderBoard()
    b.update([ProviderState("minimax", "tts", "quota")], now=0.0)
    assert b.states(now=PROVIDER_STALE_S)[0].health == "quota"
    assert b.states(now=PROVIDER_STALE_S + 0.1)[0].health == "unknown"


def test_unknown_is_silent_and_not_dropped_from_order():
    stale = ProviderState("minimax", "tts", "unknown")
    assert phrase_on_change(None, stale) is None
    assert provider_order([stale], ["minimax", "yandex"], kind="tts") == ("yandex", "minimax")  # неизвестный не отброшен, но после здоровых


def test_board_kind_filter():
    b = ProviderBoard()
    b.update([ProviderState("a", "tts", "ok"), ProviderState("b", "llm", "ok")], now=0.0)
    assert [s.kind for s in b.states(now=1.0, kinds=("tts",))] == ["tts"]


def test_dead_writer_goes_unknown_and_stays_silent_for_announcer():
    b = ProviderBoard()
    ann = ProviderAnnouncer()
    b.update([ProviderState("minimax", "tts", "ok"), ProviderState("yandex", "tts", "ok")], now=0.0)
    assert ann.observe(b.states(now=1.0), ["minimax", "yandex"], now=1.0) == []
    assert ann.observe(b.states(now=60.0), ["minimax", "yandex"], now=60.0) == []  # писатель замолчал - не отказ
