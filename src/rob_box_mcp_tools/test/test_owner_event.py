"""owner_event: одно ожидание события владельца по id запроса для всех тулов (ADR-0150 §5.2-5.3, PR-7 ч.7b)."""

import threading
import types

from rob_box_mcp_tools.owner_event import OwnerEvents, owner_events


class _Node:
    def __init__(self):
        self.subs = []

    def create_subscription(self, typ, topic, cb, depth):
        self.subs.append((topic, cb))


def _msg(rid, kind="ok"):
    return types.SimpleNamespace(rid=rid, kind=kind)


def _events(node, **kw):
    return OwnerEvents(node, object, "/owner/event", key_of=lambda m: m.rid, **kw)


def test_event_with_the_request_id_answers_the_request():
    node = _Node()
    events = _events(node)
    (_topic, cb), = node.subs
    rid, got = events.ask(lambda r: threading.Thread(target=cb, args=(_msg(r),)).start(), 2.0)
    assert got.rid == rid


def test_first_event_decides_later_ones_are_ignored():
    node = _Node()
    events = _events(node)
    (_topic, cb), = node.subs

    def publish(rid):
        cb(_msg(rid, "started"))
        cb(_msg(rid, "skipped"))

    assert events.ask(publish, 1.0)[1].kind == "started"


def test_other_requests_and_rejected_kinds_are_not_answers():
    node = _Node()
    events = _events(node, accept=lambda m: m.kind == "voice_changed")
    (_topic, cb), = node.subs

    def publish(rid):
        cb(_msg("someone-else", "voice_changed"))
        cb(_msg(rid, "finished"))

    assert events.ask(publish, 0.1)[1] is None


def test_no_event_until_deadline_is_none_and_the_waiter_is_dropped():
    node = _Node()
    events = _events(node)
    rid, got = events.ask(lambda r: None, 0.05)
    assert got is None and rid not in events._waiters
    node.subs[0][1](_msg(rid))  # опоздавшее событие никого не ждёт и не падает


def test_one_subscription_per_node_and_kind_of_request():
    node = _Node()
    first = owner_events(node, "sound", lambda: _events(node))
    assert owner_events(node, "sound", lambda: _events(node)) is first
    assert len(node.subs) == 1


def test_explicit_request_id_is_the_correlation_key():
    """Владелец сам возвращает id (``utterance_id`` регистрации, #3793) — ждём его, а не новый uuid."""
    node = _Node()
    events = _events(node)
    (_topic, cb), = node.subs
    rid, got = events.ask(lambda r: cb(_msg(r)), 1.0, request_id="utt-1")
    assert rid == "utt-1" and got.rid == "utt-1"


def test_remembered_event_answers_a_request_sent_by_someone_else():
    """Запрос отправил другой (робот сам, до вызова тула) — исход уже пришёл: берём его без ожидания."""
    node = _Node()
    events = _events(node, remember=2)
    (_topic, cb), = node.subs
    cb(_msg("utt-1", "too_short"))
    assert events.ask(None, 5.0, request_id="utt-1")[1].kind == "too_short"


def test_waiting_for_someone_elses_request_gets_the_late_event():
    node = _Node()
    events = _events(node, remember=2)
    (_topic, cb), = node.subs
    threading.Timer(0.05, cb, args=(_msg("utt-1"),)).start()
    assert events.ask(None, 2.0, request_id="utt-1")[1].rid == "utt-1"


def test_own_request_does_not_take_a_stale_remembered_event():
    node = _Node()
    events = _events(node, remember=2)
    (_topic, cb), = node.subs
    cb(_msg("utt-1", "old"))
    assert events.ask(lambda r: None, 0.05, request_id="utt-1")[1] is None


def test_memory_keeps_only_the_last_events():
    node = _Node()
    events = _events(node, remember=2)
    (_topic, cb), = node.subs
    for rid in ("a", "b", "c"):
        cb(_msg(rid))
    assert list(events._recent) == ["b", "c"]
