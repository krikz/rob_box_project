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
