"""DemandDrivenSubscriptions: камеры подписываются только пока их смотрит шлем."""

from types import SimpleNamespace

from rob_box_quest.streams.ros_demand import DemandDrivenSubscriptions


class _Node:
    def __init__(self):
        self.destroyed = []

    def destroy_subscription(self, sub):
        self.destroyed.append(sub)


def _make(demand, linger_s=5.0):
    now = [0.0]
    node = _Node()
    created = []

    def factory():
        sub = SimpleNamespace(n=len(created))
        created.append(sub)
        return sub

    subs = DemandDrivenSubscriptions(
        node, {"camera_rear": factory}, lambda ui: demand.get(ui, False), linger_s=linger_s, clock=lambda: now[0]
    )
    return subs, node, created, now


def test_no_subscription_without_viewers():
    subs, _, created, _ = _make({})
    subs.tick()
    assert created == []
    assert subs.active("camera_rear") is None


def test_subscribes_once_while_viewed():
    demand = {"camera_rear": True}
    subs, _, created, now = _make(demand)
    subs.tick()
    now[0] += 1
    subs.tick()
    assert len(created) == 1
    assert subs.active("camera_rear") is created[0]


def test_unsubscribes_after_linger():
    demand = {"camera_rear": True}
    subs, node, created, now = _make(demand, linger_s=5.0)
    subs.tick()
    demand["camera_rear"] = False
    now[0] += 3
    subs.tick()
    assert node.destroyed == []
    now[0] += 3
    subs.tick()
    assert node.destroyed == created
    assert subs.active("camera_rear") is None


def test_resubscribes_when_viewer_returns():
    demand = {"camera_rear": True}
    subs, _, created, now = _make(demand, linger_s=1.0)
    subs.tick()
    demand["camera_rear"] = False
    now[0] += 2
    subs.tick()
    demand["camera_rear"] = True
    subs.tick()
    assert len(created) == 2
