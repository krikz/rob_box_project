"""Сторож сирот FollowPath (#4458): чистая функция ``followpath_orphan.decide`` — без ROS."""

from rob_box_teleop import followpath_orphan as fo

TIMEOUT = 1.5
NAV = "aa"
FP = "bb"


def run(steps, timeout=TIMEOUT):
    """steps: [(t, nav_goals, follow_goals)] -> список (t, cancel)."""
    state, out = fo.WatchState(), []
    for t, nav_goals, follow_goals in steps:
        d = fo.decide(state, t, nav_goals, follow_goals, timeout)
        state = d.state
        out.append((t, d.cancel))
    return out


def nav(status, gid=NAV, stamp=1.0):
    return [(gid, stamp, status)]


def fp(status, gid=FP):
    return [(gid, 2.0, status)]


def test_orphan_after_aborted_is_cancelled_after_timeout():
    res = run(
        [
            (10.0, nav(fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)),
            (11.0, nav(fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)),
            (11.6, nav(fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)),
        ]
    )
    assert res == [(10.0, ()), (11.0, ()), (11.6, (FP,))]


def test_orphan_after_canceled_and_succeeded_is_cancelled_too():
    for status in (fo.STATUS_CANCELED, fo.STATUS_SUCCEEDED):
        res = run([(0.0, nav(status), fp(fo.STATUS_ACCEPTED)), (1.5, nav(status), fp(fo.STATUS_ACCEPTED))])
        assert res[-1] == (1.5, (FP,)), status


def test_follow_path_finished_by_itself_does_nothing():
    res = run(
        [
            (0.0, nav(fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)),
            (1.0, nav(fo.STATUS_ABORTED), fp(fo.STATUS_SUCCEEDED)),
            (5.0, nav(fo.STATUS_ABORTED), fp(fo.STATUS_SUCCEEDED)),
        ]
    )
    assert all(cancel == () for _, cancel in res)


def test_navigate_to_pose_still_active_does_nothing():
    for status in (fo.STATUS_ACCEPTED, fo.STATUS_EXECUTING, fo.STATUS_CANCELING):
        res = run([(0.0, nav(status), fp(fo.STATUS_EXECUTING)), (60.0, nav(status), fp(fo.STATUS_EXECUTING))])
        assert all(cancel == () for _, cancel in res), status


def test_repeat_for_the_same_goal_cancels_once():
    steps = [(t, nav(fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)) for t in (0.0, 1.5, 1.6, 2.0, 9.0)]
    cancels = [c for _, c in run(steps)]
    assert cancels == [(), (FP,), (), (), ()]


def test_new_navigation_goal_resets_the_timer():
    old = [(NAV, 1.0, fo.STATUS_ABORTED)]
    new = old + [("cc", 5.0, fo.STATUS_EXECUTING)]
    res = run(
        [
            (0.0, old, fp(fo.STATUS_EXECUTING)),
            (1.0, new, fp(fo.STATUS_EXECUTING)),
            (1.2, old, fp(fo.STATUS_EXECUTING)),
            (2.0, old, fp(fo.STATUS_EXECUTING)),
        ]
    )
    assert [c for _, c in res] == [(), (), (), ()]  # таймер пошёл заново с 1.2, к 2.0 прошло 0.8 с


def test_no_navigation_goals_at_all_leaves_foreign_follow_path_alone():
    res = run([(0.0, [], fp(fo.STATUS_EXECUTING)), (30.0, [], fp(fo.STATUS_EXECUTING))])
    assert all(cancel == () for _, cancel in res)


def test_latest_navigation_goal_by_stamp_decides():
    goals = [("old", 1.0, fo.STATUS_ABORTED), ("new", 9.0, fo.STATUS_ABORTED)]
    d = fo.decide(fo.WatchState(), 0.0, goals, fp(fo.STATUS_EXECUTING), TIMEOUT)
    assert d.nav_goal == "new" and d.nav_status == fo.STATUS_ABORTED


def test_cancelled_memory_is_dropped_when_goal_leaves_the_status_list():
    state = fo.decide(fo.WatchState(), 0.0, nav(fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING), 0.0).state
    assert state.cancelled == {FP}
    state = fo.decide(state, 1.0, nav(fo.STATUS_ABORTED), [], TIMEOUT).state
    assert state.cancelled == frozenset()
