"""Сторож сирот FollowPath (#4458): чистая функция ``followpath_orphan.decide`` — без ROS."""

from rob_box_teleop import followpath_orphan as fo

TIMEOUT = 1.5
NAV = "aa"
FP = "bb"
POSE = "navigate_to_pose"
THROUGH = "navigate_through_poses"


def run(steps, timeout=TIMEOUT):
    """steps: [(t, parent_goals, follow_goals)] -> список (t, cancel)."""
    state, out = fo.WatchState(), []
    for t, parent_goals, follow_goals in steps:
        d = fo.decide(state, t, parent_goals, follow_goals, timeout)
        state = d.state
        out.append((t, d.cancel, d.parent_source))
    return out


def parents(*items):
    """``(источник, статус, goal_id, stamp)`` → группы для ``decide``."""
    grouped = {}
    for source, status, gid, stamp in items:
        grouped.setdefault(source, []).append((gid, stamp, status))
    return tuple(grouped.items())


def one(source, status, gid=NAV, stamp=1.0):
    return parents((source, status, gid, stamp))


def fp(status, gid=FP):
    return [(gid, 2.0, status)]


def test_orphan_after_aborted_is_cancelled_after_timeout():
    res = run(
        [
            (10.0, one(POSE, fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)),
            (11.0, one(POSE, fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)),
            (11.6, one(POSE, fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)),
        ]
    )
    assert [(t, c) for t, c, _ in res] == [(10.0, ()), (11.0, ()), (11.6, (FP,))]
    assert res[-1][2] == POSE


def test_same_rule_for_voice_quest_and_hil_sources():
    """Голос, квест и HIL шлют NavigateToPose; полоса HIL — NavigateThroughPoses. Правило одно."""
    for source in (POSE, THROUGH):
        goals = one(source, fo.STATUS_CANCELED)
        res = run(
            [
                (0.0, goals, fp(fo.STATUS_EXECUTING)),
                (1.5, goals, fp(fo.STATUS_EXECUTING)),
            ]
        )
        assert res[-1][1] == (FP,) and res[-1][2] == source


def test_orphan_after_canceled_and_succeeded_is_cancelled_too():
    for status in (fo.STATUS_CANCELED, fo.STATUS_SUCCEEDED):
        res = run(
            [
                (0.0, one(POSE, status), fp(fo.STATUS_ACCEPTED)),
                (1.5, one(POSE, status), fp(fo.STATUS_ACCEPTED)),
            ]
        )
        assert res[-1][1] == (FP,), status


def test_follow_path_finished_by_itself_does_nothing():
    res = run(
        [
            (0.0, one(POSE, fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING)),
            (1.0, one(POSE, fo.STATUS_ABORTED), fp(fo.STATUS_SUCCEEDED)),
            (5.0, one(POSE, fo.STATUS_ABORTED), fp(fo.STATUS_SUCCEEDED)),
        ]
    )
    assert all(cancel == () for _, cancel, _ in res)


def test_parent_still_active_does_nothing():
    for source in (POSE, THROUGH):
        for status in (fo.STATUS_ACCEPTED, fo.STATUS_EXECUTING, fo.STATUS_CANCELING):
            goals = one(source, status)
            res = run(
                [
                    (0.0, goals, fp(fo.STATUS_EXECUTING)),
                    (60.0, goals, fp(fo.STATUS_EXECUTING)),
                ]
            )
            assert all(cancel == () for _, cancel, _ in res), (source, status)


def test_live_goal_on_another_action_holds_follow_path():
    """Отменённая NavigateToPose не гасит FollowPath, пока полоса NavigateThroughPoses ещё едет."""
    mixed = parents(
        (POSE, fo.STATUS_CANCELED, "pose", 1.0),
        (THROUGH, fo.STATUS_EXECUTING, "lane", 2.0),
    )
    res = run(
        [(0.0, mixed, fp(fo.STATUS_EXECUTING)), (30.0, mixed, fp(fo.STATUS_EXECUTING))]
    )
    assert all(cancel == () for _, cancel, _ in res)
    assert res[-1][2] == THROUGH


def test_repeat_for_the_same_goal_cancels_once():
    steps = [
        (t, one(POSE, fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING))
        for t in (0.0, 1.5, 1.6, 2.0, 9.0)
    ]
    cancels = [c for _, c, _ in run(steps)]
    assert cancels == [(), (FP,), (), (), ()]


def test_new_parent_goal_resets_the_timer():
    old = one(POSE, fo.STATUS_ABORTED)
    new = parents(
        (POSE, fo.STATUS_ABORTED, NAV, 1.0), (POSE, fo.STATUS_EXECUTING, "cc", 5.0)
    )
    res = run(
        [
            (0.0, old, fp(fo.STATUS_EXECUTING)),
            (1.0, new, fp(fo.STATUS_EXECUTING)),
            (1.2, old, fp(fo.STATUS_EXECUTING)),
            (2.0, old, fp(fo.STATUS_EXECUTING)),
        ]
    )
    assert [c for _, c, _ in res] == [
        (),
        (),
        (),
        (),
    ]  # таймер пошёл заново с 1.2, к 2.0 прошло 0.8 с


def test_no_parent_goals_at_all_leaves_foreign_follow_path_alone():
    res = run([(0.0, (), fp(fo.STATUS_EXECUTING)), (30.0, (), fp(fo.STATUS_EXECUTING))])
    assert all(cancel == () for _, cancel, _ in res)


def test_empty_status_lists_are_not_a_parent():
    res = run([(5.0, ((POSE, []), (THROUGH, [])), fp(fo.STATUS_EXECUTING))])
    assert res == [(5.0, (), "")]


def test_latest_parent_goal_by_stamp_decides_across_actions():
    """Последняя по stamp решает, но любая живая родительская цель держит FollowPath."""
    held = parents(
        (POSE, fo.STATUS_EXECUTING, "voice", 3.0),
        (THROUGH, fo.STATUS_ABORTED, "lane", 9.0),
    )
    d = fo.decide(fo.WatchState(), 30.0, held, fp(fo.STATUS_EXECUTING), TIMEOUT)
    assert d.cancel == () and d.parent_source == THROUGH and d.parent_goal == "lane"

    done = parents(
        (POSE, fo.STATUS_SUCCEEDED, "voice", 3.0),
        (THROUGH, fo.STATUS_ABORTED, "lane", 9.0),
    )
    d = fo.decide(fo.WatchState(), 0.0, done, fp(fo.STATUS_EXECUTING), TIMEOUT)
    assert (
        d.parent_source == THROUGH
        and d.parent_goal == "lane"
        and d.parent_status == fo.STATUS_ABORTED
    )
    d = fo.decide(d.state, 1.5, done, fp(fo.STATUS_EXECUTING), TIMEOUT)
    assert d.cancel == (FP,)


def test_cancelled_memory_is_dropped_when_goal_leaves_the_status_list():
    state = fo.decide(
        fo.WatchState(), 0.0, one(POSE, fo.STATUS_ABORTED), fp(fo.STATUS_EXECUTING), 0.0
    ).state
    assert state.cancelled == {FP}
    state = fo.decide(state, 1.0, one(POSE, fo.STATUS_ABORTED), [], TIMEOUT).state
    assert state.cancelled == frozenset()
