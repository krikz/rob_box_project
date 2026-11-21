"""Защёлка заключительного этапа (#4694/#4996).

Gate 2 Depot 21.11: чистый ``approach_at_goal`` переключал режим на границе
``APPROACH_TARGET_M`` = 0.10 м. Разворот на месте двигает ``base_footprint`` по
окружности ``ROTATION_SWEEP_M`` = 0.088 м, dist каждый такт перескакивал границу,
и знак ``wz`` перекидывался (passA_yaw.log 283–322: w=-0.60 / w=+0.60). Потолок
25 с сработал 7 раз в проходе A и 12 в проходе B.

Тест гоняет замкнутую кинематику заметания. На коде без защёлки он падает:
знак ``wz`` меняется каждый такт, и за 8 с модельного времени поза не сходится.
"""

import math

from rob_box_teleop import yaw_at_goal as yag

# Допуски боевого checker'а (nav2_params.yaml, решение 20.11: не трогать).
_XY = 0.25
_YAW = 0.30
_MAX_V = 0.15
_MAX_W = 0.6
_DT = 0.2  # период тика ноды при check_rate_hz=5 не важен: 0.2 с — как в логе 21.11
_HORIZON_S = 8.0
_GOAL = (0.0, 0.0, 0.0)


def _worst_pose():
    """Худший старт: dist ≈ 0.099 м (уже внутри 0.10), dyaw ≈ 2.6 рад."""
    return (0.099, 0.0, 2.6)


def _sweep(pose, omega, dt):
    """Разворот на месте: base_footprint идёт по окружности радиуса ROTATION_SWEEP_M.

    Центр окружности — справа от носа на радиус заметания (измерено 21.11: yaw +0.2°,
    dist вырос 0.25 → 0.396 при vx=0). Знак omega двигает позу в разные стороны,
    поэтому перекидывание знака и есть ливлок.
    """
    radius = yag.ROTATION_SWEEP_M
    x, y, yaw = pose
    # Центр справа от курса: (x, y) + R * (sin yaw, -cos yaw) при правом повороте
    # уменьшает dist, когда нос смотрит почти на точку. Берём знак, совпадающий с
    # замером: положительный omega (против часовой) уводит точку влево от носа.
    cx = x - radius * math.sin(yaw)
    cy = y + radius * math.cos(yaw)
    yaw2 = yaw + omega * dt
    x2 = cx + radius * math.sin(yaw2)
    y2 = cy - radius * math.cos(yaw2)
    return (x2, y2, yaw2)


def _drive(pose, vx, omega, dt):
    """Ход вперёд по текущему курсу, затем заметание от курса. vx=0 — чистое заметание.

    Порядок как у базы: за dt робот проехал vx вдоль носа, и тот же dt корпус
    описал дугу радиуса ``ROTATION_SWEEP_M``. Иначе ход считается уже после
    поворота и модель врёт о знаке.
    """
    x, y, yaw = pose
    if vx != 0.0:
        x += vx * dt * math.cos(yaw)
        y += vx * dt * math.sin(yaw)
    return _sweep((x, y, yaw), omega, dt)


def _run(stage, pose, goal=_GOAL, horizon_s=_HORIZON_S, dt=_DT):
    """Замкнутый цикл. Возвращает (финал, знаки wz, причины, тики, done)."""
    signs = []
    reasons = []
    ticks = 0
    done = False
    t = 0.0
    while t < horizon_s + 1e-9:
        step = stage.step(pose, goal, _XY, _YAW, _MAX_V, _MAX_W)
        ticks += 1
        if step is None or step.done:
            done = step is not None and step.done
            break
        if step.omega != 0.0:
            signs.append(math.copysign(1.0, step.omega))
        reasons.append(step.reason)
        pose = _drive(pose, step.vx, step.omega, dt)
        t += dt
    return pose, signs, reasons, ticks, done, t


def _sign_flips(signs):
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def _stateless_signs(pose, ticks=6):
    """Знаки wz чистой функции без защёлки — то, что ливлочило 21.11."""
    signs = []
    for _ in range(ticks):
        step = yag.approach_at_goal(pose, _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
        assert step is not None and not step.done and step.omega != 0.0
        signs.append(math.copysign(1.0, step.omega))
        pose = _sweep(pose, step.omega, _DT)
    return signs


def test_stateless_boundary_flips_wz_sign_every_tick():
    """Доказательство ливлока на чистой функции: dist скачет через 0.10, знак wz меняется."""
    signs = _stateless_signs(_worst_pose())
    assert _sign_flips(signs) >= len(signs) - 1


def test_livelock_at_the_approach_boundary_converges_without_wz_sign_flips():
    """dist=0.099, dyaw=2.6: без защёлки знак wz перекидывается каждый такт.

    После правки: сходимость ≤ 8 с, финал dist ≤ 0.10 + заметание (0.188 м),
    |dyaw| внутри допуска checker'а с запасом, знак wz в этапе не меняется.
    """
    stage = yag.ApproachStage()
    pose, signs, reasons, ticks, done, elapsed = _run(stage, _worst_pose())
    flips = _sign_flips(signs)
    assert (
        done
    ), f"не сошлось за {elapsed:.1f} с, тиков {ticks}, последний reason={reasons[-1:]}"
    assert elapsed <= _HORIZON_S
    # Худший вход 0.099 + заметание 0.088 = 0.187, ниже checker 0.25 и планки 0.32.
    # Строже 0.10 здесь нельзя: доворот на курсе цели сам ходит по окружности.
    worst = yag.APPROACH_TARGET_M + yag.ROTATION_SWEEP_M
    assert yag.pose_distance(pose, _GOAL) <= worst + 1e-6
    # Порог done — yaw_tol минус запас, не сам запас: 0.30 − 0.05 = 0.25 рад.
    assert (
        abs(yag.shortest_yaw(pose[2], _GOAL[2]))
        <= _YAW - yag.FINISH_YAW_MARGIN_RAD + 1e-6
    )
    assert flips == 0, f"знак wz сменился {flips} раз за {ticks} тиков: {reasons[:8]}"
    assert reasons[0] == "доворот к курсу цели"
    assert set(reasons) == {"доворот к курсу цели"}


def test_new_goal_clears_the_finish_latch():
    stage = yag.ApproachStage()
    pose = _worst_pose()
    stage.step(pose, _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
    assert stage.finishing
    other = (1.0, 0.0, 0.0)
    far = (1.0 + 0.20, 0.0, 1.5)
    step = stage.step(far, other, _XY, _YAW, _MAX_V, _MAX_W)
    assert not stage.finishing
    assert step is not None and step.reason == "доворот на точку"


def test_escape_clears_the_finish_latch():
    stage = yag.ApproachStage()
    stage.step(_worst_pose(), _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
    assert stage.finishing
    escaped = (_GOAL[0] + yag.ESCAPE_RADIUS_M + 0.05, _GOAL[1], 0.0)
    stage.step(escaped, _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
    assert not stage.finishing


def test_time_cap_clears_the_finish_latch():
    stage = yag.ApproachStage()
    stage.step(_worst_pose(), _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
    assert stage.finishing
    stage.forget()
    assert not stage.finishing


def test_done_clears_the_finish_latch():
    stage = yag.ApproachStage()
    inside = (_GOAL[0] + 0.05, _GOAL[1], math.radians(2.0))
    # Сначала войти в этап (dist > запаса курса, но ≤ 0.10), потом встать в допуск.
    entering = (_GOAL[0] + 0.05, _GOAL[1], 1.5)
    stage.step(entering, _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
    assert stage.finishing
    step = stage.step(inside, _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
    assert step is not None and step.done
    assert not stage.finishing


def test_same_goal_does_not_clear_the_latch_when_sweep_crosses_the_boundary():
    """Заметание выносит dist за 0.10, но не за радиус побега — защёлка держит этап."""
    stage = yag.ApproachStage()
    pose = _worst_pose()
    stage.step(pose, _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
    assert stage.finishing
    thrown = _sweep(pose, -_MAX_W, _DT)
    assert yag.pose_distance(thrown, _GOAL) > yag.APPROACH_TARGET_M
    assert yag.pose_distance(thrown, _GOAL) < yag.ESCAPE_RADIUS_M
    step = stage.step(thrown, _GOAL, _XY, _YAW, _MAX_V, _MAX_W)
    assert stage.finishing
    assert step.reason == "доворот к курсу цели"
    assert step.omega < 0.0  # курс цели 0, поза ~2.6: короткий поворот отрицательный


def test_release_publishes_exactly_one_zero(node_params):
    """Контракт ноды: ровно один нулевой Twist на снятии пола, дальше тишина."""
    from rob_box_core.follow_path_halt import YAW_AT_GOAL_CMD_VEL_TOPIC as topic
    from rob_box_teleop.followpath_watchdog_node import FollowPathWatchdogNode

    node_params["nav2_params_file"] = str(
        __import__("pathlib").Path(__file__).resolve().parents[3]
        / "docker"
        / "main"
        / "config"
        / "nav2"
        / "nav2_params.yaml"
    )
    node = FollowPathWatchdogNode()
    yaw = node._yaw
    pub = {t: p for t, p in node.publishers}[topic]
    yaw._floor = True
    yaw._release()
    yaw._release()
    zeros = [m for m in pub.messages if m.linear.x == 0.0 and m.angular.z == 0.0]
    assert zeros == pub.messages
    assert len(zeros) == 1
    assert yaw._floor is False
