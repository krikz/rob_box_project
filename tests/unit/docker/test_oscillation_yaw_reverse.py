"""Oscillation блокирует реверс курса на стоянке у цели (#4682).

Механизм — nav2_dwb_critics/src/oscillation.cpp, тег 1.1.20
(sha256 0cc39942d7b18e35610a6cccc4c0dc68f93baae88ffa9e146127f553c3249dae).
Смена знака wz взводит флаг, и противоположный знак объявляется осцилляцией,
пока корпус не уедет на oscillation_reset_dist (0.05 м) или не повернётся на
oscillation_reset_angle (0.2 рад). x_only_threshold (0.05) отслеживает wz
только при |vx| <= 0.05, то есть как раз на стоянке.

Повтор M1 (m1repeat_20261118T222220Z): на стоянке 22:27:38..22:28:09 команда
прыгала wz −0.6 ↔ +0.3 при vx=0, XY не двигался. Воспроизведение тех же правил
по /cmd_vel дало 259 тактов, где wz>0 был бы отвергнут, первый — 22:27:37.424.
В /evaluation тех тактов положительная полуось wz оборвана на Oscillation.

Здесь не исходник Nav2, а точная копия его правил: исходника в репо нет,
пакетный .so патчем не проверяется. Тест фиксирует, какое правило режет реверс
и какое минимальное изменение его отпускает, не отключая защиту.
"""

from __future__ import annotations

import math

RESET_DIST = 0.05
RESET_ANGLE = 0.2
X_ONLY = 0.05


class Trend:
    def __init__(self):
        self.sign = 0
        self.positive_only = False
        self.negative_only = False

    def reset(self):
        self.sign = 0
        self.positive_only = False
        self.negative_only = False

    def update(self, velocity):
        flipped = False
        if velocity < 0.0:
            if self.sign > 0:
                self.negative_only = True
                flipped = True
            self.sign = -1
        elif velocity > 0.0:
            if self.sign < 0:
                self.positive_only = True
                flipped = True
            self.sign = 1
        return flipped

    def oscillating(self, velocity):
        return (self.positive_only and velocity < 0.0) or (self.negative_only and velocity > 0.0)


class Oscillation:
    """Копия OscillationCritic::debrief/scoreTrajectory, без ROS."""

    def __init__(self, reset_angle=RESET_ANGLE, x_only=X_ONLY):
        self.reset_angle = reset_angle
        self.x_only = x_only
        self.theta = Trend()
        self.prev = None

    def debrief(self, vx, wz, pose):
        # Порядок из oscillation.cpp: resetAvailable() смотрит флаги и позу
        # прошлого такта, и только потом setOscillationFlags пишет текущую
        # команду. Иначе поворот, накопленный до команды, флаг не снимает.
        if self.prev and (self.theta.positive_only or self.theta.negative_only):
            dist = math.hypot(pose[0] - self.prev[0], pose[1] - self.prev[1])
            if dist > RESET_DIST or abs(pose[2] - self.prev[2]) > self.reset_angle:
                self.theta.reset()
        if self.x_only < 0.0 or abs(vx) <= self.x_only:
            if self.theta.update(wz):
                self.prev = pose

    def rejects(self, wz):
        return self.theta.oscillating(wz)


def _drive(critic, commands):
    """commands: (vx, wz, pose).

    Порядок как в контроллере: scoreTrajectory смотрит флаг, оставленный
    прошлым debrief, и только потом debrief записывает текущую команду.
    """
    rejected = 0
    for vx, wz, pose in commands:
        critic.debrief(vx, wz, pose)
        if wz > 0.0 and critic.rejects(wz):
            rejected += 1
    return rejected


# Стоянка: корпус на месте, курс за каждый такт сдвигается меньше порога
# 0.2 рад. Команда крутит в минус, потом нужен реверс в плюс. Так было
# 22:27:38..22:28:09: wz прыгал −0.6 ↔ +0.3, XY не двигался.
STALL = [
    (0.0, -0.6, (0.0, 0.0, 0.00)),
    (0.0, 0.3, (0.0, 0.0, -0.04)),
    (0.0, -0.6, (0.0, 0.0, -0.02)),
    (0.0, 0.3, (0.0, 0.0, -0.06)),
]


def test_current_rules_reject_the_yaw_reverse_while_standing():
    # Курс сдвинулся максимум на 0.06 рад — меньше 0.2, XY не двигался.
    # Флаг взводится на возврате к минусу и держится: второй реверс в плюс
    # отвергнут. Первый реверс флаг ещё не застал — его режет следующий такт.
    rejected = _drive(Oscillation(), STALL)
    assert rejected == 1


def test_reset_angle_below_the_stall_swing_lets_the_reverse_through():
    # Корпус между сменой знака и следующим реверсом повернулся на 0.04 рад.
    # Порог 0.03 это видит и снимает флаг до проверки реверса.
    rejected = _drive(Oscillation(reset_angle=0.03), STALL)
    assert rejected == 0

    # Защита жива и при новом пороге, если корпус не повернулся и не проехал:
    # флаг не снимается, возврат к прежнему знаку режется.
    stuck = [
        (0.0, -0.6, (0.0, 0.0, 0.0)),
        (0.0, 0.3, (0.0, 0.0, 0.0)),
        (0.0, -0.6, (0.0, 0.0, 0.0)),
        (0.0, 0.3, (0.0, 0.0, 0.0)),
    ]
    assert _drive(Oscillation(reset_angle=0.03), stuck) == 1


def test_x_only_threshold_does_not_help_while_standing():
    # На стоянке |vx|=0 <= любого положительного порога, wz отслеживается.
    # Поднять порог — не лечит. Лечит только порог < 0, но он отключит
    # отслеживание wz всегда, и на ходу тоже.
    rejected = _drive(Oscillation(x_only=0.0), STALL)
    assert rejected == 1
