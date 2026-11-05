"""proximity — зона безопасности вокруг корпуса по лидару (#4365). Чистый модуль, без ROS.

Решение Шифу 04.11: предупреждать **только в движении**. Умолчания координатора:

* «внимание» — препятствие ближе ``warn_m`` = 0.40 м от КОРПУСА: маячок «дорожные работы» и мерный бипер
  заднего хода, без голоса;
* «стоп» — ближе ``stop_m`` = 0.20 м: нулевой Twist в ``cmd_vel_emergency`` (twist_mux priority 255), маячок,
  частый бипер и голосом «Осторожно!» — одним событием тревоги, с одним началом и одним концом (решение Шифу 04.11).

Расстояние считается от прямоугольника корпуса (footprint из ``nav2_params.yaml``), а не от центра.
Угрозой считается только точка, к которой корпус СЕЙЧАС приближается (скорость ближайшей точки корпуса
направлена на препятствие): робот, который стоит у стены, едет вдоль неё в проёме или отъезжает назад,
не тревожит. Точки внутри корпуса (колёса, сам корпус, отражения) отбрасываются.

Здесь вся логика решения: геометрия, классификация зоны, гистерезис, расписание мигалки, писка и голоса.
Нода (``proximity_guard_node``) только подаёт сюда скан и скорость и публикует то, что решено.
"""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

Point = Tuple[float, float]

ZONE_CLEAR = "clear"
ZONE_WARN = "warn"
ZONE_STOP = "stop"
#: Порядок — по тяжести.
ZONES: Tuple[str, ...] = (ZONE_CLEAR, ZONE_WARN, ZONE_STOP)

#: Где лежит габарит корпуса — единственный источник: тот же прямоугольник, что у local_costmap Nav2.
NAV2_FOOTPRINT_PATH: Tuple[str, ...] = ("local_costmap", "local_costmap", "ros__parameters", "footprint")


def severity(zone: str) -> int:
    """Тяжесть зоны: ``clear`` 0 < ``warn`` 1 < ``stop`` 2."""
    return ZONES.index(zone)


# -- геометрия ------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Body:
    """Прямоугольник корпуса в ``base_footprint`` (оси x — вперёд, y — влево)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @classmethod
    def from_polygon(cls, points: Sequence[Sequence[float]]) -> "Body":
        """Охватывающий прямоугольник многоугольника footprint (у РОББОКСа он и есть прямоугольник)."""
        if len(points) < 3:
            raise ValueError(f"footprint: нужно ≥ 3 точек, получено {len(points)}")
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        return cls(min(xs), max(xs), min(ys), max(ys))

    def nearest(self, x: float, y: float) -> Point:
        """Ближайшая к ``(x, y)`` точка корпуса (для точки внутри — она сама)."""
        return min(max(x, self.x_min), self.x_max), min(max(y, self.y_min), self.y_max)

    def gap(self, x: float, y: float) -> float:
        """Расстояние от точки до корпуса, м; 0 — точка на корпусе или внутри."""
        qx, qy = self.nearest(x, y)
        return math.hypot(x - qx, y - qy)


def parse_footprint(text: str) -> List[Point]:
    """Строка footprint Nav2 ``"[[x, y], ...]"`` -> список точек."""
    raw = json.loads(text) if isinstance(text, str) else text
    return [(float(p[0]), float(p[1])) for p in raw]


def body_from_nav2_params(doc: Mapping) -> Body:
    """Корпус из разобранного ``nav2_params.yaml`` (footprint local_costmap)."""
    node = doc
    for key in NAV2_FOOTPRINT_PATH:
        if not isinstance(node, Mapping) or key not in node:
            raise KeyError(f"nav2_params: нет {'.'.join(NAV2_FOOTPRINT_PATH)}")
        node = node[key]
    return Body.from_polygon(parse_footprint(node))


@dataclass(frozen=True)
class Mount:
    """Положение лидара в ``base_footprint``: смещение и поворот вокруг z."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Рыскание кватерниона (лидар стоит горизонтально — крен и тангаж не нужны)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def scan_points(ranges: Iterable[float], angle_min: float, angle_increment: float, range_min: float,
                range_max: float, mount: Mount) -> List[Point]:
    """Точки ``LaserScan`` в ``base_footprint``; бесконечные, ближе ``range_min`` и дальше ``range_max`` — мимо."""
    cos_m, sin_m = math.cos(mount.yaw), math.sin(mount.yaw)
    points: List[Point] = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        points.append((mount.x + cos_m * lx - sin_m * ly, mount.y + sin_m * lx + cos_m * ly))
    return points


def closing_speed(body: Body, x: float, y: float, vx: float, wz: float) -> float:
    """Скорость, с которой корпус приближается к точке, м/с (> 0 — приближается).

    Берётся скорость ближайшей точки корпуса ``q`` (поступательная ``vx`` плюс вращение ``wz × q``)
    и проецируется на направление от ``q`` к препятствию.
    """
    qx, qy = body.nearest(x, y)
    dx, dy = x - qx, y - qy
    d = math.hypot(dx, dy)
    if d <= 0.0:
        return 0.0
    vqx, vqy = vx - wz * qy, wz * qx
    return (vqx * dx + vqy * dy) / d


# -- решение о зоне -----------------------------------------------------------------------------------


@dataclass(frozen=True)
class Thresholds:
    """Пороги зоны. Умолчания — из issue #4365 (Шифу может поменять параметрами ноды)."""

    warn_m: float = 0.40
    stop_m: float = 0.20
    #: Зона отпускается, только когда препятствие дальше порога на этот запас.
    release_margin_m: float = 0.05
    #: …и держится так ``release_s`` подряд.
    release_s: float = 0.5
    #: Сколько минимум держать стоп, с: робот должен успеть встать.
    stop_hold_s: float = 1.5
    #: «В движении»: |v| > moving_v (м/с) или |w| > moving_w (рад/с).
    moving_v: float = 0.03
    moving_w: float = 0.10
    #: Точка — угроза, если корпус приближается к ней быстрее этого, м/с.
    closing_mps: float = 0.02
    #: Угроза — не меньше стольких точек скана ближе порога: одиночный луч-призрак не тормозит
    #: (в симе луч 225 давал точку в 0.2 м от корпуса при пустом мире, прогон run5 04.11).
    min_points: int = 2

    def __post_init__(self) -> None:
        if not 0.0 < self.stop_m < self.warn_m:
            raise ValueError(f"нужно 0 < stop_m < warn_m, получено stop_m={self.stop_m}, warn_m={self.warn_m}")
        if self.min_points < 1:
            raise ValueError(f"min_points ≥ 1, получено {self.min_points}")


def is_moving(vx: float, wz: float, th: Thresholds) -> bool:
    """Робот едет или крутится."""
    return abs(vx) > th.moving_v or abs(wz) > th.moving_w


@dataclass(frozen=True)
class Threat:
    """Ближайшая точка, к которой едет корпус: расстояние до корпуса и сама точка (``None`` — угрозы нет)."""

    gap: float = math.inf
    point: Optional[Point] = None


NO_THREAT = Threat()


def nearest_threat(body: Body, points: Iterable[Point], vx: float, wz: float, th: Thresholds) -> Threat:
    """Угроза: ``min_points``-я по близости к корпусу точка из тех, к которым робот едет; ``NO_THREAT`` — таких меньше.

    Стоит — угрозы нет (решение Шифу: только в движении). Точки внутри корпуса не считаются.
    """
    if not is_moving(vx, wz, th):
        return NO_THREAT
    closing = ((body.gap(x, y), (x, y)) for x, y in points if closing_speed(body, x, y, vx, wz) > th.closing_mps)
    nearest = heapq.nsmallest(th.min_points, (c for c in closing if c[0] > 0.0))
    if len(nearest) < th.min_points:
        return NO_THREAT
    gap, point = nearest[-1]
    return Threat(gap, point)


def classify(gap: float, th: Thresholds, margin: float = 0.0) -> str:
    """Зона по расстоянию; ``margin`` раздвигает пороги (отпускание с гистерезисом)."""
    if gap < th.stop_m + margin:
        return ZONE_STOP
    if gap < th.warn_m + margin:
        return ZONE_WARN
    return ZONE_CLEAR


@dataclass(frozen=True)
class Transition:
    """Смена зоны: откуда, куда, расстояние в момент смены."""

    previous: str
    zone: str
    gap: float


@dataclass
class ZoneTracker:
    """Зона с гистерезисом: вход сразу, выход — после ``release_s`` с запасом, стоп — не раньше ``stop_hold_s``."""

    th: Thresholds = field(default_factory=Thresholds)
    zone: str = ZONE_CLEAR
    entered_at: float = 0.0
    _calm_since: Optional[float] = None

    def update(self, now: float, gap: float) -> Optional[Transition]:
        """Новый замер; вернуть ``Transition``, если зона сменилась."""
        raw = classify(gap, self.th)
        if severity(raw) > severity(self.zone):
            return self._enter(now, raw, gap)
        relaxed = classify(gap, self.th, self.th.release_margin_m)
        if severity(relaxed) >= severity(self.zone):
            self._calm_since = None
            return None
        if self._calm_since is None:
            self._calm_since = now
        held = self.zone != ZONE_STOP or now - self.entered_at >= self.th.stop_hold_s
        if held and now - self._calm_since >= self.th.release_s:
            return self._enter(now, relaxed, gap)
        return None

    @property
    def stopping(self) -> bool:
        """Держать ли сейчас аварийный стоп."""
        return self.zone == ZONE_STOP

    def _enter(self, now: float, zone: str, gap: float) -> Transition:
        tr = Transition(self.zone, zone, gap)
        self.zone, self.entered_at, self._calm_since = zone, now, None
        return tr


# -- тревога: мигалка, писк, голос -----------------------------------------------------------------


@dataclass(frozen=True)
class Alarm:
    """Что делает зона — одним событием тревоги (решение Шифу 04.11): все каналы начинаются на входе в зону
    и кончаются на выходе из неё.

    ``animation`` — сцена LED-лица (``rob_box_animations.face.states``), ``gesture`` в ``ExpressionRequest``;
    живёт ``LIGHTS_TTL_MS`` и продлевается каждые ``LIGHTS_EVERY_S``, пока зона держится, — гаснет не позже
    ``LIGHTS_TTL_MS`` после выхода.
    ``sound`` — триггер sound_pack (``/voice/sound/trigger``): на входе и каждые ``sound_every_s``, пока зона
    держится (``None`` — только на входе).
    ``phrase`` — фраза кода голосом робота (``SpeechRequest``, не LLM): на входе, не чаще ``PHRASE_COOLDOWN_S``;
    ``None`` — без голоса.
    """

    animation: str
    sound: str
    sound_every_s: Optional[float]
    phrase: Optional[str]


#: Мигалка тревоги — янтарный маячок «дорожные работы» (решение Шифу 05.11: не полицейская).
ALARM_LIGHTS = "roadworks_beacon"
#: Писк — бипер заднего хода грузовика (решение Шифу 05.11; ``sound_pack/make_backup_beep.py``): бип 0.45 с.
#: «Внимание» повторяет его каждые 0.9 с (бип 0.45 / пауза 0.45), «стоп» — короткий бип 0.2 с каждые 0.4 с.
WARN_BEEP, WARN_BEEP_EVERY_S = "robot_backup_beep", 0.9
STOP_BEEP, STOP_BEEP_EVERY_S = "robot_backup_beep_fast", 0.4
#: «Внимание» (< 0.40 м): маячок и мерный бипер, без голоса. «Стоп» (< 0.20 м): маячок, частый бипер, пока стоит
#: стоп, и «Осторожно!» на входе.
ALARMS: Mapping[str, Alarm] = {
    ZONE_WARN: Alarm(ALARM_LIGHTS, WARN_BEEP, WARN_BEEP_EVERY_S, None),
    ZONE_STOP: Alarm(ALARM_LIGHTS, STOP_BEEP, STOP_BEEP_EVERY_S, "Осторожно!"),
}
#: Время жизни мигалки на лице и шаг продления, пока зона держится.
LIGHTS_TTL_MS = 1000
LIGHTS_EVERY_S = 0.5
#: Голосовую фразу — не чаще раза в столько секунд (повторные стопы подряд не тараторят).
PHRASE_COOLDOWN_S = 10.0

#: Допуск сравнения времени тактов: ритм бипера не съезжает на такт из-за погрешности float (0.4 → 0.45 с).
TICK_EPS_S = 1e-6

EFFECT_LIGHTS = "lights"
EFFECT_SOUND = "sound"
EFFECT_SPEECH = "speech"


@dataclass(frozen=True)
class Effect:
    """Одно действие для ноды: ``kind`` — ``lights`` (сцена, ``ttl_ms``) | ``sound`` (триггер) | ``speech`` (фраза)."""

    kind: str
    name: str
    ttl_ms: int = 0


@dataclass
class AlarmScheduler:
    """Расписание тревоги: на входе в зону — все каналы зоны в одном такте; пока держится — продление мигалки и
    повтор писка; на выходе — ничего нового (мигалка гаснет по ``LIGHTS_TTL_MS``)."""

    alarms: Mapping[str, Alarm] = field(default_factory=lambda: ALARMS)
    _zone: str = ZONE_CLEAR
    _lights_at: float = -math.inf
    _sound_at: float = -math.inf
    _phrase_at: float = -math.inf

    def tick(self, now: float, zone: str) -> List[Effect]:
        """Действия на этот такт."""
        entered = zone != self._zone
        self._zone = zone
        alarm = self.alarms.get(zone)
        if alarm is None:
            return []
        effects: List[Effect] = []
        if entered or now - self._lights_at >= LIGHTS_EVERY_S - TICK_EPS_S:
            self._lights_at = now
            effects.append(Effect(EFFECT_LIGHTS, alarm.animation, LIGHTS_TTL_MS))
        repeat = alarm.sound_every_s is not None and now - self._sound_at >= alarm.sound_every_s - TICK_EPS_S
        if entered or repeat:
            self._sound_at = now
            effects.append(Effect(EFFECT_SOUND, alarm.sound))
        if entered and alarm.phrase and now - self._phrase_at >= PHRASE_COOLDOWN_S:
            self._phrase_at = now
            effects.append(Effect(EFFECT_SPEECH, alarm.phrase))
        return effects


def describe(tr: Transition) -> str:
    """Строка журнала о смене зоны (её строит код, не LLM)."""
    gap = "—" if math.isinf(tr.gap) else f"{tr.gap:.2f} м"
    return f"зона {tr.previous} → {tr.zone} (до корпуса {gap})"


__all__ = [
    "ALARMS", "ALARM_LIGHTS", "Alarm", "AlarmScheduler", "Body", "EFFECT_LIGHTS", "EFFECT_SOUND", "EFFECT_SPEECH",
    "Effect", "LIGHTS_EVERY_S", "LIGHTS_TTL_MS", "PHRASE_COOLDOWN_S", "Mount", "NAV2_FOOTPRINT_PATH", "NO_THREAT",
    "STOP_BEEP", "STOP_BEEP_EVERY_S", "Threat", "Thresholds", "Transition", "WARN_BEEP", "WARN_BEEP_EVERY_S", "ZONES",
    "ZONE_CLEAR", "ZONE_STOP", "ZONE_WARN", "ZoneTracker", "body_from_nav2_params", "classify", "closing_speed",
    "describe", "is_moving", "nearest_threat", "parse_footprint", "scan_points", "severity", "yaw_from_quaternion",
]
