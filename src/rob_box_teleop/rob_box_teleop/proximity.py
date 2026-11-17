"""proximity — зона безопасности вокруг корпуса по лидару (#4365). Чистый модуль, без ROS.

Решение Шифу 04.11: предупреждать **только в движении**. Умолчания координатора:

* «внимание» — препятствие ближе ``warn_m`` = 0.40 м от КОРПУСА: маячок «дорожные работы» и мерный бипер
  заднего хода, без голоса;
* «стоп» — ближе ``stop_m`` = 0.20 м в коридоре хода (полоса корпуса ± 5 см впереди/сзади по ходу), ближе
  ``side_stop_m`` = 0.08 м для косяков и бортов сбоку, ``spin_stop_m`` = 0.10 м при развороте на месте
  (HIL 05.11, #4365): нулевой Twist в ``cmd_vel_emergency`` (twist_mux priority 255), маячок,
  частый бипер и голосом «Осторожно!» — одним событием тревоги, с одним началом и одним концом (решение Шифу 04.11).

Расстояние считается от прямоугольника корпуса (footprint из ``nav2_params.yaml``), а не от центра.
Угрозой считается только точка, к которой корпус СЕЙЧАС приближается (скорость ближайшей точки корпуса
направлена на препятствие): робот, который стоит у стены, едет вдоль неё в проёме или отъезжает назад,
не тревожит. Точки внутри корпуса (колёса, сам корпус, отражения) отбрасываются. Стоп держится, пока команда,
идущая в twist_mux, толкает корпус в то же препятствие (``assess``).

Здесь вся логика решения: геометрия, классификация зоны, гистерезис, расписание мигалки, писка и голоса.
Нода (``proximity_guard_node``) только подаёт сюда скан и скорость и публикует то, что решено.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

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
                range_max: float, mount: Mount) -> np.ndarray:
    """Точки ``LaserScan`` в ``base_footprint`` массивом ``N×2``; бесконечные, ближе ``range_min`` и дальше
    ``range_max`` — мимо. Векторно (numpy): нода считает скан 10 раз в секунду на Main Pi (CPU, HIL 05.11)."""
    r = np.asarray(ranges, dtype=float).reshape(-1)
    a = angle_min + np.arange(r.size) * angle_increment
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, a = r[ok], a[ok]
    lx, ly = r * np.cos(a), r * np.sin(a)
    cos_m, sin_m = math.cos(mount.yaw), math.sin(mount.yaw)
    return np.column_stack((mount.x + cos_m * lx - sin_m * ly, mount.y + sin_m * lx + cos_m * ly))


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
    """Пороги зоны. Умолчания — из issue #4365 и HIL 05.11 (Шифу может поменять параметрами ноды).

    Стоп разный по тому, ГДЕ точка (HIL 05.11: 81 % стопов в проёмах давал косяк впереди-сбоку при прямом ходе):

    * ``stop_m`` — точка в коридоре хода: полоса шириной корпуса ± ``corridor_margin_m`` впереди при v > 0,
      сзади при v < 0 — туда корпус въедет;
    * ``side_stop_m`` — прочие точки при ходе (косяк сбоку, угол, борт): корпус проходит мимо, тормозим только
      у самого касания;
    * ``spin_stop_m`` — разворот на месте (|v| ≤ ``moving_v``, |w| > ``moving_w``): углы корпуса заметают круг.

    «Внимание» (``warn_m``) — по всем точкам, к которым едет корпус, без коридора: это только маячок и бипер,
    движению не мешает, а у косяка сбоку оператору как раз нужно слышать, что тесно.
    """

    warn_m: float = 0.40
    stop_m: float = 0.20
    side_stop_m: float = 0.08
    spin_stop_m: float = 0.10
    corridor_margin_m: float = 0.05
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
        for name in ("side_stop_m", "spin_stop_m"):
            if not 0.0 < getattr(self, name) <= self.stop_m:
                raise ValueError(f"нужно 0 < {name} ≤ stop_m, получено {name}={getattr(self, name)}, "
                                 f"stop_m={self.stop_m}")
        if self.corridor_margin_m < 0.0:
            raise ValueError(f"corridor_margin_m ≥ 0, получено {self.corridor_margin_m}")
        if self.min_points < 1:
            raise ValueError(f"min_points ≥ 1, получено {self.min_points}")


def is_moving(vx: float, wz: float, th: Thresholds) -> bool:
    """Робот едет или крутится."""
    return abs(vx) > th.moving_v or abs(wz) > th.moving_w


def stop_limits(body: Body, points: np.ndarray, vx: float, th: Thresholds) -> np.ndarray:
    """Порог стопа для каждой точки, м: коридор хода — ``stop_m``, прочие — ``side_stop_m``;
    разворот на месте (|v| ≤ ``moving_v``) — ``spin_stop_m`` для всех."""
    if abs(vx) <= th.moving_v:
        return np.full(len(points), th.spin_stop_m)
    x, y = points[:, 0], points[:, 1]
    band = (y >= body.y_min - th.corridor_margin_m) & (y <= body.y_max + th.corridor_margin_m)
    ahead = x > body.x_max if vx > 0.0 else x < body.x_min
    return np.where(band & ahead, th.stop_m, th.side_stop_m)


@dataclass(frozen=True)
class Threat:
    """Что видно по ходу корпуса.

    ``gap``/``point`` — ближайшая точка, к которой едет корпус (для «внимания»).
    ``slack``/``stop_point``/``stop_gap`` — точка, ближе всех подошедшая к СВОЕМУ порогу стопа:
    ``slack = gap − порог`` (< 0 — стоп). ``inf`` и ``None`` — угрозы нет.
    """

    gap: float = math.inf
    point: Optional[Point] = None
    slack: float = math.inf
    stop_point: Optional[Point] = None
    stop_gap: float = math.inf

    def worse(self, other: "Threat") -> "Threat":
        """Худшее из двух: ближайшая точка «внимания» и наименьший запас до стопа — каждое от своего."""
        near = self if self.gap <= other.gap else other
        tight = self if self.slack <= other.slack else other
        return Threat(near.gap, near.point, tight.slack, tight.stop_point, tight.stop_gap)


NO_THREAT = Threat()


def _as_points(points) -> np.ndarray:
    return np.asarray(points, dtype=float).reshape(-1, 2)


def _kth(values: np.ndarray, k: int) -> int:
    """Индекс ``k``-го (с нуля) по возрастанию значения."""
    return int(np.argpartition(values, k)[k]) if k else int(np.argmin(values))


def nearest_threat(body: Body, points, vx: float, wz: float, th: Thresholds) -> Threat:
    """Угроза по точкам, к которым корпус едет со скоростью ``(vx, wz)``; ``NO_THREAT`` — таких меньше ``min_points``.

    Для «внимания» — ``min_points``-я по близости к корпусу точка, для стопа — ``min_points``-я по запасу до своего
    порога (``stop_limits``). Стоит — угрозы нет (решение Шифу: только в движении). Точки внутри корпуса
    не считаются.
    """
    if not is_moving(vx, wz, th):
        return NO_THREAT
    p = _as_points(points)
    if len(p) < th.min_points:
        return NO_THREAT
    qx = np.clip(p[:, 0], body.x_min, body.x_max)
    qy = np.clip(p[:, 1], body.y_min, body.y_max)
    dx, dy = p[:, 0] - qx, p[:, 1] - qy
    gap = np.hypot(dx, dy)
    outside = gap > 0.0
    with np.errstate(divide="ignore", invalid="ignore"):
        closing = ((vx - wz * qy) * dx + wz * qx * dy) / gap
    keep = outside & (closing > th.closing_mps)
    if int(keep.sum()) < th.min_points:
        return NO_THREAT
    p, gap = p[keep], gap[keep]
    slack = gap - stop_limits(body, p, vx, th)
    k = th.min_points - 1
    i_near, i_tight = _kth(gap, k), _kth(slack, k)
    return Threat(float(gap[i_near]), (float(p[i_near, 0]), float(p[i_near, 1])),
                  float(slack[i_tight]), (float(p[i_tight, 0]), float(p[i_tight, 1])), float(gap[i_tight]))


def assess(body: Body, points, vel: Tuple[float, float], cmd: Optional[Tuple[float, float]], holding: bool,
           th: Thresholds) -> Threat:
    """Угроза на такт: по скорости из одометрии, а пока держится стоп — ещё и по команде, что идёт в twist_mux.

    Стоящий в стопе робот по одометрии никуда не едет, и без команды стоп отпускался через ``stop_hold_s``:
    робот трогался на 0.1 с и снова вставал (HIL 05.11, толчок в стену: период 1.65 с). Пока команда толкает
    корпус в препятствие ближе порога, стоп держится; команда «назад», в сторону или ноль — отпускает
    (решение Шифу 05.11 «держать стоп, пока команда толкает»).
    """
    threat = nearest_threat(body, points, vel[0], vel[1], th)
    if holding and cmd is not None:
        threat = threat.worse(nearest_threat(body, points, cmd[0], cmd[1], th))
    return threat


def classify(threat: Threat, th: Thresholds, margin: float = 0.0) -> str:
    """Зона по угрозе; ``margin`` раздвигает пороги (отпускание с гистерезисом)."""
    if threat.slack < margin:
        return ZONE_STOP
    if threat.gap < th.warn_m + margin:
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

    def update(self, now: float, threat: Threat) -> Optional[Transition]:
        """Новый замер; вернуть ``Transition``, если зона сменилась."""
        raw = classify(threat, self.th)
        if severity(raw) > severity(self.zone):
            return self._enter(now, raw, threat)
        relaxed = classify(threat, self.th, self.th.release_margin_m)
        if severity(relaxed) >= severity(self.zone):
            self._calm_since = None
            return None
        if self._calm_since is None:
            self._calm_since = now
        held = self.zone != ZONE_STOP or now - self.entered_at >= self.th.stop_hold_s
        if held and now - self._calm_since >= self.th.release_s:
            return self._enter(now, relaxed, threat)
        return None

    @property
    def stopping(self) -> bool:
        """Держать ли сейчас аварийный стоп."""
        return self.zone == ZONE_STOP

    def _enter(self, now: float, zone: str, threat: Threat) -> Transition:
        tr = Transition(self.zone, zone, threat.stop_gap if zone == ZONE_STOP else threat.gap)
        self.zone, self.entered_at, self._calm_since = zone, now, None
        return tr


# -- команда в twist_mux: что сейчас велят корпусу ------------------------------------------------


@dataclass(frozen=True)
class MuxSource:
    """Вход twist_mux: топик, таймаут свежести, с, приоритет."""

    topic: str
    timeout: float
    priority: int


def mux_sources(doc: Mapping, skip_topic: str) -> List[MuxSource]:
    """Входы twist_mux из его ``twist_mux.yaml`` (единственный источник приоритетов), кроме ``skip_topic`` —
    аварийного входа, куда пишет сама нода (петли нет)."""
    topics = doc["twist_mux"]["ros__parameters"]["topics"]
    return [MuxSource(str(t["topic"]), float(t["timeout"]), int(t["priority"]))
            for t in topics.values() if str(t["topic"]) != skip_topic]


@dataclass
class CommandMux:
    """Какая команда сейчас прошла бы через twist_mux: самый приоритетный СВЕЖИЙ вход (как сам mux).

    Замки (``/joystick_lock``, ``/teleop_lock``) не учитываются: при замке нода может посчитать командой то,
    что mux заглушил, — это лишь продлит стоп, пока заглушённый источник толкает в препятствие, а робот
    в это время и так стоит.
    """

    sources: Sequence[MuxSource]
    _last: dict = field(default_factory=dict)

    def update(self, topic: str, now: float, v: float, w: float) -> None:
        """Пришла команда на вход ``topic``."""
        self._last[topic] = (now, v, w)

    def current(self, now: float) -> Optional[Tuple[float, float]]:
        """``(v, w)`` самого приоритетного свежего входа; ``None`` — свежих нет."""
        best: Optional[Tuple[int, float, float]] = None
        for src in self.sources:
            last = self._last.get(src.topic)
            if last is None or (src.timeout > 0.0 and now - last[0] > src.timeout):
                continue
            if best is None or src.priority > best[0]:
                best = (src.priority, last[1], last[2])
        return None if best is None else (best[1], best[2])


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


#: Переменная окружения, по которой контейнер знает, что едет на симе (HIL): ``docker/main/compose.hil.yaml``,
#: ``docker/common/scripts/use_sim_time.sh``. Один источник признака HIL для защиты.
SIM_TIME_ENV = "USE_SIM_TIME"

#: ``HIL_GUARD_SOUND=off`` (hil.conf -> compose.hil.yaml) глушит звук и голос защиты в HIL; по умолчанию они звучат.
GUARD_SOUND_ENV = "HIL_GUARD_SOUND"

#: Эффекты, которые при выключенном звуке HIL не уходят на настоящего робота: звук и голос слышны в мастерской (#4470).
#: Мигалка (``lights``) остаётся — она на лице, а не в воздухе.
HIL_MUTED_KINDS = frozenset({EFFECT_SOUND, EFFECT_SPEECH})


def hil_mute_sound(env: Mapping[str, str]) -> bool:
    """Глушить звук и голос: только в HIL и только при ``HIL_GUARD_SOUND=off``. Боевой режим не глушится никогда."""
    return hil_active(env) and env.get(GUARD_SOUND_ENV, "on").strip().lower() == "off"


def hil_active(env: Mapping[str, str]) -> bool:
    """HIL включён, если ``USE_SIM_TIME=true`` (допустимы только true/false, как в use_sim_time.sh)."""
    return env.get(SIM_TIME_ENV, "false").strip().lower() == "true"


def split_hil_effects(effects: Sequence[Effect], mute: bool) -> Tuple[List[Effect], List[Effect]]:
    """(что публиковать, что заглушено). Без ``mute`` заглушённых нет. Стоп сюда не попадает — он не ``Effect``."""
    if not mute:
        return list(effects), []
    kept = [e for e in effects if e.kind not in HIL_MUTED_KINDS]
    return kept, [e for e in effects if e.kind in HIL_MUTED_KINDS]


def describe(tr: Transition) -> str:
    """Строка журнала о смене зоны (её строит код, не LLM)."""
    gap = "—" if math.isinf(tr.gap) else f"{tr.gap:.2f} м"
    return f"зона {tr.previous} → {tr.zone} (до корпуса {gap})"


__all__ = [
    "ALARMS", "ALARM_LIGHTS", "Alarm", "AlarmScheduler", "Body", "CommandMux", "EFFECT_LIGHTS", "EFFECT_SOUND",
    "EFFECT_SPEECH", "Effect", "GUARD_SOUND_ENV", "HIL_MUTED_KINDS", "LIGHTS_EVERY_S", "LIGHTS_TTL_MS", "MuxSource",
    "PHRASE_COOLDOWN_S", "Mount", "SIM_TIME_ENV",
    "NAV2_FOOTPRINT_PATH", "NO_THREAT", "STOP_BEEP", "STOP_BEEP_EVERY_S", "Threat", "Thresholds", "Transition",
    "WARN_BEEP", "WARN_BEEP_EVERY_S", "ZONES", "ZONE_CLEAR", "ZONE_STOP", "ZONE_WARN", "ZoneTracker", "assess",
    "body_from_nav2_params", "classify", "closing_speed", "describe", "hil_active", "hil_mute_sound",
    "is_moving", "mux_sources", "nearest_threat",
    "parse_footprint", "scan_points", "severity", "split_hil_effects", "stop_limits", "yaw_from_quaternion",
]
