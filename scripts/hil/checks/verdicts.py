#!/usr/bin/env python3
"""Вердикты HIL-проверок группы А (``scripts/hil/checks``, ``docs/runbooks/hil-checks.md``): PASS/FAIL строит код.

Чистая логика без ROS. Вход — raw-файлы одного сценария (лог пробы ``probe.py`` с песочницы, ``docker logs
--timestamps`` контейнеров робота), выход — строка ``VERDICT <карточка> <сценарий> <статус>: <факты>``.

Статусы:
  PASS          ожидание выполнено, факты в строке;
  FAIL          нарушение, факты в строке;
  NOT_TRIGGERED проверяемый случай не наступил (ABORTED не случился, тул не вызвался) — это НЕ PASS;
  NO_DATA       нет raw (файл пуст или проба не стартовала).

Знания не дублируются: фразы робота — из ``phrases/ru.yaml`` диалога, имена писка, маячка и фраза защиты — из
``rob_box_teleop/proximity.py`` (``ALARMS``), формат строки зоны — ``proximity.describe``.

    python verdicts.py <сценарий> <каталог> [--mode on|off] [--owner ИМЯ] [--need "к1 к2"] [--grace S]
Код выхода: 0 PASS, 1 FAIL, 2 NOT_TRIGGERED/NO_DATA. Строка вердикта дописывается в ``<каталог>/../verdicts.txt``.
"""

from __future__ import annotations

import argparse
import datetime as dt
import importlib.util
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[3]
PHRASES_FILE = ROOT / "src" / "rob_box_dialog" / "rob_box_dialog" / "phrases" / "ru.yaml"
PROXIMITY_FILE = ROOT / "src" / "rob_box_teleop" / "rob_box_teleop" / "proximity.py"

PASS, FAIL, NOT_TRIGGERED, NO_DATA = "PASS", "FAIL", "NOT_TRIGGERED", "NO_DATA"
EXIT_CODES = {PASS: 0, FAIL: 1, NOT_TRIGGERED: 2, NO_DATA: 2}

#: Статусы action_msgs/GoalStatus, как их пишет проба.
TERMINAL_FAIL = ("ABORTED", "CANCELED")
ACTIVE = ("ACCEPTED", "EXECUTING")
#: Скорость на выходе twist_mux, которую считаем «едет» (м/с, рад/с).
MOVE_EPS = 0.01
#: Сдвиг правды Gazebo, который считаем «стоит» за время удержания стопа (м).
HOLD_DRIFT_M = 0.03
#: Сколько сторожу сирот дать сверх ``orphan_timeout_s`` (1.5 с sim): такт 0.1 с + отмена.
DEFAULT_GRACE_S = 2.0
#: Окно после события, в котором ждём фразу робота (с, стена).
PHRASE_WINDOW_S = 30.0
#: Писк и голос защиты после входа в «стоп» (с, стена; часы Main и Sound Pi разные — запас).
BEEP_LAG_S = 2.0
SPEECH_LAG_S = 6.0
#: Нет ни одной строки пробы: не стартовала или упала (traceback — в probe.log).
NO_PROBE = "в probe.log нет строк пробы — не стартовала или упала, см. файл"
#: Тулы навигации, исход которых озвучивает класс ``action.motion``.
NAV_TOOLS = ("navigate_to_coordinates", "navigate_to_waypoint", "move_direction")


# ── raw → события ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Event:
    """Строка raw: время UTC (эпоха), вид (KIND пробы или LOG для docker logs), поля ``k=v`` и весь текст."""

    t: float
    kind: str
    fields: Mapping[str, str] = field(default_factory=dict)
    text: str = ""

    def num(self, key: str, default: float = math.nan) -> float:
        try:
            return float(self.fields[key])
        except (KeyError, ValueError):
            return default


@dataclass(frozen=True)
class Verdict:
    issue: str
    scenario: str
    status: str
    facts: Tuple[str, ...]

    def line(self) -> str:
        return f"VERDICT {self.issue} {self.scenario} {self.status}: " + "; ".join(self.facts)


_KV = re.compile(r"(\w+)=(\S+)")
_DOCKER_TS = re.compile(r"^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)(?:\.(\d+))?Z\s?(.*)$")


def parse_probe(lines: Iterable[str]) -> List[Event]:
    """Строки пробы ``<эпоха> <KIND> k=v ...``; шум (zenoh, трассы) пропускается."""
    out = []
    for raw in lines:
        parts = raw.strip().split(" ", 2)
        if len(parts) < 2 or not parts[1].isupper():
            continue
        try:
            t = float(parts[0])
        except ValueError:
            continue
        rest = parts[2] if len(parts) > 2 else ""
        out.append(Event(t, parts[1], dict(_KV.findall(rest)), rest))
    return out


def parse_docker(lines: Iterable[str]) -> List[Event]:
    """``docker logs --timestamps``: ``2026-11-09T16:34:42.123456789Z <строка>`` → Event(kind=LOG)."""
    out = []
    for raw in lines:
        m = _DOCKER_TS.match(raw.rstrip("\r\n"))
        if not m:
            continue
        frac = float("0." + (m.group(2) or "0")[:6])
        base = dt.datetime.strptime(m.group(1), "%Y-%m-%dT%H:%M:%S").replace(tzinfo=dt.timezone.utc)
        out.append(Event(base.timestamp() + frac, "LOG", {}, m.group(3)))
    return out


def grep(events: Sequence[Event], pattern: str, since: float = -math.inf, until: float = math.inf) -> List[Event]:
    rx = re.compile(pattern)
    return [e for e in events if since <= e.t <= until and rx.search(e.text)]


def kinds(events: Sequence[Event], *names: str) -> List[Event]:
    return [e for e in events if e.kind in names]


def hhmmss(t: float) -> str:
    return dt.datetime.fromtimestamp(t, dt.timezone.utc).strftime("%H:%M:%S.%f")[:-3]


# ── знания из одного источника ────────────────────────────────────────────────────────────────


def load_phrases(path: Path = PHRASES_FILE) -> Dict[str, List[str]]:
    """Ключ фразы → варианты (``phrases/ru.yaml`` диалога)."""
    import yaml  # noqa: PLC0415 - нужен только тут

    with open(path, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    return {k: (v if isinstance(v, list) else [v]) for k, v in doc.items() if isinstance(v, (list, str))}


def phrase_matcher(phrases: Mapping[str, List[str]], *keys: str) -> "re.Pattern[str]":
    """Регекс «текст совпадает с одним из вариантов ключей»; ``{target}`` — любое непустое."""
    alts = []
    for key in keys:
        for variant in phrases[key]:
            alts.append(re.sub(r"\\\{\w+\\\}", ".+", re.escape(variant)))
    return re.compile(r"^(?:" + "|".join(alts) + r")$")


def load_proximity(path: Path = PROXIMITY_FILE):
    """``rob_box_teleop.proximity`` по пути файла (без установки пакета): ALARMS, describe."""
    spec = importlib.util.spec_from_file_location("hil_checks_proximity", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_TTS = re.compile(r"TTS: .*?text=(['\"])(.*)\1\s*$")
_TRIGGER = re.compile(r"Триггер: (\S+)")
_ZONE = re.compile(r"зона (\w+) → (\w+) \(до корпуса ([0-9.]+|—)")


def tts(events: Sequence[Event]) -> List[Tuple[float, str]]:
    """Что сказал робот: строки ``tts_node`` ``🔊 TTS: ... text='...'``."""
    out = []
    for e in events:
        m = _TTS.search(e.text)
        if m:
            out.append((e.t, m.group(2)))
    return out


def triggers(events: Sequence[Event], names: Sequence[str], since: float, until: float) -> List[Tuple[float, str]]:
    """Писки ``sound_node`` (``🔔 Триггер: <имя>``) из списка имён в окне."""
    out = []
    for e in events:
        m = _TRIGGER.search(e.text)
        if m and m.group(1) in names and since <= e.t <= until:
            out.append((e.t, m.group(1)))
    return out


def zone_transitions(events: Sequence[Event]) -> List[Tuple[float, str, str, float]]:
    """Смены зоны защиты: (время, было, стало, зазор до корпуса или nan)."""
    out = []
    for e in events:
        m = _ZONE.search(e.text)
        if m:
            gap = math.nan if m.group(3) == "—" else float(m.group(3))
            out.append((e.t, m.group(1), m.group(2), gap))
    return out


# ── предусловия ───────────────────────────────────────────────────────────────────────────────


def parse_inspect(lines: Iterable[str], now: float) -> Dict[str, Dict[str, object]]:
    """``docker inspect -f '{{.Name}} статус здоровье рестарты StartedAt'`` → имя → поля; «No such object» — absent."""
    out: Dict[str, Dict[str, object]] = {}
    for raw in lines:
        line = raw.strip()
        miss = re.search(r"No such object: (\S+)", line)
        if miss:
            out[miss.group(1)] = {"state": "absent", "health": "-", "restarts": 0, "age_s": 0.0}
            continue
        parts = line.split()
        if len(parts) != 5 or not parts[0].startswith("/"):
            continue
        started = parse_docker([parts[4] + " x"])
        age = now - started[0].t if started else 0.0
        out[parts[0][1:]] = {"state": parts[1], "health": parts[2], "restarts": int(parts[3]), "age_s": age}
    return out


def v_preflight(
    lock_text: str,
    owner: str,
    status_text: str,
    inspect: Mapping[str, Mapping[str, object]],
    need: Sequence[str],
    skew_s: Mapping[str, float],
    max_skew_s: float = 5.0,
    min_age_s: float = 15.0,
) -> Verdict:
    """Стенд мой, HIL включён, нужные контейнеры живут (не restarting, здоровы, не только что поднялись)."""
    bad, facts = [], []
    holder = lock_text.split()[0] if lock_text.split() else "нет"
    (facts if holder == owner else bad).append(f"stand.lock: {lock_text.strip() or 'нет файла'}")
    if not owner:
        bad.append("HIL_CHECKS_OWNER не задан")
    hil_on = re.search(r"HIL on since \S+", status_text)
    (facts if hil_on else bad).append(hil_on.group(0) if hil_on else "hil.sh status: замка HIL нет (HIL выключен)")
    cm = re.search(r"/controller_manager в графе[^:]*: (\S+)", status_text)
    cm_ok = cm is not None and cm.group(1) == "1"
    (facts if cm_ok else bad).append(f"/controller_manager в графе: {cm.group(1) if cm else 'gazebo не работает'}")
    for name in need:
        info = inspect.get(name)
        if info is None:
            bad.append(f"{name}: нет данных docker inspect")
            continue
        ok = info["state"] == "running" and info["health"] in ("healthy", "none") and info["age_s"] >= min_age_s
        text = f"{name}: {info['state']} {info['health']} restarts={info['restarts']} up={info['age_s']:.0f}s"
        (facts if ok else bad).append(text)
    for host, skew in skew_s.items():
        (facts if abs(skew) <= max_skew_s else bad).append(f"часы {host}: {skew:+.1f} с")
    if bad:
        return Verdict("-", "preflight", FAIL, tuple(bad + facts))
    return Verdict("-", "preflight", PASS, tuple(facts))


# ── навигация: остановка после провала цели (#4435, #4456, #4458) ───────────────────────────────


def _sim(e: Event) -> float:
    return e.num("sim", e.t)


def follow_active_at(probe: Sequence[Event], sim_t: float) -> List[str]:
    """Цели FollowPath, которые по последнему известному статусу активны в момент ``sim_t``."""
    last: Dict[str, str] = {}
    for e in kinds(probe, "FP_STATUS"):
        if _sim(e) <= sim_t:
            last[e.fields.get("goal", "?")] = e.fields.get("status", "?")
    return sorted(g for g, s in last.items() if s in ACTIVE)


def moving_mux(probe: Sequence[Event], since_sim: float, until_sim: float) -> List[Event]:
    """Ненулевые команды на выходе twist_mux в окне sim-времени."""
    return [
        e
        for e in kinds(probe, "MUX")
        if since_sim <= _sim(e) <= until_sim and (abs(e.num("v", 0.0)) > MOVE_EPS or abs(e.num("w", 0.0)) > MOVE_EPS)
    ]


def halt_violations(probe: Sequence[Event], grace_s: float) -> Tuple[List[str], List[Event]]:
    """Нарушения «провал цели → робот стоит»: для каждой NavigateToPose в ABORTED/CANCELED через ``grace_s`` (sim)
    нет активного FollowPath и нет ненулевой команды на выходе twist_mux до следующей цели.
    Возвращает (нарушения, терминальные события NavigateToPose)."""
    navs = kinds(probe, "NAV_STATUS")
    terminal = [e for e in navs if e.fields.get("status") in TERMINAL_FAIL]
    bad = []
    for e in terminal:
        t0 = _sim(e) + grace_s
        nxt = [_sim(n) for n in navs if _sim(n) > _sim(e) and n.fields.get("status") in ACTIVE]
        t1 = min(nxt) if nxt else math.inf
        active = follow_active_at(probe, t0)
        if active:
            bad.append(
                f"NavigateToPose {e.fields.get('status')} в sim={_sim(e):.2f}, через {grace_s} с FollowPath "
                f"ещё активен: {','.join(g[:8] for g in active)}"
            )
        moving = moving_mux(probe, t0, t1)
        if moving:
            m = moving[0]
            bad.append(
                f"после {e.fields.get('status')} (sim={_sim(e):.2f}) выход twist_mux ≠ 0: {len(moving)} команд, "
                f"первая sim={_sim(m):.2f} v={m.fields.get('v')} w={m.fields.get('w')}"
            )
    return bad, terminal


def v_orphan_synthetic(probe: Sequence[Event], watchdog: Sequence[Event], grace_s: float) -> Verdict:
    """#4458: своя цель FollowPath после отменённой NavigateToPose (сирота нарочно) гасится сторожем за ``grace_s``."""
    sc = ("#4458", "orphan")
    if not probe:
        return Verdict(*sc, NO_DATA, (NO_PROBE,))
    skip = kinds(probe, "ORPHAN_SKIP", "ERROR")
    if skip:
        return Verdict(*sc, NOT_TRIGGERED, tuple(f"{e.kind} {e.text}" for e in skip))
    nav_end = [e for e in kinds(probe, "NAV_RESULT")]
    acc = kinds(probe, "FP_ACCEPTED")
    res = kinds(probe, "FP_RESULT")
    if not acc or not res:
        return Verdict(*sc, NOT_TRIGGERED, ("своя цель FollowPath не принята или нет итога",))
    goal = acc[0].fields.get("goal", "?")
    facts = [
        f"NavigateToPose до сироты: {nav_end[0].fields.get('status') if nav_end else 'нет'}",
        f"FollowPath {goal[:8]} принят sim={_sim(acc[0]):.2f}",
    ]
    dur = _sim(res[0]) - _sim(acc[0])
    status = res[0].fields.get("status")
    facts.append(f"итог {status} через {dur:.2f} с sim")
    drove = moving_mux(probe, _sim(acc[0]), _sim(res[0]))
    facts.append(f"сирота ехала: {len(drove)} ненулевых команд mux до отмены")
    fired = grep(watchdog, rf"#4458 сирота FollowPath goal_id={re.escape(goal[:8])}")
    facts.append(f"журнал сторожа: {fired[0].text.strip()[:160] if fired else 'строки о цели нет'}")
    after = moving_mux(probe, _sim(res[0]) + 0.5, math.inf)
    if after:
        facts.append(f"после отмены выход twist_mux ≠ 0: {len(after)} команд")
    bad = status != "CANCELED" or dur > grace_s or not fired or bool(after) or bool(kinds(probe, "FP_SELF_CANCEL"))
    return Verdict(*sc, FAIL if bad else PASS, tuple(facts))


def v_orphan_natural(probe: Sequence[Event], watchdog: Sequence[Event], grace_s: float) -> Verdict:
    """#4435 (сценарий 3b): цель из скрипта; после провала или отмены цели робот стоит.

    Сирота не возникла — NOT_TRIGGERED: сторож не проверен."""
    sc = ("#4435", "orphan_natural")
    if not probe:
        return Verdict(*sc, NO_DATA, (NO_PROBE,))
    bad, terminal = halt_violations(probe, grace_s)
    result = kinds(probe, "NAV_RESULT")
    facts = [f"итог цели: {result[0].fields.get('status') if result else 'нет'}"]
    fired = grep(watchdog, r"#4458 сирота FollowPath")
    facts.append(f"срабатываний сторожа: {len(fired)}")
    orphans = [e for e in terminal if follow_active_at(probe, _sim(e) + 0.2)]
    facts.append(f"сирот (FollowPath активен после провала цели): {len(orphans)}")
    if bad:
        return Verdict(*sc, FAIL, tuple(bad + facts))
    if not terminal:
        return Verdict(*sc, NOT_TRIGGERED, tuple(facts + ["цель не ушла в ABORTED/CANCELED"]))
    if not orphans:
        return Verdict(
            *sc,
            NOT_TRIGGERED,
            tuple(
                facts
                + [
                    f"после провала через {grace_s} с FollowPath нет и mux = 0, "
                    "но сироты не было — сторож не проверен"
                ]
            ),
        )
    return Verdict(*sc, PASS, tuple(facts + [f"через {grace_s} с после провала FollowPath нет и mux = 0"]))


def v_tool_halt(probe: Sequence[Event], grace_s: float) -> Verdict:
    """#4435/#4456: после провала цели тула навигации (из диалога) через ``grace_s`` нет FollowPath и mux = 0."""
    sc = ("#4435", "tool_halt")
    if not probe:
        return Verdict(*sc, NO_DATA, (NO_PROBE,))
    bad, terminal = halt_violations(probe, grace_s)
    if bad:
        return Verdict(*sc, FAIL, tuple(bad))
    if not terminal:
        return Verdict(*sc, NOT_TRIGGERED, ("NavigateToPose в ABORTED/CANCELED за окно не было",))
    ran = [e for e in kinds(probe, "FP_STATUS") if e.fields.get("status") in ACTIVE]
    note = "FollowPath за окно запускался" if ran else "FollowPath не запускался (план не построен) — проверка слабая"
    return Verdict(*sc, PASS, (f"{len(terminal)} провалов цели, через {grace_s} с FollowPath нет и mux = 0", note))


# ── фразы тулов движения (#3789) ──────────────────────────────────────────────────────────────


_NAV_RESULT = re.compile(r"Инструмент (" + "|".join(NAV_TOOLS) + r") завершился с ошибкой: (.*)$")
_NAV_OK = re.compile(r"✅ (?:Навигация|Движение) ")


def _spoken(voice: Sequence[Event], since: float, until: float) -> List[Tuple[float, str]]:
    return [(t, s) for t, s in tts(voice) if since <= t <= until]


def _said(spoken: Sequence[Tuple[float, str]]) -> str:
    return "; ".join(f"{hhmmss(t)} «{s}»" for t, s in spoken) or "робот молчал"


def v_nav_aborted(voice: Sequence[Event], phrases: Mapping[str, List[str]]) -> Verdict:
    """#3789: исход тула ABORTED → фраза «Не доехал…: навигация прервалась.» по данным, без «Приехал»."""
    sc = ("#3789", "nav_aborted")
    if not voice:
        return Verdict(*sc, NO_DATA, ("лог voice-assistant пуст",))
    lie = phrase_matcher(phrases, "motion.done", "motion.arrived")
    aborted = phrase_matcher(phrases, "motion.rejected.aborted", "motion.not_reached.aborted")
    lies = [(t, s) for t, s in tts(voice) if lie.match(s)]
    asked = [m.group(1) for m in (re.search(r"Отправлен запрос \S+: (\w+)", e.text) for e in voice) if m]
    results = [(e.t, _NAV_RESULT.search(e.text)) for e in voice if _NAV_RESULT.search(e.text)]
    ok = grep(voice, _NAV_OK.pattern)
    facts = [f"запросы тулов: {', '.join(asked) or 'нет'}"]
    transport = grep(voice, r"Timeout ожидания результата для (" + "|".join(NAV_TOOLS) + ")")
    if transport:
        facts.append(f"транспорт не дождался тула (#4344): {hhmmss(transport[0].t)}")
    if lies:
        return Verdict(*sc, FAIL, tuple(facts + [f"ложный успех: {_said(lies)}"]))
    hit = [(t, m) for t, m in results if "ABORTED" in m.group(2)]
    if not hit:
        seen = [f"{m.group(1)}: {m.group(2)}" for _, m in results] + [e.text.strip()[-80:] for e in ok]
        return Verdict(*sc, NOT_TRIGGERED, tuple(facts + [f"исхода ABORTED не было; исходы: {seen or 'нет'}"]))
    t_a, m = hit[0]
    facts.append(f"{hhmmss(t_a)} {m.group(1)}: {m.group(2)}")
    spoken = _spoken(voice, t_a - 1.0, t_a + PHRASE_WINDOW_S)
    facts.append(f"сказано: {_said(spoken)}")
    good = any(aborted.match(s) for _, s in spoken)
    return Verdict(*sc, PASS if good else FAIL, tuple(facts))


def v_stop(
    voice: Sequence[Event],
    phrases: Mapping[str, List[str]],
    expect_key: str,
    scenario: str,
    probe: Optional[Sequence[Event]] = None,
) -> Verdict:
    """#3789: тул ``stop_navigation`` → фраза кода по виду отмены (``motion.stop.idle`` / ``motion.stop.confirmed``).

    ``probe`` — для «во время поездки»: цель из скрипта должна кончиться CANCELED (её отменил тул), иначе отменять
    было нечего и случай не наступил."""
    sc = ("#3789", scenario)
    if not voice:
        return Verdict(*sc, NO_DATA, ("лог voice-assistant пуст",))
    if probe is not None:
        result = kinds(probe, "NAV_RESULT")
        status = result[0].fields.get("status") if result else "нет"
        if status != "CANCELED" or kinds(probe, "NAV_SELF_CANCEL"):
            return Verdict(*sc, NOT_TRIGGERED, (f"цель из скрипта не отменена тулом: итог {status}",))
    lie = phrase_matcher(phrases, "motion.done", "motion.arrived")
    want = phrase_matcher(phrases, expect_key)
    called = grep(voice, r"\[stop_navigation\] Остановка навигации")
    tier1 = [re.search(r"tier1 kind=(\w+)", e.text).group(1) for e in grep(voice, r"tier1 kind=\w+")]
    asked = [m.group(1) for m in (re.search(r"Отправлен запрос \S+: (\w+)", e.text) for e in voice) if m]
    facts = [f"tier1: {', '.join(tier1) or 'нет'}", f"запросы тулов: {', '.join(asked) or 'нет'}"]
    if not called:
        return Verdict(*sc, NOT_TRIGGERED, tuple(facts + ["stop_navigation не вызван (модель выбрала другое)"]))
    t_s = called[0].t
    outcome = grep(voice, r"\[stop_navigation\] (Отмена подтверждена: \w+|⚠️.*)", since=t_s)
    facts.append(
        f"{hhmmss(t_s)} stop_navigation, {outcome[0].text.split('] ')[-1].strip() if outcome else 'исхода нет'}"
    )
    spoken = _spoken(voice, t_s, t_s + PHRASE_WINDOW_S)
    facts.append(f"сказано: {_said(spoken)}")
    if any(lie.match(s) for _, s in spoken):
        return Verdict(*sc, FAIL, tuple(facts + ["ложный успех «Приехал»"]))
    good = any(want.match(s) for _, s in spoken)
    facts.append(f"ожидалось {expect_key} = «{phrases[expect_key][0]}»")
    return Verdict(*sc, PASS if good else FAIL, tuple(facts))


# ── зона безопасности (#4365) и звук защиты в HIL (#4470) ───────────────────────────────────────


@dataclass(frozen=True)
class ZoneRun:
    """Один наезд на препятствие: что видела проба и что было на роботе в окне наезда."""

    probe: Sequence[Event]
    guard: Sequence[Event]
    voice: Sequence[Event]
    lights: int  # сколько ExpressionRequest с маячком тревоги поймал ``ros2 topic echo``

    def window(self) -> Tuple[float, float]:
        return (self.probe[0].t - 1.0, self.probe[-1].t + 5.0) if self.probe else (0.0, 0.0)

    def stops(self) -> List[Tuple[float, str, str, float]]:
        lo, hi = self.window()
        return [z for z in zone_transitions(self.guard) if z[2] == "stop" and lo <= z[0] <= hi]


def _hold_drift(probe: Sequence[Event]) -> Optional[float]:
    """Сдвиг правды Gazebo от EM_ON + 0.5 с до конца толкания (м): стоп держит, пока команда толкает."""
    em = kinds(probe, "EM_ON")
    end = kinds(probe, "PUSH_END")
    if not em or not end:
        return None
    track = [e for e in kinds(probe, "TRACK") if em[0].t + 0.5 <= e.t <= end[0].t]
    if len(track) < 2:
        return None
    return math.dist((track[0].num("x"), track[0].num("y")), (track[-1].num("x"), track[-1].num("y")))


def v_zone(run: ZoneRun, px) -> Verdict:
    """#4365: в движении у препятствия — стоп (аварийный вход mux, робот стоит, пока толкает команда), и вместе с ним
    писк, «Осторожно!» и маячок (Шифу 04.11: «звучат вместе и согласованно»)."""
    sc = ("#4365", "zone")
    if not run.probe:
        return Verdict(*sc, NO_DATA, (NO_PROBE,))
    skipped = kinds(run.probe, "WALL_NOT_FOUND", "ROTATE_FAIL", "ERROR")
    if skipped:
        return Verdict(*sc, NOT_TRIGGERED, tuple(e.kind + " " + e.text for e in skipped))
    stops = run.stops()
    em = kinds(run.probe, "EM_ON")
    end = kinds(run.probe, "SCAN_END")
    if not stops or not em:
        return Verdict(
            *sc,
            FAIL,
            (
                f"доехал до препятствия, а стопа нет: переходов в stop {len(stops)}, EM_ON {len(em)}",
                f"зазор впереди в конце: {end[0].text if end else '?'}",
            ),
        )
    t_stop, prev, _, gap = stops[0]
    alarm = px.ALARMS[px.ZONE_STOP]
    lo, hi = run.window()
    beeps = triggers(run.voice, (alarm.sound,), lo, hi)
    said = [(t, s) for t, s in _spoken(run.voice, lo, hi) if s == alarm.phrase]
    drift = _hold_drift(run.probe)
    facts = [
        f"{hhmmss(t_stop)} зона {prev} → stop, до корпуса {gap:.2f} м",
        f"EM_ON {hhmmss(em[0].t)}",
        f"сдвиг за удержание стопа: {'?' if drift is None else f'{drift:.3f} м'}",
        f"писк {alarm.sound}: {len(beeps)}" + (f", первый через {beeps[0][0] - t_stop:+.2f} с" if beeps else ""),
        f"«{alarm.phrase}»: {len(said)}" + (f", через {said[0][0] - t_stop:+.2f} с" if said else ""),
        f"маячок {alarm.animation}: {run.lights} запросов",
    ]
    bad = (
        not (gap > 0.0)
        or drift is None
        or drift > HOLD_DRIFT_M
        or not beeps
        or not said
        or run.lights < 1
        or beeps[0][0] - t_stop > BEEP_LAG_S
        or said[0][0] - t_stop > SPEECH_LAG_S
    )
    return Verdict(*sc, FAIL if bad else PASS, tuple(facts))


def v_guard_sound(run: ZoneRun, px, mode: str, env_text: str) -> Verdict:
    """#4470: ``HIL_GUARD_SOUND=on`` — писк уходит на настоящий робот; ``off`` — писка и голоса нет, строка в журнале
    защиты, а стоп и маячок работают."""
    sc = ("#4470", f"guard_sound_{mode}")
    if not run.probe:
        return Verdict(*sc, NO_DATA, (NO_PROBE,))
    env = dict(re.findall(r"(\w+)=(\S+)", env_text))
    facts = [
        f"env proximity-guard: HIL_GUARD_SOUND={env.get(px.GUARD_SOUND_ENV, 'unset')} "
        f"USE_SIM_TIME={env.get(px.SIM_TIME_ENV, 'unset')}"
    ]
    if env.get(px.GUARD_SOUND_ENV, "on") != mode or env.get(px.SIM_TIME_ENV) != "true":
        return Verdict(*sc, FAIL, tuple(facts + [f"контейнер не в режиме {mode} или не в HIL"]))
    stops = run.stops()
    if not stops:
        return Verdict(*sc, NOT_TRIGGERED, tuple(facts + ["в окне наезда защита не входила в stop"]))
    lo, hi = run.window()
    names = tuple(a.sound for a in px.ALARMS.values())
    phrases = tuple(a.phrase for a in px.ALARMS.values() if a.phrase)
    beeps = triggers(run.voice, names, lo, hi)
    said = [(t, s) for t, s in _spoken(run.voice, lo, hi) if s in phrases]
    muted = grep(run.guard, r"HIL: зона \w+, на настоящий робот не уходит", lo, hi)
    em = kinds(run.probe, "EM_ON")
    facts += [
        f"входов в stop: {len(stops)}",
        f"EM_ON: {len(em)}",
        f"писков на Sound Pi: {len(beeps)}",
        f"фраз защиты: {len(said)}",
        f"строк «не уходит» в журнале защиты: {len(muted)}",
        f"маячок: {run.lights} запросов",
    ]
    if mode == "on":
        ok = bool(beeps) and not muted
    else:
        ok = not beeps and not said and bool(muted) and bool(em) and run.lights >= 1
    return Verdict(*sc, PASS if ok else FAIL, tuple(facts))


# ── CLI ───────────────────────────────────────────────────────────────────────────────────────


def _read(d: Path, name: str) -> List[str]:
    p = d / name
    return p.read_text(encoding="utf-8", errors="replace").splitlines() if p.is_file() else []


def _zone_run(d: Path, px) -> ZoneRun:
    lights = sum(line.count(px.ALARM_LIGHTS) for line in _read(d, "expr.log"))
    return ZoneRun(
        parse_probe(_read(d, "probe.log")),
        parse_docker(_read(d, "guard.log")),
        parse_docker(_read(d, "voice.log")),
        lights,
    )


def _skews(d: Path) -> Dict[str, float]:
    """``clock.txt``: строки ``<хост> <эпоха>``, первая — машина оператора."""
    rows = [line.split() for line in _read(d, "clock.txt") if len(line.split()) == 2]
    if not rows:
        return {}
    base = float(rows[0][1])
    return {host: float(t) - base for host, t in rows[1:]}


def evaluate(scenario: str, d: Path, args) -> Verdict:
    """Сценарий + каталог raw → вердикт."""
    if scenario == "preflight":
        import time  # noqa: PLC0415

        return v_preflight(
            "\n".join(_read(d, "stand_lock.txt")),
            args.owner,
            "\n".join(_read(d, "hil_status.txt")),
            parse_inspect(_read(d, "inspect.txt"), time.time()),
            args.need.split(),
            _skews(d),
        )
    probe = parse_probe(_read(d, "probe.log"))
    if scenario == "orphan":
        return v_orphan_synthetic(probe, parse_docker(_read(d, "watchdog.log")), args.grace)
    if scenario == "orphan_natural":
        return v_orphan_natural(probe, parse_docker(_read(d, "watchdog.log")), args.grace)
    if scenario == "tool_halt":
        return v_tool_halt(probe, args.grace)
    voice = parse_docker(_read(d, "voice.log"))
    if scenario == "nav_aborted":
        return v_nav_aborted(voice, load_phrases())
    if scenario in ("stop_idle", "stop_active"):
        if scenario == "stop_idle":
            return v_stop(voice, load_phrases(), "motion.stop.idle", scenario)
        return v_stop(voice, load_phrases(), "motion.stop.confirmed", scenario, probe)
    px = load_proximity()
    if scenario == "zone":
        return v_zone(_zone_run(d, px), px)
    if scenario == "guard_sound":
        return v_guard_sound(_zone_run(d, px), px, args.mode, "\n".join(_read(d, "env.txt")))
    raise SystemExit(f"неизвестный сценарий {scenario}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("scenario")
    ap.add_argument("dir", type=Path)
    ap.add_argument("--mode", default="on", choices=("on", "off"))
    ap.add_argument("--owner", default="")
    ap.add_argument("--need", default="")
    ap.add_argument("--grace", type=float, default=DEFAULT_GRACE_S)
    args = ap.parse_args(argv)
    verdict = evaluate(args.scenario, args.dir, args)
    print(verdict.line())
    with open(args.dir.parent / "verdicts.txt", "a", encoding="utf-8") as fh:
        fh.write(verdict.line() + "\n")
    return EXIT_CODES[verdict.status]


if __name__ == "__main__":
    sys.exit(main())
