#!/usr/bin/env python3
"""conductor.py — голосовой дирижёр сцены: запись бэга + разметка по ходу (ADR-0144 §4.1).

Читает YAML-сценарий сцены (``scenes/sNN_*.yaml``), сам запускает
``ros2 bag record``, по шагам говорит команды роботом и ставит метки
событий по часам хоста; в конце пишет ``scene.yaml`` (формат —
``scene_spec.py``) рядом с бэгом.

Правила (пилот 29.09.2026, согласовано с владельцем):
  * команды повелительные и звучат тогда, когда действие должно
    произойти («войдите в кадр», «выйдите из кадра»);
  * перед каждым ``enter`` живого участника дирижёр САМ проверяет пустой
    кадр: ≥ ``--empty-hold-s`` (3 с) без наблюдений person/face в
    ``/perception/observations`` (статичные участники в своём коридоре
    ``anchor_cx`` не мешают), говорит «кадр пуст», ставит метку ``empty``;
    не пусто за ``--empty-timeout-s`` — повторяет «выйдите из кадра»;
  * ``leave`` проверяется так же: метка ставится на начало пустого
    периода, а не на момент команды;
  * пока камера молчит (нет ``…/compressed`` дольше 1 с), кадр не
    считается пустым — «не видно» ≠ «никого нет».

Шаг сценария — словарь, в нём не больше одного действия:
    enter: <label>      вход (с проверкой пустого кадра, если участник живой)
    leave: <label>      выход (метка по проверенно пустому кадру)
    speech: <label>     окно речи участника; window_s — длина; introduce: <имя> — представление
    mark: {type, who}   справочная метка (glasses_on, turn_back …)
    consent: <label>    устное согласие гостя + подтверждение оператора в stdin
и общие поля: say (что сказать роботом ДО действия/метки), wait_s (пауза после).

Запуск — через ``record_scene.sh`` на хосте Vision Pi (внутри voice-assistant).
``--dry-run`` печатает шаги без ROS.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import scene_spec as ss  # noqa: E402
from replay import RECORD_TOPICS  # noqa: E402

ACTIONS = ("enter", "leave", "speech", "mark", "consent")
BLOCKING_CLASSES = ("person", "face")
CAMERA_TOPIC = "/camera/camera/color/image_raw/compressed"
CAMERA_STALE_S = 1.0

SAY_EMPTY = "Кадр пуст."
SAY_GET_OUT = "Выйдите из кадра."


class ScriptError(ValueError):
    """Сценарий нельзя исполнить однозначно."""


# ── сценарий ─────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Step:
    action: str  # "" — только реплика
    who: str = ""
    say: str = ""
    wait_s: float = 0.0
    window_s: float = 0.0
    introduce: str = ""
    mark: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class Script:
    scene: str
    participants: List[Mapping[str, Any]]
    steps: List[Step]
    expected: Dict[str, Any] = field(default_factory=dict)
    description: str = ""


def _parse_step(raw: Mapping[str, Any], idx: int, labels: Sequence[str]) -> Step:
    where = f"steps[{idx}]"
    actions = [a for a in ACTIONS if a in raw]
    if len(actions) > 1:
        raise ScriptError(f"{where}: больше одного действия {actions}")
    action = actions[0] if actions else ""
    who = "" if action in ("", "mark") else str(raw[action])
    if who and who not in labels:
        raise ScriptError(f"{where}: {action}: участника {who!r} нет")
    mark = dict(raw.get("mark") or {})
    if action == "mark" and mark.get("type") not in ss.INFO_EVENT_TYPES:
        raise ScriptError(f"{where}: mark.type {mark.get('type')!r} не из {ss.INFO_EVENT_TYPES}")
    window_s = float(raw.get("window_s", 0.0))
    if action == "speech" and window_s <= 0:
        raise ScriptError(f"{where}: у speech нужен window_s > 0")
    if not action and not raw.get("say"):
        raise ScriptError(f"{where}: пустой шаг")
    return Step(action, who, str(raw.get("say") or ""), float(raw.get("wait_s", 0.0)),
                window_s, str(raw.get("introduce") or ""), mark)


def parse_script(raw: Mapping[str, Any]) -> Script:
    scene = str(raw.get("scene") or "")
    if not scene:
        raise ScriptError("нет поля scene")
    participants = list(raw.get("participants") or [])
    labels = [ss.parse_participant(p, i).label for i, p in enumerate(participants)]
    steps = [_parse_step(s, i, labels) for i, s in enumerate(raw.get("steps") or [])]
    if not steps:
        raise ScriptError("steps: пусто")
    return Script(scene, participants, steps, dict(raw.get("expected") or {}), str(raw.get("description") or ""))


def ssml(text: str) -> str:
    """Реплика → SSML: ``tts_node`` без поля ``ssml`` молчит (память speak-through-robot)."""
    return f"<speak>{escape(text)}</speak>"


def tts_request(text: str) -> str:
    return json.dumps({"ssml": ssml(text), "source": "operator"}, ensure_ascii=False)


# ── пустой кадр ──────────────────────────────────────────────────────────────


def is_blocking(
    class_name: str, cx: Optional[float], anchors: Sequence[Tuple[float, float]],
    region: Optional[Tuple[float, float]] = None,
) -> bool:
    """Мешает ли наблюдение считать «пусто».

    Без ``region`` — весь кадр: person/face вне коридоров статичных
    участников. С ``region`` — только этот коридор (уход статичного
    участника: маску уносит человек, который сам ещё в кадре).
    """
    if class_name not in BLOCKING_CLASSES:
        return False
    if region is not None:
        return cx is None or region[0] <= cx <= region[1]
    if cx is None:
        return True
    return not any(lo <= cx <= hi for lo, hi in anchors)


class EmptyWatcher:
    """Кадр пуст ≥ ``hold_s``, считая от ``start`` проверки, при живой камере.

    Время — секунды хоста; подаются снаружи (тестируется без ROS).
    """

    def __init__(self, start: float, hold_s: float) -> None:
        self.hold_s = hold_s
        self.since = start
        self.last_frame: Optional[float] = None

    def frame(self, t: float) -> None:
        if self.last_frame is not None and t - self.last_frame > CAMERA_STALE_S:
            self.since = max(self.since, t)  # камера молчала — пустоту не видели
        self.last_frame = t

    def blocking(self, t: float) -> None:
        self.since = max(self.since, t)

    def empty_since(self, now: float) -> Optional[float]:
        if self.last_frame is None or now - self.last_frame > CAMERA_STALE_S:
            return None
        return self.since if now - self.since >= self.hold_s else None


# ── сборка scene.yaml ────────────────────────────────────────────────────────


def assemble_scene(
    script: Script, marks: Sequence[Mapping[str, Any]], bag_start: float, duration: float,
    recording: Mapping[str, Any], consent: Sequence[Mapping[str, Any]] = (),
) -> Dict[str, Any]:
    """Метки с абсолютным временем (``t_abs``, ``until_abs``) → dict ``scene.yaml``.

    Результат прогоняется через ``scene_spec.parse_scene`` — невалидная
    разметка не пишется.
    """
    events = []
    for m in marks:
        e = {k: v for k, v in m.items() if k not in ("t_abs", "until_abs")}
        e["t"] = round(max(0.0, m["t_abs"] - bag_start), 3)
        if "until_abs" in m:
            e["until"] = round(max(e["t"], m["until_abs"] - bag_start), 3)
        events.append(e)
    doc = {
        "schema": ss.SCHEMA_VERSION,
        "scene": script.scene,
        "description": script.description,
        "recording": dict(recording),
        "duration": round(max([duration] + [e.get("until", e["t"]) for e in events]), 3),
        "participants": list(script.participants),
        "events": events,
        "consent": list(consent),
        "expected": dict(script.expected),
    }
    ss.parse_scene(doc)
    return doc


# ── исполнение (ROS, внутри voice-assistant) ─────────────────────────────────


class Runtime:
    """rclpy: TTS с ожиданием finished, наблюдения, камера, STT для согласия."""

    def __init__(self, anchors_of) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rob_box_perception_msgs.msg import Observation
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        rclpy.init()
        self._String = String
        self.node = rclpy.create_node("scene_conductor")
        self.tts_pub = self.node.create_publisher(String, "/voice/tts/request", 10)
        self._finished = threading.Event()
        self._lock = threading.Lock()
        self.watcher: Optional[EmptyWatcher] = None
        self.region: Optional[Tuple[float, float]] = None
        self.anchors_of = anchors_of
        self.stt: List[Tuple[float, str]] = []
        self.node.create_subscription(String, "/voice/tts/finished", lambda _m: self._finished.set(), 10)
        self.node.create_subscription(Observation, "/perception/observations", self._on_obs, 10)
        self.node.create_subscription(CompressedImage, CAMERA_TOPIC, self._on_frame, qos_profile_sensor_data)
        self.node.create_subscription(String, "/voice/stt/result", self._on_stt, 10)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _on_obs(self, msg) -> None:
        cx = msg.bbox_cx_px / msg.image_width if msg.image_width else None
        with self._lock:
            if self.watcher and is_blocking(msg.class_name, cx, self.anchors_of(), self.region):
                self.watcher.blocking(time.time())

    def _on_frame(self, _msg) -> None:
        with self._lock:
            if self.watcher:
                self.watcher.frame(time.time())

    def _on_stt(self, msg) -> None:
        self.stt.append((time.time(), msg.data))

    def say(self, text: str, timeout_s: float = 25.0) -> bool:
        self._finished.clear()
        self.tts_pub.publish(self._String(data=tts_request(text)))
        return self._finished.wait(timeout_s)

    def wait_empty(
        self, hold_s: float, timeout_s: float, region: Optional[Tuple[float, float]] = None
    ) -> Optional[float]:
        with self._lock:
            self.watcher = EmptyWatcher(time.time(), hold_s)
            self.region = region
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._lock:
                since = self.watcher.empty_since(time.time())
            if since is not None:
                return since
            time.sleep(0.2)
        return None

    def close(self) -> None:
        import rclpy

        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


class Conductor:
    def __init__(self, script: Script, rt: Runtime, args: argparse.Namespace, log) -> None:
        self.script = script
        self.rt = rt
        self.args = args
        self.log = log
        self.marks: List[Dict[str, Any]] = []
        self.consent: List[Dict[str, Any]] = []
        self.present: List[str] = [p["label"] for p in script.participants if p.get("present_at_start")]
        self.parts = {p["label"]: ss.parse_participant(p, i) for i, p in enumerate(script.participants)}

    def anchors(self) -> List[Tuple[float, float]]:
        return [self.parts[w].anchor_cx for w in self.present if self.parts[w].anchor_cx]

    def mark(self, t_abs: float, **fields: Any) -> None:
        self.marks.append(dict(fields, t_abs=t_abs))
        self.log(f"MARK {fields} t_abs={t_abs:.3f}")

    def speak(self, text: str) -> float:
        if text and not self.rt.say(text):
            self.log(f"WARN: нет /voice/tts/finished за 25 с: {text!r}")
        return time.time()

    def wait_empty_or_nag(self, nag: str, region: Optional[Tuple[float, float]] = None) -> float:
        """Ждать пустоты; не дождались — повторить команду; после ``empty_retries`` — прервать сцену."""
        for attempt in range(self.args.empty_retries):
            since = self.rt.wait_empty(self.args.empty_hold_s, self.args.empty_timeout_s, region)
            if since is not None:
                return since
            self.log(f"не пусто за {self.args.empty_timeout_s}s (попытка {attempt + 1}), region={region} — повторяю")
            self.speak(nag)
        raise SystemExit("кадр так и не опустел — сцена прервана")

    def do_enter(self, step: Step) -> None:
        if not self.parts[step.who].is_static:
            since = self.wait_empty_or_nag(SAY_GET_OUT)
            self.speak(SAY_EMPTY)
            self.mark(since, type="empty")
        t = self.speak(step.say)
        self.present.append(step.who)
        self.mark(t, type="enter", who=step.who)

    def do_leave(self, step: Step) -> None:
        self.speak(step.say)
        self.present.remove(step.who)
        part = self.parts[step.who]
        if part.is_static and part.anchor_cx:
            since = self.wait_empty_or_nag(step.say, part.anchor_cx)
        else:
            since = self.wait_empty_or_nag(SAY_GET_OUT)
        self.mark(since, type="leave", who=step.who)

    def do_speech(self, step: Step) -> None:
        t = self.speak(step.say)
        self.mark(t, type="say", who=step.who, until_abs=t + step.window_s, text=step.say)
        if step.introduce:
            self.mark(t, type="introduce", who=step.who, name=step.introduce)
        time.sleep(step.window_s)

    def do_consent(self, step: Step) -> None:
        t = self.speak(step.say)
        time.sleep(step.window_s or 10.0)
        heard = " / ".join(txt for ts, txt in self.rt.stt if ts >= t)
        answer = input(f"[consent] {step.who}: услышано {heard!r}. Согласие подтверждено? [y/N] ")
        if answer.strip().lower() != "y":
            raise SystemExit("согласие не подтверждено — сцена прервана, бэг удалить")
        self.consent.append({"participant": step.who, "t_abs": t, "utterance": heard})
        self.mark(t, type="consent", who=step.who)

    def run(self) -> None:
        for i, step in enumerate(self.script.steps):
            self.log(f"STEP {i}: {step.action or 'say'} {step.who} {step.say!r}")
            handler = getattr(self, f"do_{step.action}", None)
            if handler is not None:
                handler(step)
            elif step.action == "mark":
                self.mark(self.speak(step.say), **dict(step.mark))
            else:
                self.speak(step.say)
            time.sleep(step.wait_s)


def cx_summary(cxs: Sequence[float]) -> str:
    if not cxs:
        return "лиц не видно"
    lo, hi = min(cxs), max(cxs)
    corridor = f"[{max(0.0, lo - 0.05):.2f}, {min(1.0, hi + 0.05):.2f}]"
    return f"n={len(cxs)} cx min={lo:.3f} max={hi:.3f} -> anchor_cx: {corridor}"


def probe_anchor(seconds: float) -> int:
    """Коридор ``anchor_cx`` статичного участника: поставить маску, уйти из кадра, запустить."""
    import rclpy
    from rob_box_perception_msgs.msg import Observation

    rclpy.init()
    node = rclpy.create_node("scene_anchor_probe")
    cxs: List[float] = []
    node.create_subscription(
        Observation, "/perception/observations",
        lambda m: cxs.append(m.bbox_cx_px / m.image_width) if m.class_name == "face" and m.image_width else None,
        10,
    )
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(cx_summary(cxs))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


def _read_bag_meta(bag_dir: str) -> Tuple[float, float]:
    import yaml

    with open(os.path.join(bag_dir, "metadata.yaml"), encoding="utf-8") as f:
        info = yaml.safe_load(f)["rosbag2_bagfile_information"]
    return info["starting_time"]["nanoseconds_since_epoch"] / 1e9, info["duration"]["nanoseconds"] / 1e9


def _record(bag_dir: str) -> subprocess.Popen:
    # Без --qos-profile-overrides-path: с /dev/null падает (ADR-0144 §1.3).
    return subprocess.Popen(["ros2", "bag", "record", "-s", "sqlite3", "-o", bag_dir] + list(RECORD_TOPICS))


def _finalize(script: Script, cond: Conductor, scene_dir: str, recording: Dict[str, Any], log) -> None:
    bag_start, duration = _read_bag_meta(os.path.join(scene_dir, "bag"))
    consent = [
        {"participant": c["participant"], "t": round(c["t_abs"] - bag_start, 3), "utterance": c["utterance"]}
        for c in cond.consent
    ]
    doc = assemble_scene(script, cond.marks, bag_start, duration, recording, consent)
    import yaml

    with open(os.path.join(scene_dir, "scene.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
    log(f"scene.yaml: {len(doc['events'])} events, duration {doc['duration']}s")


def _meta(pairs: Sequence[str]) -> Dict[str, str]:
    return dict(p.split("=", 1) for p in pairs)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--script", required=True)
    ap.add_argument("--out", default="/tmp/scenes")
    ap.add_argument("--meta", action="append", default=[], help="key=value в recording (коммит, образы)")
    ap.add_argument("--empty-hold-s", type=float, default=3.0)
    ap.add_argument("--empty-timeout-s", type=float, default=20.0)
    ap.add_argument("--empty-retries", type=int, default=5)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--probe-anchor-s", type=float, default=0.0,
                    help="только замер: N секунд печатать cx лиц в кадре (коридор подставки) и выйти")
    args = ap.parse_args(argv)

    import yaml

    if args.probe_anchor_s:
        return probe_anchor(args.probe_anchor_s)

    with open(args.script, encoding="utf-8") as f:
        script = parse_script(yaml.safe_load(f))
    if args.dry_run:
        for i, s in enumerate(script.steps):
            print(f"{i:2d} {s.action or 'say':8s} {s.who:6s} wait={s.wait_s:<5} {s.say}")
        return 0

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    scene_dir = os.path.join(args.out, f"{script.scene}_{stamp}")
    os.makedirs(scene_dir)
    logf = open(os.path.join(scene_dir, "conductor.log"), "w", encoding="utf-8")

    def log(msg: str) -> None:
        line = f"{time.time():.3f} {msg}"
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()

    rec = _record(os.path.join(scene_dir, "bag"))
    rt = None
    try:
        time.sleep(3.0)  # бэг открылся и подписался
        cond = Conductor(script, None, args, log)  # type: ignore[arg-type]
        rt = Runtime(cond.anchors)
        cond.rt = rt
        cond.run()
        cond.speak("Сцена записана. Спасибо.")
    finally:
        rec.send_signal(signal.SIGINT)
        rec.wait(timeout=30)
        if rt:
            rt.close()
    recording = dict(_meta(args.meta), bag="bag", started_utc=stamp)
    _finalize(script, cond, scene_dir, recording, log)
    print(f"SCENE_DIR={scene_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
