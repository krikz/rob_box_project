#!/usr/bin/env python3
"""replay.py — офлайн-прогон одной сцены через конвейер 1.1 (или 2.0) на Vision Pi (ADR-0144 §5).

Запускается на ХОСТЕ Vision Pi (python3 stdlib + docker CLI). Поднимает
изолированный граф из отдельных контейнеров:

    router   eclipse/zenoh:1.6.2 — свой роутер, 127.0.0.1:7457, никуда не подключён
    face     образ vision-hailo — start_vision_face.sh, FACE_STORE_ROOT=/replay/faces
    voice    образ voice-assistant — speaker_id_node, db_path=/replay/voice/speakers.db
    decoder  образ vision-hailo — сжатые кадры → Image bgr8 / 16UC1 (decoder_node.py)
    recorder образ voice-assistant — ros2 bag record выходов прогона
    player   образ voice-assistant — ros2 bag play ТОЛЬКО входов сцены

Изоляция — пять барьеров ADR-0144 §5.1; ``check_isolation`` проверяет план
ДО запуска и отказывается стартовать при любом нарушении. ``/data`` не
монтируется никуда.

Живой ``vision-face`` на время прогона обязан быть остановлен: второй
процесс рядом с живым узлом уже ронял его в HAILO_STREAM_ABORT(63)
(22.09.2026). Без ``--stop-live-face`` скрипт откажется, если он запущен;
с флагом — остановит и после прогона перезапустит, напечатав хвост логов.

Пример (сид из прогона s00, выход — ~/scenes/_replay/<run>/<сцена>/):
    python3 replay.py --scene-dir ~/scenes/s01_owner_20261001T101500Z \\
        --seed ~/scenes/_seed/R0 --run-id R1 --stop-live-face
    python3 replay.py --scene-dir ~/scenes/s00_... --empty-seed --save-seed R0 --run-id R0 --stop-live-face
    python3 replay.py ... --dry-run          # только план и проверка изоляции

Результат: ``journal.jsonl`` (прогон), ``control_journal.jsonl`` (живая 1.1
из бэга сцены), ``logs/*.log``, ``out_bag/``. Дальше — ``metrics.py``.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

REPLAY_DOMAIN_ID = 77
ROUTER_ENDPOINT = "tcp/127.0.0.1:7457"
LIVE_METRICS_PORT = 9112
REPLAY_METRICS_PORT = 19112
ZENOH_IMAGE = "eclipse/zenoh:1.6.2"
CONTAINER_PREFIX = "scene-replay-"

#: Входы сцены (ADR-0144 §2.2), которые проигрываются в граф прогона.
INPUT_TOPICS = (
    "/camera/camera/color/image_raw/compressed",
    "/camera/camera/depth/image_rect_raw/compressedDepth",
    "/camera/camera/color/camera_info",
    "/tf_static",
    "/audio/speech_audio",
    "/audio/vad",
    "/audio/direction",
    "/voice/speaker/register",
    "/voice/tts/state",
    "/voice/tts/finished",
)
#: Решения живой 1.1 и метки: не проигрываются НИКОГДА (смешались бы с выходами).
CONTROL_TOPICS = (
    "/voice/speaker/result",
    "/vision/hailo/events",
    "/perception/observations",
    "/voice/stt/result",
    "/voice/stt/speaker",
)
#: Что пишется в бэг сцены (conductor.py): входы + сырой звук + контроль.
RECORD_TOPICS = INPUT_TOPICS + ("/audio/audio",) + CONTROL_TOPICS
#: Выходы прогона. camera_info — для пересчёта часов (ADR-0144 §5.3).
OUTPUT_TOPICS = (
    "/vision/hailo/events",
    "/voice/speaker/result",
    "/perception/observations",
    "/camera/camera/color/camera_info",
)

#: Переменные живого vision-face, которые переносятся в прогон 1.1 (пороги,
#: HEF, режим). Всё про граф и хранилище задаётся прогоном заново.
FACE_ENV_RE = re.compile(r"^(FACE_|HAILO_|HEF_|ARCFACE_|MIN_|MAX_|KEEP_|CONFIDENCE_|NMS_|DEPTH_|GAZE_)")


# ── конфиги Zenoh ────────────────────────────────────────────────────────────


def router_config() -> Dict:
    return {
        "mode": "router",
        "connect": {"endpoints": []},
        "listen": {"endpoints": [ROUTER_ENDPOINT]},
        "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": True}},
    }


def session_config(run_id: str) -> Dict:
    # shared_memory выключен: без хостового /dev/shm rmw_zenoh падает на
    # создании POSIX SHM (ENOMEM, проверено 29.09.2026); и даже там, где
    # /dev/shm хоста смонтирован (узел лица, ради HailoRT), данные прогона
    # не должны идти через общие с живыми узлами сегменты.
    return {
        "mode": "peer",
        "namespace": f"replay/{run_id}",
        "connect": {"endpoints": [ROUTER_ENDPOINT]},
        "listen": {"endpoints": ["tcp/127.0.0.1:0"]},
        "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": True}},
        "transport": {"shared_memory": {"enabled": False}},
    }


# ── план контейнеров ─────────────────────────────────────────────────────────


@dataclass
class Container:
    role: str
    image: str
    command: List[str]
    env: Dict[str, str] = field(default_factory=dict)
    volumes: List[Tuple[str, str, str]] = field(default_factory=list)  # (host, container, mode)
    devices: List[str] = field(default_factory=list)
    detach: bool = True

    def name(self, run_id: str) -> str:
        return f"{CONTAINER_PREFIX}{run_id}-{self.role}".lower()

    def argv(self, run_id: str) -> List[str]:
        out = ["docker", "run", "-d" if self.detach else "--rm", "--name", self.name(run_id), "--network", "host"]
        for dev in self.devices:
            out += ["--device", dev]
        for k, v in sorted(self.env.items()):
            out += ["-e", f"{k}={v}"]
        for host, cont, mode in self.volumes:
            out += ["-v", f"{host}:{cont}:{mode}"]
        return out + [self.image] + self.command


@dataclass
class Plan:
    run_id: str
    run_dir: str
    scene_bag: str
    router: Container
    nodes: List[Container]
    player: Container
    session: Dict
    router_cfg: Dict

    @property
    def all(self) -> List[Container]:
        return [self.router] + self.nodes + [self.player]


def ros_env(extra: Optional[Mapping[str, str]] = None) -> Dict[str, str]:
    env = {
        "ROS_DOMAIN_ID": str(REPLAY_DOMAIN_ID),
        "RMW_IMPLEMENTATION": "rmw_zenoh_cpp",
        "ZENOH_SESSION_CONFIG_URI": "/replay/zenoh_session.json5",
        "ZENOH_ROUTER_CHECK_ATTEMPTS": "10",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "PYTHONUNBUFFERED": "1",
    }
    env.update(extra or {})
    return env


def face_env(live_env: Sequence[str]) -> Dict[str, str]:
    """Пороги/HEF живого vision-face + хранилище и таймауты прогона поверх."""
    carried = {}
    for item in live_env:
        k, _, v = item.partition("=")
        if FACE_ENV_RE.match(k):
            carried[k] = v
    carried.update({
        "FACE_STORE_ROOT": "/replay/faces",
        # Узел ждёт первый кадр; проигрыватель стартует после готовности узлов.
        "FIRST_FRAME_TIMEOUT_SEC": "600",
        "HAILO_MODELS_YAML": "/config/hailo_models.yaml",
    })
    return ros_env(carried)


def _ros(cmd: str) -> List[str]:
    return ["bash", "-lc", f"source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && {cmd}"]


def speaker_cmd() -> List[str]:
    return _ros(
        "exec ros2 run rob_box_voice speaker_id_node --ros-args "
        "--params-file $(ros2 pkg prefix rob_box_voice)/share/rob_box_voice/config/speaker_id_node.yaml "
        "-p db_path:=/replay/voice/speakers.db "
        "-p memory_db_path:=/replay/voice/harness_voice.db "
        "-p voice_facts_db_path:=/replay/voice/voice_memory.db "
        "-p e2e_db_path:=/replay/voice/speakers.e2e.db "
        f"-p metrics_port:={REPLAY_METRICS_PORT}"
    )


def build_plan(
    run_id: str, run_dir: str, scene_bag: str, repo: str, scene_set_dir: str,
    face_image: str, voice_image: str, live_face_env: Sequence[str],
) -> Plan:
    vision = os.path.join(repo, "docker", "vision")
    replay_vol = (run_dir, "/replay", "rw")
    tools_vol = (scene_set_dir, "/scene_set", "ro")
    scene_vol = (scene_bag, "/scene/bag", "ro")
    face = Container(
        "face", face_image, ["/scripts/start_vision_face.sh"], face_env(live_face_env),
        [replay_vol, (os.path.join(vision, "config"), "/config", "ro"),
         (os.path.join(vision, "scripts", "vision-hailo"), "/scripts", "ro"),
         ("/opt/rob_box/models", "/opt/rob_box/models", "ro"),
         # /dev/shm хоста — как у живого vision-face (HailoRT ↔ hailort_service).
         # Графы через него не сходятся: Zenoh SHM в прогоне выключен.
         ("/dev/shm", "/dev/shm", "rw"),
         ("/tmp/hailort_uds.sock", "/tmp/hailort_uds.sock", "rw")],
        ["/dev/hailo0:/dev/hailo0"],
    )
    voice = Container("voice", voice_image, speaker_cmd(), ros_env(), [replay_vol])
    decoder = Container(
        "decoder", face_image, _ros("exec python3 /scene_set/decoder_node.py"), ros_env(), [replay_vol, tools_vol]
    )
    recorder = Container(
        "recorder", voice_image,
        _ros("exec ros2 bag record -s sqlite3 -o /replay/out_bag " + " ".join(OUTPUT_TOPICS)),
        ros_env(), [replay_vol],
    )
    player = Container(
        "player", voice_image, _ros("ros2 bag play /scene/bag --topics " + " ".join(INPUT_TOPICS)),
        ros_env(), [replay_vol, scene_vol], detach=False,
    )
    router = Container("router", ZENOH_IMAGE, ["-c", "/replay/zenoh_router.json5"], {}, [replay_vol])
    return Plan(run_id, run_dir, scene_bag, router, [face, voice, decoder, recorder], player,
                session_config(run_id), router_config())


# ── проверка изоляции (ADR-0144 §5.1) ────────────────────────────────────────


def _is_data_path(path: str) -> bool:
    parts = [p for p in os.path.normpath(path).replace("\\", "/").split("/") if p]
    return "data" in parts


def _check_zenoh(plan: Plan) -> List[str]:
    bad = []
    if plan.router_cfg["connect"]["endpoints"]:
        bad.append("router: connect не пуст — роутер прогона мостится наружу")
    if any("127.0.0.1" not in e for e in plan.router_cfg["listen"]["endpoints"]):
        bad.append("router: listen не только на 127.0.0.1")
    if plan.session["connect"]["endpoints"] != [ROUTER_ENDPOINT]:
        bad.append(f"session: connect {plan.session['connect']['endpoints']} != [{ROUTER_ENDPOINT}]")
    for name, cfg in (("router", plan.router_cfg), ("session", plan.session)):
        if cfg["scouting"]["multicast"]["enabled"]:
            bad.append(f"{name}: multicast scouting включён")
    if plan.session.get("transport", {}).get("shared_memory", {}).get("enabled", True):
        bad.append("session: Zenoh shared_memory не выключен")
    return bad


def _check_container(c: Container) -> List[str]:
    bad = []
    for host, cont, _mode in c.volumes:
        if _is_data_path(host) or _is_data_path(cont):
            bad.append(f"{c.role}: монтирование {host}:{cont} задевает data")
    if c.role != "router" and c.env.get("ROS_DOMAIN_ID") != str(REPLAY_DOMAIN_ID):
        bad.append(f"{c.role}: ROS_DOMAIN_ID={c.env.get('ROS_DOMAIN_ID')!r}")
    if c.role != "router" and c.env.get("ZENOH_SESSION_CONFIG_URI") != "/replay/zenoh_session.json5":
        bad.append(f"{c.role}: чужой ZENOH_SESSION_CONFIG_URI")
    if c.role == "face" and not c.env.get("FACE_STORE_ROOT", "").startswith("/replay/"):
        bad.append("face: FACE_STORE_ROOT не под /replay")
    joined = " ".join(c.command)
    if c.role == "voice" and ("/data/" in joined or f"metrics_port:={LIVE_METRICS_PORT}" in joined):
        bad.append("voice: БД или порт метрик боевые")
    if c.role == "player" and any(t in joined.split() for t in CONTROL_TOPICS):
        bad.append("player: проигрывает контрольный топик")
    return bad


def check_isolation(plan: Plan) -> List[str]:
    """Список нарушений; пусто — можно запускать."""
    bad = _check_zenoh(plan)
    for c in plan.all:
        bad += _check_container(c)
    if _is_data_path(plan.run_dir):
        bad.append(f"run_dir {plan.run_dir} задевает data")
    return bad


# ── исполнение (только на Vision Pi) ─────────────────────────────────────────


def sh(argv: Sequence[str], check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    print("+ " + " ".join(argv), flush=True)
    return subprocess.run(list(argv), check=check, text=True, capture_output=capture)


def _inspect(name: str, fmt: str) -> Optional[str]:
    r = subprocess.run(["docker", "inspect", "-f", fmt, name], text=True, capture_output=True)
    return r.stdout.strip() if r.returncode == 0 else None


def _wait_log(name: str, pattern: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        r = subprocess.run(["docker", "logs", name], text=True, capture_output=True)
        if pattern in (r.stdout + r.stderr):
            return True
        if _inspect(name, "{{.State.Running}}") != "true":
            return False
        time.sleep(2)
    return False


READY_PATTERNS = {
    "face": "subscribed to /camera/camera/color/image_raw",
    "voice": "speaker_id_node ready",
    "decoder": "decoder ready",
}


def prepare_run_dir(run_dir: str, seed: Optional[str], plan: Plan) -> None:
    os.makedirs(os.path.join(run_dir, "logs"), exist_ok=True)
    for sub in ("faces", "voice"):
        dst = os.path.join(run_dir, sub)
        if seed and os.path.isdir(os.path.join(seed, sub)):
            shutil.copytree(os.path.join(seed, sub), dst)
        else:
            os.makedirs(dst, exist_ok=True)
    for fname, cfg in (("zenoh_router.json5", plan.router_cfg), ("zenoh_session.json5", plan.session)):
        with open(os.path.join(run_dir, fname), "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)


def _start_nodes(plan: Plan, ready_timeout_s: float) -> None:
    sh(plan.router.argv(plan.run_id))
    time.sleep(2)
    for c in plan.nodes:
        sh(c.argv(plan.run_id))
    for c in plan.nodes:
        pattern = READY_PATTERNS.get(c.role)
        if pattern and not _wait_log(c.name(plan.run_id), pattern, ready_timeout_s):
            raise SystemExit(f"{c.role}: нет {pattern!r} за {ready_timeout_s}s — см. logs/{c.role}.log")
    time.sleep(3)  # recorder подписывается на выходы


def _collect_and_remove(plan: Plan) -> None:
    for c in [plan.router] + plan.nodes:
        name = c.name(plan.run_id)
        r = subprocess.run(["docker", "logs", name], text=True, capture_output=True)
        with open(os.path.join(plan.run_dir, "logs", f"{c.role}.log"), "w", encoding="utf-8") as f:
            f.write(r.stdout + r.stderr)
        subprocess.run(["docker", "rm", "-f", name], capture_output=True)


def _extract(plan: Plan, face_image: str, scene_set_dir: str) -> None:
    base = ["docker", "run", "--rm", "-v", f"{plan.run_dir}:/replay:rw", "-v", f"{scene_set_dir}:/scene_set:ro",
            "-v", f"{plan.scene_bag}:/scene/bag:ro", face_image]
    sh(base + _ros("python3 /scene_set/extract.py --bag /replay/out_bag --scene-bag /scene/bag "
                   "--mode replay --out /replay/journal.jsonl"))
    sh(base + _ros("python3 /scene_set/extract.py --bag /scene/bag --mode control "
                   "--out /replay/control_journal.jsonl"))


def _chown_back(run_dir: str, image: str) -> None:
    """Контейнеры пишут от root; без этого каталог прогона не удалить без sudo (ADR-0144 §4.3)."""
    sh(["docker", "run", "--rm", "-v", f"{run_dir}:/replay:rw", image,
        "chown", "-R", f"{os.getuid()}:{os.getgid()}", "/replay"], check=False)


def _restart_live_face() -> None:
    sh(["docker", "start", "vision-face"], check=False)
    time.sleep(20)
    r = subprocess.run(["docker", "logs", "--tail", "20", "vision-face"], text=True, capture_output=True)
    tail = r.stdout + r.stderr
    print(tail)
    if "STREAM_ABORT" in tail:
        print("!!! vision-face: STREAM_ABORT после прогона — нужен docker restart vision-face", flush=True)


def execute(plan: Plan, args: argparse.Namespace, face_image: str, scene_set_dir: str) -> None:
    live_running = _inspect("vision-face", "{{.State.Running}}") == "true"
    if live_running and not args.stop_live_face:
        raise SystemExit("живой vision-face запущен; прогон рядом с ним роняет Hailo — нужен --stop-live-face")
    if live_running:
        sh(["docker", "stop", "vision-face"])
    try:
        _start_nodes(plan, args.ready_timeout_s)
        sh(plan.player.argv(plan.run_id))
        time.sleep(args.tail_s)
        recorder = next(c for c in plan.nodes if c.role == "recorder")
        # ros2 bag record закрывает бэг по SIGINT; docker stop шлёт SIGTERM.
        sh(["docker", "kill", "-s", "INT", recorder.name(plan.run_id)], check=False)
        time.sleep(5)
    finally:
        _collect_and_remove(plan)
        if live_running:
            _restart_live_face()
    try:
        _extract(plan, face_image, scene_set_dir)
    finally:
        _chown_back(plan.run_dir, face_image)


def _save_seed(scenes_root: str, run_dir: str, seed_id: str) -> str:
    dst = os.path.join(scenes_root, "_seed", seed_id)
    for sub in ("faces", "voice"):
        shutil.copytree(os.path.join(run_dir, sub), os.path.join(dst, sub))
    return dst


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene-dir", required=True, help="~/scenes/<сцена> (внутри bag/ и scene.yaml)")
    ap.add_argument("--run-id", required=True)
    seed = ap.add_mutually_exclusive_group(required=True)
    seed.add_argument("--seed", help="~/scenes/_seed/<id> (faces/ + voice/)")
    seed.add_argument("--empty-seed", action="store_true", help="только для s00")
    ap.add_argument("--save-seed", help="после прогона сохранить хранилища как ~/scenes/_seed/<id>")
    ap.add_argument("--repo", default=os.path.expanduser("~/rob_box_project"))
    ap.add_argument("--face-image", help="по умолчанию — образ живого vision-face (1.1)")
    ap.add_argument("--voice-image", help="по умолчанию — образ живого voice-assistant (1.1)")
    ap.add_argument("--stop-live-face", action="store_true")
    ap.add_argument("--ready-timeout-s", type=float, default=180.0)
    ap.add_argument("--tail-s", type=float, default=10.0, help="сколько ждать выходов после конца бэга")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)

    scene_dir = os.path.abspath(os.path.expanduser(args.scene_dir))
    scene = os.path.basename(scene_dir.rstrip("/"))
    scenes_root = os.path.dirname(scene_dir)
    run_dir = os.path.join(scenes_root, "_replay", args.run_id, scene)
    scene_set_dir = os.path.dirname(os.path.abspath(__file__))
    face_image = args.face_image or _inspect("vision-face", "{{.Config.Image}}") or "<vision-face image?>"
    voice_image = args.voice_image or _inspect("voice-assistant", "{{.Config.Image}}") or "<voice-assistant image?>"
    live_env = json.loads(_inspect("vision-face", "{{json .Config.Env}}") or "[]")
    plan = build_plan(args.run_id, run_dir, os.path.join(scene_dir, "bag"), args.repo, scene_set_dir,
                      face_image, voice_image, live_env)

    violations = check_isolation(plan)
    for c in plan.all:
        print(" ".join(c.argv(plan.run_id)))
    if violations:
        print("ИЗОЛЯЦИЯ НАРУШЕНА — прогон не запускается:\n  " + "\n  ".join(violations))
        return 2
    print(f"isolation: OK (domain={REPLAY_DOMAIN_ID}, router={ROUTER_ENDPOINT}, no /data mounts)")
    if args.dry_run:
        return 0
    if os.path.exists(run_dir):
        raise SystemExit(f"{run_dir} уже есть — новый --run-id")
    prepare_run_dir(run_dir, None if args.empty_seed else os.path.expanduser(args.seed), plan)
    execute(plan, args, face_image, scene_set_dir)
    if args.save_seed:
        print(f"seed saved: {_save_seed(scenes_root, run_dir, args.save_seed)}")
    print(f"done: {run_dir}/journal.jsonl, control_journal.jsonl, logs/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
