"""CLI агента: тик run, read-only status и C1 adoption/dry-run activation/rollback.

Конфиг — аргументы или переменные окружения CHUANGONG_* (systemd берёт их из /etc/chuangong/chuangong.env).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from pathlib import Path

from . import state as st


def _env(name: str, default: str) -> str:
    return os.environ.get(f"CHUANGONG_{name}", default)


def _runtime_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--robot-id", default=_env("ROBOT_ID", ""))
    p.add_argument("--node", default=_env("NODE", ""), choices=st.NODES)
    p.add_argument("--channel", default=_env("CHANNEL", "dev"))
    p.add_argument("--registry", default=_env("REGISTRY", "http://10.1.1.249:5000"))
    p.add_argument("--pubkey", default=_env("PUBKEY", "/etc/chuangong/release.pub"))
    p.add_argument("--state-dir", default=_env("STATE_DIR", "/var/lib/chuangong"))
    p.add_argument("--releases-root", default=_env("RELEASES_ROOT", "/opt/rob_box/releases"))
    p.add_argument("--legacy-root", default=_env("LEGACY_ROOT", "/home/ros2/rob_box_project"))
    p.add_argument("--zenoh", default=_env("ZENOH", "http://127.0.0.1:8000"))
    p.add_argument("--mode", default=_env("MODE", "observe"), choices=[m.value for m in st.Mode])
    p.add_argument("--dry-run", action="store_true", default=_env("DRY_RUN", "0") == "1")
    p.add_argument("--hil-lock", default=_env("HIL_LOCK", "/tmp/hil.lock"))
    p.add_argument("--reserve-gb", type=float, default=float(_env("RESERVE_GB", "2")))


def _run_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser("run", help="один тик: fetch → verify → [stage] → report")
    _runtime_args(p)


def _c1_parsers(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser("adopt", help="доказать current по legacy-файлам и image ID; по умолчанию dry-run")
    _runtime_args(p)
    p.add_argument("release")
    p.add_argument("--apply", action="store_true", help="записать только current/previous и adoption evidence")

    p = sub.add_parser("preflight", help="C1 dry-run: staged/signature, HIL, dirty, место и current")
    _runtime_args(p)
    p.add_argument("release")

    p = sub.add_parser("activate", help="C1 dry-run: preflight и план Vision → Sound → Main")
    _runtime_args(p)
    p.add_argument("release")

    p = sub.add_parser("rollback", help="C1 dry-run: проверить signed previous и построить план отката")
    _runtime_args(p)


def _status_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser("status", help="собрать состояния трёх Pi через zenoh REST (для F: Update Robot)")
    p.add_argument("--robot-id", required=True)
    p.add_argument("--zenoh", required=True, help="REST роутера Main Pi, например http://10.1.1.10:8000")
    p.add_argument("--release", default="", help="ожидаемый релиз (пусто — не сверять)")
    p.add_argument("--wait-s", type=float, default=360.0)
    p.add_argument("--summary", default="", help="файл для markdown-итога (GITHUB_STEP_SUMMARY)")


def _config(args: argparse.Namespace):
    from .agent import Config

    if not args.robot_id or not args.node:
        raise ValueError("нужны --robot-id и --node (или CHUANGONG_ROBOT_ID / CHUANGONG_NODE)")
    return Config(
        robot_id=args.robot_id,
        node=args.node,
        channel=args.channel,
        registry_url=args.registry,
        pubkey=Path(args.pubkey),
        state_dir=Path(args.state_dir),
        releases_root=Path(args.releases_root),
        legacy_root=Path(args.legacy_root),
        zenoh_url=args.zenoh,
        mode=st.Mode(args.mode),
        dry_run=args.dry_run,
        hil_lock=Path(args.hil_lock),
        reserve_bytes=int(args.reserve_gb * 1024**3),
    )


def _deps(cfg):
    from .agent import Deps
    from .registry import Registry
    from .stage import DockerCli
    from .zenoh_rest import ZenohRest

    return Deps(registry=Registry(cfg.registry_url), docker=DockerCli(), zenoh=ZenohRest(cfg.zenoh_url))


def cmd_run(args: argparse.Namespace) -> int:
    from .agent import tick

    try:
        cfg = _config(args)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    record = tick(cfg, _deps(cfg))
    print(json.dumps(record, ensure_ascii=False, sort_keys=True))
    # Ошибка fetch/stage — штатное состояние агента (оно в отчёте), а не падение юнита.
    return 0


def cmd_c1(args: argparse.Namespace) -> int:
    from .activation import ActivationMachine

    try:
        cfg = _config(args)
        machine = ActivationMachine(cfg, _deps(cfg))
        if args.cmd == "adopt":
            record = machine.adopt(args.release, apply=args.apply)
        elif args.cmd == "preflight":
            record = machine.preflight(args.release)
        elif args.cmd == "activate":
            record = machine.activate(args.release)
        else:
            record = machine.rollback()
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(record, ensure_ascii=False, sort_keys=True))
    return 1 if record["state"] in {s.value for s in st.ERROR_STATES} else 0


def cmd_status(args: argparse.Namespace) -> int:
    from .status import collect, render, verdict
    from .zenoh_rest import ZenohRest

    found, notes = collect(ZenohRest(args.zenoh), args.robot_id, args.wait_s)
    ok, problems = verdict(found, args.release or None)
    text = render(args.robot_id, found, ok, problems, notes)
    print(text)
    if args.summary:
        with open(args.summary, "a", encoding="utf-8") as fh:
            fh.write(text)
    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    parser = argparse.ArgumentParser(prog="chuangong")
    sub = parser.add_subparsers(dest="cmd", required=True)
    _run_parser(sub)
    _c1_parsers(sub)
    _status_parser(sub)
    args = parser.parse_args(argv)
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "status":
        return cmd_status(args)
    return cmd_c1(args)


if __name__ == "__main__":
    sys.exit(main())
