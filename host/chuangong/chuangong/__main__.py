"""CLI агента: `python3 -m chuangong run` (тик, его зовёт systemd) и `python3 -m chuangong status` (наблюдатель).

Конфиг тика — аргументы или переменные окружения CHUANGONG_* (systemd берёт их из /etc/chuangong/chuangong.env).
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


def _run_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser("run", help="один тик: fetch → verify → [stage] → report")
    p.add_argument("--robot-id", default=_env("ROBOT_ID", ""))
    p.add_argument("--node", default=_env("NODE", ""), choices=st.NODES)
    p.add_argument("--channel", default=_env("CHANNEL", "dev"))
    p.add_argument("--registry", default=_env("REGISTRY", "http://10.1.1.249:5000"))
    p.add_argument("--pubkey", default=_env("PUBKEY", "/etc/chuangong/release.pub"))
    p.add_argument("--state-dir", default=_env("STATE_DIR", "/var/lib/chuangong"))
    p.add_argument("--releases-root", default=_env("RELEASES_ROOT", "/opt/rob_box/releases"))
    p.add_argument("--zenoh", default=_env("ZENOH", "http://127.0.0.1:8000"))
    p.add_argument("--mode", default=_env("MODE", "observe"), choices=[m.value for m in st.Mode])
    p.add_argument("--dry-run", action="store_true", default=_env("DRY_RUN", "0") == "1")
    p.add_argument("--hil-lock", default=_env("HIL_LOCK", "/tmp/hil.lock"))
    p.add_argument("--reserve-gb", type=float, default=float(_env("RESERVE_GB", "2")))


def _status_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser("status", help="собрать состояния трёх Pi через zenoh REST (для F: Update Robot)")
    p.add_argument("--robot-id", required=True)
    p.add_argument("--zenoh", required=True, help="REST роутера Main Pi, например http://10.1.1.10:8000")
    p.add_argument("--release", default="", help="ожидаемый релиз (пусто — не сверять)")
    p.add_argument("--wait-s", type=float, default=360.0)
    p.add_argument("--summary", default="", help="файл для markdown-итога (GITHUB_STEP_SUMMARY)")


def cmd_run(args: argparse.Namespace) -> int:
    from .agent import Config, Deps, tick
    from .registry import Registry
    from .stage import DockerCli
    from .zenoh_rest import ZenohRest

    if not args.robot_id or not args.node:
        print("нужны --robot-id и --node (или CHUANGONG_ROBOT_ID / CHUANGONG_NODE)", file=sys.stderr)
        return 2
    cfg = Config(
        robot_id=args.robot_id,
        node=args.node,
        channel=args.channel,
        registry_url=args.registry,
        pubkey=Path(args.pubkey),
        state_dir=Path(args.state_dir),
        releases_root=Path(args.releases_root),
        zenoh_url=args.zenoh,
        mode=st.Mode(args.mode),
        dry_run=args.dry_run,
        hil_lock=Path(args.hil_lock),
        reserve_bytes=int(args.reserve_gb * 1024**3),
    )
    deps = Deps(registry=Registry(cfg.registry_url), docker=DockerCli(), zenoh=ZenohRest(cfg.zenoh_url))
    record = tick(cfg, deps)
    print(json.dumps(record, ensure_ascii=False, sort_keys=True))
    # Ошибка fetch/stage — штатное состояние агента (оно в отчёте), а не падение юнита.
    return 0


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
    _status_parser(sub)
    args = parser.parse_args(argv)
    return cmd_run(args) if args.cmd == "run" else cmd_status(args)


if __name__ == "__main__":
    sys.exit(main())
