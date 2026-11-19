"""CLI агента: тик run, локальный cancel, validate-inputs пульта, status и C1 dry-run.

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

    p = sub.add_parser("activate", help="план Vision → Sound → Main; --execute зовёт исполнителя роли")
    _runtime_args(p)
    p.add_argument("release")
    p.add_argument("--execute", action="store_true", help="исполнить план (по умолчанию только план, execute=false)")

    p = sub.add_parser("rollback", help="план отката на signed previous; --execute зовёт исполнителя роли")
    _runtime_args(p)
    p.add_argument("--execute", action="store_true", help="исполнить план отката (по умолчанию только план)")


def _cancel_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser("cancel", help="локально отменить staged; current/previous не трогать")
    p.add_argument("--node", default=_env("NODE", ""), choices=st.NODES)
    p.add_argument("--state-dir", default=_env("STATE_DIR", "/var/lib/chuangong"))
    p.add_argument("--releases-root", default=_env("RELEASES_ROOT", "/opt/rob_box/releases"))


def _status_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser("status", help="собрать состояния трёх Pi через zenoh REST (для F: Update Robot)")
    p.add_argument("--robot-id", required=True)
    p.add_argument("--zenoh", required=True, help="REST роутера Main Pi, например http://10.1.1.10:8000")
    p.add_argument("--release", default="", help="ожидаемый релиз (пусто — не сверять)")
    p.add_argument("--wait-s", type=float, default=360.0)
    p.add_argument("--summary", default="", help="файл для markdown-итога (GITHUB_STEP_SUMMARY)")


def _validate_inputs_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser(
        "validate-inputs",
        help="F: Update Robot: отказать при пустом robot_id или release (update/rollback)",
    )
    p.add_argument("--action", required=True, help="status|update|rollback|cancel")
    p.add_argument("--robot-id", default="")
    p.add_argument("--release", default="")
    p.add_argument("--operation-id", default="", help="общий id команды и отчёта (github.run_id)")
    p.add_argument("--github-output", default="", help="GITHUB_OUTPUT: operation_id=...")
    p.add_argument("--summary", default="", help="GITHUB_STEP_SUMMARY")


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
        from .role_switch import execute_role

        # Исполнитель роли подключён (#4911), но пульт зовёт его только с проверенным
        # адаптером Compose. Без подтверждения --execute останавливается до машины,
        # не доходя до compose_down (Шифу, #4375).
        verified = os.environ.get("CHUANGONG_COMPOSE_ADAPTER_VERIFIED") == "1"
        machine = ActivationMachine(cfg, _deps(cfg), executor=execute_role if verified else None)
        if args.cmd == "adopt":
            record = machine.adopt(args.release, apply=args.apply)
        elif args.cmd == "preflight":
            record = machine.preflight(args.release)
        elif args.cmd == "activate":
            record = machine.activate(args.release, execute=args.execute)
        else:
            record = machine.rollback(execute=args.execute)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(record, ensure_ascii=False, sort_keys=True))
    return 1 if record["state"] in {s.value for s in st.ERROR_STATES} else 0


def cmd_cancel(args: argparse.Namespace) -> int:
    """Локальная отмена staged: сеть и compose не трогаются."""
    from . import cancel

    if not args.node:
        print("нужен --node (или CHUANGONG_NODE)", file=sys.stderr)
        return 2
    try:
        record = cancel.cancel_staged(Path(args.state_dir), Path(args.releases_root), args.node)
    except cancel.CancelError as exc:
        print(
            json.dumps(
                {"state": "error", "reason": exc.reason, "detail": exc.detail},
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 1
    print(json.dumps(record, ensure_ascii=False, sort_keys=True))
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


def cmd_validate_inputs(args: argparse.Namespace) -> int:
    """Пульт F: Update Robot — проверка входов; update/rollback/cancel → not_wired (exit 2)."""
    from .fleet_inputs import FleetInputError, render_error_summary, render_summary, validate_inputs

    try:
        record = validate_inputs(
            action=args.action,
            robot_id=args.robot_id,
            release=args.release,
            operation_id=args.operation_id,
        )
    except FleetInputError as exc:
        print(json.dumps({"ok": False, "reason": exc.reason, "detail": exc.detail}, ensure_ascii=False, sort_keys=True))
        if args.summary:
            with open(args.summary, "a", encoding="utf-8") as fh:
                fh.write(
                    render_error_summary(
                        reason=exc.reason,
                        detail=exc.detail,
                        action=(args.action or "").strip(),
                        robot_id=(args.robot_id or "").strip(),
                        release=(args.release or "").strip(),
                        operation_id=(args.operation_id or "").strip(),
                    )
                )
        return 2
    print(json.dumps({"ok": True, **record}, ensure_ascii=False, sort_keys=True))
    if args.github_output:
        with open(args.github_output, "a", encoding="utf-8") as fh:
            op = record.get("operation_id") or ""
            fh.write(f"operation_id={op}\n")
            fh.write(f"action={record['action']}\n")
            fh.write(f"robot_id={record['robot_id']}\n")
            rel = record.get("release") or ""
            fh.write(f"release={rel}\n")
    if args.summary:
        with open(args.summary, "a", encoding="utf-8") as fh:
            fh.write(render_summary(record))
    return 0


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    parser = argparse.ArgumentParser(prog="chuangong")
    sub = parser.add_subparsers(dest="cmd", required=True)
    _run_parser(sub)
    _cancel_parser(sub)
    _c1_parsers(sub)
    _status_parser(sub)
    _validate_inputs_parser(sub)
    args = parser.parse_args(argv)
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "cancel":
        return cmd_cancel(args)
    if args.cmd == "status":
        return cmd_status(args)
    if args.cmd == "validate-inputs":
        return cmd_validate_inputs(args)
    return cmd_c1(args)


if __name__ == "__main__":
    sys.exit(main())
