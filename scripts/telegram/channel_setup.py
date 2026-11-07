#!/usr/bin/env python3
"""Оформление канала @robbox_project: аватар, название, описание, первый пост и его закреп.

ТОЛЬКО после «да» Шифу: запуск без --dry-run требует флага --shifu-approved, а пост 001 в очереди должен быть
в статусе `approved`. Шаги: setChatPhoto, setChatTitle, setChatDescription, публикация поста 001, pinChatMessage.
Тексты берутся из docs/channel/profile.md и docs/channel/queue/001-*.md.

    python -X utf8 scripts/telegram/channel_setup.py --dry-run
    python -X utf8 scripts/telegram/channel_setup.py --shifu-approved
"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))
import channel_lib as lib  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
TITLE_LIMIT, DESCRIPTION_LIMIT = 128, 255


def read_profile(repo_root: Path) -> dict:
    text = (repo_root / "docs" / "channel" / "profile.md").read_text(encoding="utf-8").replace("\r\n", "\n")
    meta = {}
    for line in text[4:].partition("\n---\n")[0].splitlines():
        key, _, value = line.partition(":")
        meta[key.strip()] = value.strip()
    return meta


def plan(repo_root: Path) -> list:
    """Шаги оформления: список (описание, метод, параметры, файл)."""
    profile = read_profile(repo_root)
    cid = lib.CHANNEL_ID
    if len(profile["title"]) > TITLE_LIMIT or len(profile["description"]) > DESCRIPTION_LIMIT:
        raise SystemExit(
            f"название ≤{TITLE_LIMIT}, описание ≤{DESCRIPTION_LIMIT} знаков: "
            f"{len(profile['title'])}/{len(profile['description'])}"
        )
    pin = lib.parse_post(repo_root / profile["pinned"])
    problems = lib.validate(pin, repo_root)
    if problems:
        raise SystemExit(f"закреп {pin.name}: " + "; ".join(problems))
    method, params, media = lib.build_request(pin, repo_root)
    return [
        ("аватар", "setChatPhoto", {"chat_id": cid}, repo_root / profile["avatar"]),
        ("название", "setChatTitle", {"chat_id": cid, "title": profile["title"]}, None),
        ("описание", "setChatDescription", {"chat_id": cid, "description": profile["description"]}, None),
        ("первый пост", method, params, media),
        ("закреп", "pinChatMessage", {"chat_id": cid, "disable_notification": True}, None),
    ]


def run(repo_root: Path, transport: Optional[lib.Transport], dry_run: bool, out=print) -> int:
    steps = plan(repo_root)
    pin_path = repo_root / read_profile(repo_root)["pinned"]
    if not dry_run and lib.parse_post(pin_path).status != "approved":
        out(f"{pin_path.name} не approved: сначала одобрение Шифу")
        return 1
    message_id = None
    for title, method, params, media in steps:
        if method == "pinChatMessage" and message_id is not None:
            params = dict(params, message_id=message_id)
        out(f"{'DRY-RUN' if dry_run else 'ШАГ'} [{title}]")
        out(lib.describe(method, params, media))
        if dry_run:
            continue
        result = transport(method, params, media)
        if not result.get("ok"):
            out(f"Остановился на шаге «{title}»: {result.get('description')}")
            return 1
        if title == "первый пост":
            message_id = result["message_id"]
            lib.set_front_matter(
                pin_path, status="posted", message_id=message_id, posted_at=datetime.now(timezone.utc).isoformat()
            )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--shifu-approved", action="store_true", help="Шифу сказал «да» на оформление и закреп")
    ap.add_argument("--robot", default="ros2@10.1.1.22")
    ap.add_argument(
        "--transport",
        choices=("robot", "direct"),
        default="robot",
        help="robot — через контейнер telegram-bot на Sound Pi (по умолчанию); direct — через токен студии "
        "(TG_STUDIO_BOT_TOKEN)",
    )
    args = ap.parse_args()
    if not args.dry_run and not args.shifu_approved:
        print("Боевой запуск только с --shifu-approved (одобрение Шифу). Для проверки — --dry-run.")
        return 2
    return run(REPO, None if args.dry_run else lib.make_transport(args.transport, args.robot), args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
