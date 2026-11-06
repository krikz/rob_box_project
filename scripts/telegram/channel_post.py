#!/usr/bin/env python3
"""Планировщик канала @robbox_project: публикует следующий одобренный пост из docs/channel/queue/.

Берёт первый по номеру пост со статусом `approved`, у которого `not_before` наступил, если с прошлой
публикации прошло не меньше 48 часов. Публикует через бота в контейнере telegram-bot на Sound Pi и
записывает в front matter `status: posted`, `message_id`, `posted_at`. Правки файла коммитит координатор.

    python -X utf8 scripts/telegram/channel_post.py --dry-run
    python -X utf8 scripts/telegram/channel_post.py --preview-all     # показать все посты вслух, ничего не слать
    python -X utf8 scripts/telegram/channel_post.py                   # боевой режим
    python -X utf8 scripts/telegram/channel_post.py --send-draft 3    # заготовка поста 003 в личку Шифу

Правила — docs/process/telegram-channel.md. Токен остаётся в контейнере.
"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))
import channel_lib as lib  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


def run(
    queue_dir: Path,
    repo_root: Path,
    now: datetime,
    transport: Optional[lib.Transport],
    dry_run: bool,
    out=print,
) -> int:
    posts = lib.load_queue(queue_dir)
    post = lib.pick_next(posts, now)
    if post is None:
        last = lib.last_posted_at(posts)
        out(
            "Публиковать нечего: нет approved-поста с наступившим not_before"
            + (f"; последняя публикация {last.isoformat()}, нужно ≥ 48 ч между постами" if last else "")
        )
        return 0
    problems = lib.validate(post, repo_root)
    if problems:
        out(f"{post.name}: не публикую, проблемы:")
        for p in problems:
            out(f"  - {p}")
        return 1
    method, params, media = lib.build_request(post, repo_root)
    out(f"{'DRY-RUN: было бы отправлено' if dry_run else 'Отправляю'} {post.name}")
    out(lib.describe(method, params, media))
    extra = lib.build_extra_request(post, repo_root)
    if extra is not None:
        out("  затем отдельным сообщением без подписи:")
        out(lib.describe(*extra))
    if dry_run:
        return 0
    result = transport(method, params, media)
    if not result.get("ok") or not result.get("message_id"):
        out(f"Telegram не принял пост: {result.get('description')}")
        return 1
    lib.set_front_matter(post.path, status="posted", message_id=result["message_id"], posted_at=now.isoformat())
    extra = lib.build_extra_request(post, repo_root)
    if extra is not None:
        extra_result = transport(*extra)
        if not extra_result.get("ok"):
            out(f"ВНИМАНИЕ: основной пост ушёл, а картинка к нему нет: {extra_result.get('description')}")
    out(f"Опубликовано, message_id={result['message_id']}; {post.name} помечен posted (закоммить правку)")
    return 0


def preview_all(queue_dir: Path, repo_root: Path, out=print) -> int:
    bad = 0
    for post in lib.load_queue(queue_dir):
        out(f"=== {post.name} [{post.status}] не раньше {post.not_before.isoformat()}")
        method, params, media = lib.build_request(post, repo_root)
        out(lib.describe(method, params, media))
        extra = lib.build_extra_request(post, repo_root)
        if extra is not None:
            out("  затем отдельным сообщением без подписи:")
            out(lib.describe(*extra))
        problems = lib.validate(post, repo_root)
        for p in problems:
            out(f"  !! {p}")
        bad += bool(problems)
        out("")
    out(f"Постов с проблемами: {bad}")
    return 1 if bad else 0


def send_draft(
    queue_dir: Path, repo_root: Path, number: int, transport: Optional[lib.Transport], dry_run: bool, out=print
) -> int:
    """Путь 2: готовая заготовка поста в личку Шифу. Он пересылает её в канал и ставит время в приложении.

    В Bot API нет отложенной отправки (проверено по документации 06.11.2026, версия 10.3), а userbot через аккаунт
    Шифу запрещён. Сообщение-заготовка — тот же текст в той же разметке и тот же файл, что ушли бы в канал.
    """
    matches = [p for p in lib.load_queue(queue_dir) if p.name.startswith(f"{number:03d}-")]
    if not matches:
        out(f"нет поста с номером {number:03d}")
        return 1
    post = matches[0]
    problems = lib.validate(post, repo_root)
    if problems:
        out(f"{post.name}: заготовку не шлю, проблемы:")
        for p in problems:
            out(f"  - {p}")
        return 1
    slot = post.not_before.astimezone(timezone(timedelta(hours=1)))
    note = (
        f"Заготовка {post.name}. Переслать в канал @robbox_project со скрытым именем отправителя "
        f"(«Скрыть имя автора»), затем долгий тап по кнопке отправки → «Отправить позже» на "
        f"{slot:%d.%m.%Y %H:%M} (GMT+1). Когда поставишь время, скажи координатору: он пометит пост posted."
    )
    steps = [lib.build_request(post, repo_root, chat_id=lib.DRAFT_CHAT_ID)]
    extra = lib.build_extra_request(post, repo_root, chat_id=lib.DRAFT_CHAT_ID)
    if extra is not None:
        steps.append(extra)
    steps.append(("sendMessage", {"chat_id": lib.DRAFT_CHAT_ID, "text": note}, None))
    for method, params, media in steps:
        out(f"{'DRY-RUN: было бы отправлено Шифу' if dry_run else 'Шлю Шифу'}")
        out(lib.describe(method, params, media))
        if dry_run:
            continue
        result = transport(method, params, media)
        if not result.get("ok"):
            out(f"Telegram не принял заготовку: {result.get('description')}")
            return 1
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="напечатать, что было бы отправлено, и ничего не слать")
    ap.add_argument("--preview-all", action="store_true", help="показать все посты очереди (dry-run, любой статус)")
    ap.add_argument(
        "--send-draft", type=int, metavar="N", help="прислать Шифу в личку заготовку поста N (путь без планировщика)"
    )
    ap.add_argument("--queue", type=Path, default=REPO / "docs" / "channel" / "queue")
    ap.add_argument("--now", help="подменить текущее время (ISO, UTC) — для проверки расписания")
    ap.add_argument("--robot", default="ros2@10.1.1.22")
    args = ap.parse_args()
    if args.send_draft is not None:
        transport = None if args.dry_run else lib.RobotTransport(args.robot)
        return send_draft(args.queue, REPO, args.send_draft, transport, args.dry_run)
    if args.preview_all:
        return preview_all(args.queue, REPO)
    now = datetime.fromisoformat(args.now.replace("Z", "+00:00")) if args.now else datetime.now(timezone.utc)
    transport = None if args.dry_run else lib.RobotTransport(args.robot)
    return run(args.queue, REPO, now, transport, args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
