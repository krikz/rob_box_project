"""Polling lock and Conflict recovery for the Telegram bot (issue #5061).

Telegram allows exactly one ``getUpdates`` consumer per bot token. A second
consumer (a check that constructed ``TelegramNode()`` inside the live
container, or a second process with the same token) makes Telegram answer the
first with ``409 Conflict: terminated by other getUpdates request``.
``python-telegram-bot`` catches that inside its own polling loop, logs it and
keeps calling ``getUpdates`` on a dead session — the exception never reaches
the node's retry loop, so the bot looks healthy while it accepts nothing.

Two decisions live here, not in a prompt (ADR-0148):

* a file lock held for the whole polling lifetime — the second instance in the
  same container fails before it opens a network connection;
* a watchdog over the updater task — a swallowed ``Conflict`` is logged in
  plain words and polling is restarted with backoff until it holds.
"""

from __future__ import annotations

import asyncio
import fcntl
import os
from typing import Optional, TextIO

# One lock per container: both the live bot and a ``docker exec`` check share
# the container filesystem, so the second one sees the lock the first holds.
POLLING_LOCK_PATH = os.environ.get("TELEGRAM_POLLING_LOCK", "/tmp/rob_box_telegram_polling.lock")

# Backoff after a Conflict that the library swallowed. Capped: a foreign
# consumer may hold the token for a long time, and we must not hammer
# getUpdates. Unbounded attempts — the channel must come back on its own once
# the other consumer is gone; a finite cap would leave it dead again.
CONFLICT_BACKOFF_START_S = 5.0
CONFLICT_BACKOFF_CAP_S = 60.0

CONFLICT_LOG = (
    "Telegram Conflict (409): another getUpdates is running on this bot token, "
    "so this polling is dead and incoming messages are dropped. "
    "Retrying with backoff until polling holds. Do not start a second "
    "TelegramNode in this container — it takes the token and kills the live bot."
)

LOCK_HELD_LOG = (
    "Telegram polling lock is already held ({path}): another instance in this "
    "container is the bot. This process will NOT call getUpdates. Check the "
    "face owner with `ros2 service call /perception/people ...` from "
    "avatar-supervisor, never by constructing TelegramNode()."
)


class PollingLockHeld(RuntimeError):
    """Another instance in this container already owns the polling lock."""

    def __init__(self, path: str) -> None:
        super().__init__(LOCK_HELD_LOG.format(path=path))
        self.path = path


class PollingLock:
    """Exclusive ``flock`` held for the lifetime of one polling session.

    The descriptor stays open: the kernel drops the lock when the process
    exits, including a crash, so a restarted container is never stuck behind
    its own stale lock.
    """

    def __init__(self, path: str = POLLING_LOCK_PATH) -> None:
        self.path = path
        self._fh: Optional[TextIO] = None

    def acquire(self) -> None:
        fh = open(self.path, "a+")  # noqa: SIM115 — closed in release(), or by process exit
        try:
            fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            fh.close()
            raise PollingLockHeld(self.path) from exc
        fh.seek(0)
        fh.truncate()
        fh.write(f"{os.getpid()}\n")
        fh.flush()
        self._fh = fh

    def release(self) -> None:
        fh, self._fh = self._fh, None
        if fh is None:
            return
        try:
            fcntl.flock(fh.fileno(), fcntl.LOCK_UN)
        finally:
            fh.close()

    def __enter__(self) -> "PollingLock":
        self.acquire()
        return self

    def __exit__(self, *_exc) -> None:
        self.release()


def is_conflict(exc: BaseException) -> bool:
    """``telegram.error.Conflict``, including one wrapped by the library."""
    seen = set()
    current: Optional[BaseException] = exc
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        if type(current).__name__ == "Conflict":
            return True
        current = current.__cause__ or current.__context__
    return False


def conflict_backoff_s(attempt: int) -> float:
    """5, 10, 20, 40, then 60 seconds — ``attempt`` counts from 1."""
    growth = CONFLICT_BACKOFF_START_S * (2 ** max(0, attempt - 1))
    return min(growth, CONFLICT_BACKOFF_CAP_S)


async def watch_polling(updater_task: "asyncio.Task[object]", poll_s: float = 1.0) -> bool:
    """``True`` once the updater task has ended, ``False`` after one quiet tick.

    ``asyncio.wait`` (not ``wait_for``): a timeout must not cancel the polling
    task it is only observing. The caller loops this and leaves as soon as the
    library's polling task dies, so a swallowed ``Conflict`` cannot leave the
    channel mute.
    """
    done, _pending = await asyncio.wait({updater_task}, timeout=poll_s)
    return updater_task in done
