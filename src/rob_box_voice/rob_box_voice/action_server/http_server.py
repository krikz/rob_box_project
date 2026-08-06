"""Standalone HTTP entry point for the action sidecar."""
from __future__ import annotations

import asyncio
import os
import signal

from .http import create_app
from .server import ActionServer


def _handler(goal, feedback, cancelled):
    """Safe default action: acknowledge planning; real executor is injected later."""
    feedback({"progress": 1.0, "goal_id": goal.get("goal_id")})
    return {"accepted": True, "goal": dict(goal)}


def _import_aiohttp():
    """Import aiohttp with a clear failure message.

    Issue #1004-class bug: if aiohttp isn't installed in the running image,
    a generic ``ModuleNotFoundError`` causes container restart loops. Replacing
    it with a sidecar-friendly hint makes the next operator debug session
    obvious (see E2E_TESTING_DESIGN_v2 §D A43).
    """
    try:
        from aiohttp import web  # type: ignore
    except ImportError as exc:  # pragma: no cover - defensive
        raise RuntimeError(
            "voice-action-server requires the 'aiohttp' Python package. "
            "It is declared in src/rob_box_voice/package.xml as <exec_depend> "
            "and pinned in docker/vision/voice_assistant/requirements.txt. "
            "If you see this on a freshly built image, the build cache was "
            "stale — bump BASE image or rebuild without cache."
        ) from exc
    return web


async def main() -> None:
    web = _import_aiohttp()
    server = ActionServer(_handler)
    app = create_app(server, web=web)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, os.getenv("ACTION_SERVER_HOST", "127.0.0.1"),
                       int(os.getenv("ACTION_SERVER_PORT", "8765")))
    await site.start()
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, stop_event.set)
    try:
        await stop_event.wait()
    finally:
        await server.shutdown(timeout=float(os.getenv("ACTION_SERVER_SHUTDOWN_TIMEOUT", "5.0")))
        await runner.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
