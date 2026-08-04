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


async def main() -> None:
    from aiohttp import web
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
