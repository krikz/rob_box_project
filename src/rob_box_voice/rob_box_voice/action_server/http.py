"""HTTP adapter exposing the action protocol for PASTE/sidecar clients.

Endpoints:
  GET  /healthz
  POST /actions        {"goal": {...}}
  GET  /actions/{id}   status + feedback/result
  POST /actions/{id}/cancel

The module is optional at runtime; ROS deployments can use ActionServer
without installing aiohttp.
"""
from __future__ import annotations

from typing import Any

from .server import ActionServer


def create_app(server: ActionServer, *, web: Any = None) -> Any:
    if web is None:
        try:
            from aiohttp import web
        except ImportError as exc:
            raise RuntimeError("aiohttp is required for the HTTP action adapter") from exc
    app = web.Application()

    async def health(_request):
        health = server.health()
        return web.json_response({"ok": health.ok, "active_goals": health.active_goals,
                                  "shutting_down": health.shutting_down},
                                 status=200 if health.ok else 503)

    async def submit(request):
        payload = await request.json()
        handle = server.submit(payload.get("goal", payload))
        return web.json_response({"goal_id": handle.goal_id, "state": handle.state.value}, status=202)

    async def status(request):
        handle = server._handles.get(request.match_info["goal_id"])
        if handle is None:
            raise web.HTTPNotFound()
        return web.json_response({"goal_id": handle.goal_id, "state": handle.state.value,
                                  "feedback": handle.feedback, "result": handle.result,
                                  "error": handle.error})

    async def cancel(request):
        if not server.cancel(request.match_info["goal_id"]):
            raise web.HTTPNotFound()
        return web.json_response({"cancelled": True})

    app.router.add_get("/healthz", health)
    app.router.add_post("/actions", submit)
    app.router.add_get("/actions/{goal_id}", status)
    app.router.add_post("/actions/{goal_id}/cancel", cancel)
    return app
