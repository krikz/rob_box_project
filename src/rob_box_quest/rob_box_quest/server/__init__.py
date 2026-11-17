"""aiohttp WSS server for rob_box_quest.

Phase 1.2: handshake (HELLO/WELCOME), PIN auth, SUBSCRIBE routing,
heartbeat send (200 ms), watchdog (close если нет ping > 600 ms).
"""

from .session import ClientSession, SessionState
from .ws_server import WSSServer, build_app

__all__ = ["ClientSession", "SessionState", "WSSServer", "build_app"]
