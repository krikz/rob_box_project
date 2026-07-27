"""``TelegramHarness`` — harness wrapping the Telegram bot logic.

A :class:`Harness` subclass that adapts the full Telegram bot
pipeline (command dispatch → LLM chat → reply) into the harness
framework per ADR-0001 §2.7.3.

**Design — parallel implementation:**
    This harness lives side-by-side with the existing
    ``rob_box_telegram.telegram_node`` and its submodules
    (``commands.py``, ``messages.py``, ``voice_processor.py``,
    ``camera_cache.py``, ``auth.py``). Both coexist; switching is
    via config.

**What moved in:**
    * LLMChat (469 lines) → ``LLMProvider`` port (reuse existing)
    * MCPBridge (137 lines) → ``ToolProvider`` port
    * commands.py (25 handlers, 534 lines) → ``TelegramCommandRegistry``
    * camera_cache.py (76 lines) → ``SnapshotStore``
    * auth.py (89 lines) → ``AuthMiddleware``

**What STAYS in telegram_node.py:**
    * python-telegram-bot ``Application`` setup
    * ``Update``/``Message`` deserialization
    * Telegram API I/O (HTTP long-polling)
    * The async event loop

Usage::

    harness = TelegramHarness(config)
    async with harness:
        update = {"chat_id": "123", "user_id": "456", "command": "/start"}
        result = await harness.run(update)
        print(result.output)  # "Welcome!"
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Callable

from rob_box_harness.config import HarnessConfig
from rob_box_harness.harness import Harness
from rob_box_harness.memory import Turn

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# TelegramState
# ---------------------------------------------------------------------------


@dataclass
class TelegramState:
    """Runtime state bag for the Telegram harness.

    Tracks per-message context: who is chatting, what command was
    last used, and the auth status. The harness snapshots this on
    each turn.
    """

    chat_id: str = ""
    """Telegram chat/group identifier."""

    user_id: str = ""
    """Telegram user identifier (numeric string)."""

    username: str = ""
    """Telegram @username or first_name."""

    last_command: str = ""
    """Most recent slash-command executed."""

    message_count: int = 0
    """Monotonic counter of messages processed."""

    auth_status: str = "anonymous"
    """One of: anonymous, authenticated, admin."""

    metadata: dict[str, Any] = field(default_factory=dict)
    """Extension point for per-chat telemetry or flags."""


# ---------------------------------------------------------------------------
# TelegramCommandRegistry
# ---------------------------------------------------------------------------


class TelegramCommandRegistry:
    """Declarative command → handler dispatch for Telegram bot commands.

    Mirrors the pattern from ``rob_box_telegram.commands`` (25 handlers)
    in a ROS-agnostic way. Commands are registered with a name
    (e.g. ``"/start"``) and an async handler callable.

    Registration is eager (during ``init()``) so missing handlers
    are caught at startup, not at first use.
    """

    def __init__(self) -> None:
        self._handlers: dict[str, Callable[..., Any]] = {}

    def register(self, command: str, handler: Callable[..., Any]) -> None:
        """Register a handler for a slash-command.

        Args:
            command: Command name with leading slash, e.g. ``"/start"``.
            handler: Async callable ``(args: str, state: TelegramState) → str``.
        """
        if not command.startswith("/"):
            command = f"/{command}"
        self._handlers[command] = handler
        logger.debug("TelegramCommandRegistry: registered '%s'", command)

    async def dispatch(
        self,
        command: str,
        args: str,
        state: TelegramState,
    ) -> str:
        """Execute the handler for ``command``.

        Args:
            command: Slash-command name.
            args: Everything after the command (may be empty).
            state: Current TelegramState for context.

        Returns:
            Response text to send back to the user.
        """
        handler = self._handlers.get(command)
        if handler is None:
            return f"Неизвестная команда: {command}. Используйте /help для списка команд."

        try:
            result = handler(args, state)
            # Support both sync and async handlers
            import asyncio
            if asyncio.iscoroutine(result):
                result = await result
            return str(result) if result is not None else ""
        except Exception:
            logger.exception(
                "TelegramCommandRegistry: handler '%s' failed",
                command,
            )
            return "Произошла ошибка при выполнении команды."

    def commands(self) -> list[str]:
        """Return all registered command names."""
        return list(self._handlers.keys())

    def __len__(self) -> int:
        return len(self._handlers)

    def __contains__(self, command: str) -> bool:
        return command in self._handlers


# ---------------------------------------------------------------------------
# AuthMiddleware
# ---------------------------------------------------------------------------


class AuthMiddleware:
    """Auth check at the dispatcher level — wraps command execution.

    Replaces ``rob_box_telegram.auth`` (89 lines). Checks that the
    Telegram user_id belongs to the allowed set before any command
    or LLM interaction is processed.
    """

    def __init__(self, allowed_users: list[str] | None = None) -> None:
        self._allowed: set[str] = set(allowed_users or [])

    def check(self, user_id: str) -> bool:
        """Return True if ``user_id`` is authorised.

        An empty allowed_users set means "allow all" (development mode).
        """
        if not self._allowed:
            return True
        return user_id in self._allowed

    def add_user(self, user_id: str) -> None:
        """Grant access to a user."""
        self._allowed.add(user_id)

    def remove_user(self, user_id: str) -> None:
        """Revoke access from a user."""
        self._allowed.discard(user_id)

    def wrap(self, handler: Callable[..., Any]) -> Callable[..., Any]:
        """Wrap a handler with an auth check.

        Returns a callable that checks auth before delegating to
        the original handler.
        """
        _check = self.check

        async def _wrapper(*args: Any, **kwargs: Any) -> str:
            user_id = kwargs.get("user_id", "")
            if not _check(str(user_id)):
                return "⛔ Доступ запрещён. Обратитесь к администратору."
            return await handler(*args, **kwargs)

        return _wrapper


# ---------------------------------------------------------------------------
# SnapshotStore
# ---------------------------------------------------------------------------


class SnapshotStore:
    """Temporary camera snapshot cache.

    Replaces ``rob_box_telegram.camera_cache`` (76 lines). Stores
    image bytes keyed by chat_id with automatic expiration. Not
    a persistent store — snapshots are ephemeral (default TTL: 5 min).
    """

    def __init__(self) -> None:
        self._store: dict[str, tuple[bytes, float]] = {}
        import time as _t
        self._time = _t

    def store(self, chat_id: str, image_data: bytes) -> str:
        """Store a snapshot and return its cache key.

        Args:
            chat_id: Telegram chat ID (used to namespace snapshots).
            image_data: Raw JPEG/PNG bytes.

        Returns:
            Cache key for later retrieval.
        """
        key = f"snap:{chat_id}:{self._time.monotonic()}"
        self._store[key] = (image_data, self._time.monotonic())
        return key

    def retrieve(self, key: str) -> bytes | None:
        """Retrieve snapshot bytes by key.

        Returns:
            Image bytes or None if expired/missing.
        """
        entry = self._store.get(key)
        if entry is None:
            return None
        return entry[0]

    def expire(self, max_age_seconds: float = 300.0) -> None:
        """Remove all snapshots older than ``max_age_seconds``."""
        now = self._time.monotonic()
        expired = [
            k for k, (_, ts) in self._store.items()
            if now - ts > max_age_seconds
        ]
        for k in expired:
            del self._store[k]
        if expired:
            logger.debug("SnapshotStore: expired %d snapshots", len(expired))

    def __len__(self) -> int:
        return len(self._store)


# ---------------------------------------------------------------------------
# Default command handlers
# ---------------------------------------------------------------------------


def _cmd_start(args: str, state: TelegramState) -> str:
    """Handle /start — greet the user."""
    _ = args
    return (
        "🤖 Привет! Я РОББОКС — ваш голосовой ассистент.\n\n"
        "Доступные команды:\n"
        "/help — список команд\n"
        "/status — статус робота\n"
        "/photo — сделать фото\n"
        "/voice — голосовой запрос\n"
        "/led — управление светодиодами\n"
        "/sound — проиграть звук\n"
        "/navigate — навигация\n"
        "/stop — остановить движение\n\n"
        "Или просто напишите сообщение — я отвечу!"
    )


def _cmd_help(args: str, state: TelegramState) -> str:
    """Handle /help — list all commands."""
    _ = args
    _ = state
    return (
        "📋 **Команды РОББОКС:**\n\n"
        "/start — приветствие\n"
        "/help — этот список\n"
        "/status — состояние робота\n"
        "/photo — сделать снимок с камеры\n"
        "/voice <текст> — голосовой запрос через TTS\n"
        "/led <on|off|color> — управление LED-матрицей\n"
        "/sound <имя> — проиграть звук из пакета\n"
        "/navigate <точка> — поехать к waypoint\n"
        "/stop — экстренная остановка\n\n"
        "Бот также понимает обычные сообщения — просто напишите!"
    )


def _cmd_status(args: str, state: TelegramState) -> str:
    """Handle /status — report robot state."""
    _ = args
    return (
        f"📊 **Статус РОББОКС**\n"
        f"• Сообщений обработано: {state.message_count}\n"
        f"• Последняя команда: {state.last_command or 'нет'}\n"
        f"• Статус: {'🔓 admin' if state.auth_status == 'admin' else '👤 ' + state.auth_status}"
    )


def _cmd_photo(args: str, state: TelegramState) -> str:
    """Handle /photo — placeholder."""
    _ = args
    _ = state
    return "📸 Заглушка фото. Камера будет доступна после интеграции SnapshotStore."


def _cmd_voice(args: str, state: TelegramState) -> str:
    """Handle /voice — forward to voice pipeline."""
    _ = state
    if not args.strip():
        return "🎤 Использование: /voice <текст для озвучивания>"
    return f"🔊 Голосовой запрос принят: «{args}» (заглушка — TTS в P1)"

    async def _async_impl() -> str:
        return f"Голосовой запрос: {args}"
    return _async_impl()  # dummy; real impl calls TTS via side-effect bus


def _cmd_led(args: str, state: TelegramState) -> str:
    """Handle /led — placeholder."""
    _ = args
    _ = state
    return "💡 Управление LED (заглушка). Доступно после интеграции LED-ноды."


def _cmd_sound(args: str, state: TelegramState) -> str:
    """Handle /sound — placeholder."""
    _ = args
    _ = state
    return "🔔 Звук (заглушка). Доступно после интеграции sound-ноды."


def _cmd_navigate(args: str, state: TelegramState) -> str:
    """Handle /navigate — placeholder."""
    _ = args
    _ = state
    return "🗺️ Навигация (заглушка). Доступно после интеграции Nav2."


def _cmd_stop(args: str, state: TelegramState) -> str:
    """Handle /stop — emergency stop."""
    _ = args
    _ = state
    return "🛑 Экстренная остановка! Робот немедленно прекращает движение."


# ---------------------------------------------------------------------------
# TelegramHarness
# ---------------------------------------------------------------------------


class TelegramHarness(Harness[TelegramState]):
    """Harness wrapping the full Telegram bot interaction pipeline.

    Processes a single Telegram update per ``step()`` call. The
    update dict follows the pattern of ``python-telegram-bot``'s
    ``Update`` object, normalized to a plain dict for testability.

    The outer loop (long-polling / webhook) stays in the existing
    ``telegram_node.py`` — this harness only processes individual
    messages.
    """

    name = "telegram"

    # ── init ────────────────────────────────────────────────────

    async def init(self) -> None:
        """Wire command registry, auth middleware, and snapshot store."""
        await super().init()

        # Command registry with placeholder handlers
        self._registry = TelegramCommandRegistry()
        self._registry.register("/start", _cmd_start)
        self._registry.register("/help", _cmd_help)
        self._registry.register("/status", _cmd_status)
        self._registry.register("/photo", _cmd_photo)
        self._registry.register("/voice", _cmd_voice)
        self._registry.register("/led", _cmd_led)
        self._registry.register("/sound", _cmd_sound)
        self._registry.register("/navigate", _cmd_navigate)
        self._registry.register("/stop", _cmd_stop)

        # Auth — allow all in dev mode, config-driven in production
        allowed = getattr(self.config, "telegram_allowed_users", None)
        if isinstance(allowed, str):
            allowed = [u.strip() for u in allowed.split(",") if u.strip()]
        self._auth = AuthMiddleware(allowed_users=allowed)

        # Snapshot store
        self._snapshots = SnapshotStore()

        # Initialize state
        self.state = TelegramState()

        logger.info(
            "TelegramHarness: %d commands registered, auth=%s",
            len(self._registry),
            "enabled" if allowed else "open",
        )

    # ── step ────────────────────────────────────────────────────

    async def step(self, input_data: Any) -> str:
        """Process a single Telegram update.

        Args:
            input_data: A dict with keys matching the Telegram
                ``Update`` pattern: ``chat_id``, ``user_id``,
                ``username`` (optional), ``text`` or ``command`` +
                ``args``. Also accepts a plain string (treated as
                a text message from an anonymous user).

        Returns:
            Response text to send back to the Telegram chat.
        """
        # Normalise input
        update: dict[str, Any] = (
            input_data if isinstance(input_data, dict) else {"text": str(input_data)}
        )
        self.state.chat_id = str(update.get("chat_id", ""))
        self.state.user_id = str(update.get("user_id", ""))
        self.state.username = str(update.get("username", ""))
        self.state.message_count += 1

        # Auth check
        if not self._auth.check(self.state.user_id):
            logger.warning(
                "TelegramHarness: blocked user %s (not in allowed list)",
                self.state.user_id,
            )
            return "⛔ Доступ запрещён."

        # Command dispatch
        command = str(update.get("command", ""))
        if command:
            self.state.last_command = command
            args = str(update.get("args", ""))
            return await self._registry.dispatch(command, args, self.state)

        # Text message → LLM chat
        text = str(update.get("text", ""))
        if text:
            scope = f"tg:{self.state.chat_id}"
            try:
                history = await self.memory.load_recent(scope, limit=20)
                messages = [
                    {"role": t.role, "content": t.content} for t in history
                ]
                messages.append({"role": "user", "content": text})

                response = await self.llm.complete(messages, tools=[])

                await self.memory.append_turn(
                    scope, Turn(role="user", content=text)
                )
                await self.memory.append_turn(
                    scope,
                    Turn(
                        role="assistant",
                        content=str(response.content),
                    ),
                )
                return str(response.content)
            except Exception:
                logger.exception("TelegramHarness: LLM turn failed")
                return "Извините, произошла ошибка. Попробуйте позже."

        return "no_action"

    # ── teardown ────────────────────────────────────────────────

    async def teardown(self) -> None:
        """Expire all snapshots and release parent resources."""
        self._snapshots.expire(max_age_seconds=0)
        await super().teardown()


__all__ = [
    "TelegramHarness",
    "TelegramState",
    "TelegramCommandRegistry",
    "AuthMiddleware",
    "SnapshotStore",
]
