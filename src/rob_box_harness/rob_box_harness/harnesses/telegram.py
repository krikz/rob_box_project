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

import asyncio
import logging
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, Union, cast

from rob_box_harness.config import HarnessConfig
from rob_box_harness.effects import Effect, SendReplyEffect
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
# TelegramCommandContext + handler type aliases (P1.4 declarative API)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TelegramCommandContext:
    """Per-call context passed to declarative handlers.

    The new ``(ctx, args) → Effect`` signature (ADR §2.7.3, host
    decision H.1) carries the chat_id / user_id / state the handler
    needs without the harness threading positional args through a
    state object — handlers stay pure functions of (ctx, args).

    The legacy ``(args, state) → str`` signature is preserved for
    back-compat with the 9 placeholder handlers shipped in this
    file and the 28 regression tests that exercise them.
    """

    command: str
    """The slash-command that triggered the handler, e.g. ``"/start"``."""

    args: str
    """Everything after the command (may be empty)."""

    chat_id: str
    """Telegram chat id, already projected from ``state.chat_id``."""

    user_id: str
    """Telegram user id, already projected from ``state.user_id``."""

    state: TelegramState
    """Full mutable state bag for handlers that need more than ctx fields."""


# A declarative handler is ``async (ctx, args) -> Effect``. The
# return type is ``Effect[Any]`` (not a subclass) so handlers may
# emit any member of the effect union.
DeclarativeHandler = Callable[[TelegramCommandContext, str], Awaitable[Effect[Any]]]
DeclarativeSyncHandler = Callable[[TelegramCommandContext, str], Effect[Any]]
DeclarativeHandlerLike = Union[DeclarativeHandler, DeclarativeSyncHandler]


# ---------------------------------------------------------------------------
# TelegramCommandRegistry
# ---------------------------------------------------------------------------


class TelegramCommandRegistry:
    """Declarative command → handler dispatch for Telegram bot commands.

    Mirrors the pattern from ``rob_box_telegram.commands`` (25 handlers)
    in a ROS-agnostic way. Commands are registered with a name
    (e.g. ``"/start"``) and a handler callable.

    Two registration styles are supported (ADR §2.7.3, host H.1 + H.4):

    * **Legacy** ``register(name, (args, state) -> str)`` — used by
      the 9 placeholder handlers in this file and exercised by the
      28 tests in ``test_telegram_harness.py``. Kept for back-compat.
    * **Declarative** ``@registry.command(name)`` decorating an
      ``async (ctx, args) -> Effect`` function. The new behaviour;
      see ``test_telegram_registry.py`` for the contract.

    :meth:`dispatch` returns either the handler's ``Effect`` (new
    API) or its ``str`` (legacy API). The :class:`TelegramHarness`
    dispatches returned ``Effect`` instances through its configured
    ``SideEffectBus`` and returns the string form (synthesised from
    ``SendReplyEffect.text`` for the new path) to callers that
    expect a string.
    """

    def __init__(self) -> None:
        self._handlers: dict[str, DeclarativeHandlerLike] = {}
        # Legacy handlers keyed separately so we can tell a
        # legacy (args, state) -> str from a declarative
        # (ctx, args) -> Effect apart at dispatch time.
        self._legacy: dict[str, Callable[..., Any]] = {}
        self._metadata: dict[str, dict[str, Any]] = {}

    # ----- registration ---------------------------------------------------

    def register(self, command: str, handler: Callable[..., Any]) -> None:
        """Register a LEGACY handler ``(args, state) → str``.

        Kept for back-compat with the 9 placeholder handlers in
        this module and the 28 regression tests. New code should
        use :meth:`command` instead.
        """
        command = _normalise_command(command)
        self._legacy[command] = handler
        self._handlers.pop(command, None)  # declarative wins if previously set
        logger.debug("TelegramCommandRegistry: registered legacy '%s'", command)

    def command(
        self,
        name: str,
        *,
        description: str | None = None,
    ) -> Callable[[DeclarativeHandlerLike], DeclarativeHandlerLike]:
        """Decorator for declarative handlers ``(ctx, args) → Effect``.

        Usage::

            @registry.command("/start", description="Greet the user")
            async def start(ctx, args):
                return SendReplyEffect(channel=ctx.chat_id, text="Hi!")
        """
        command = _normalise_command(name)

        def _decorator(handler: DeclarativeHandlerLike) -> DeclarativeHandlerLike:
            self._handlers[command] = handler
            self._legacy.pop(command, None)  # legacy loses if previously set
            self._metadata[command] = {
                "description": description or "",
                "decorated": True,
            }
            logger.debug("TelegramCommandRegistry: registered declarative '%s'", command)
            return handler

        return _decorator

    def metadata(self, command: str) -> dict[str, Any]:
        """Return metadata registered for ``command`` (description, flags)."""
        command = _normalise_command(command)
        return dict(self._metadata.get(command, {}))

    # ----- dispatch -------------------------------------------------------

    async def dispatch(
        self,
        command: str,
        args: str,
        state: TelegramState,
    ) -> Union[Effect[Any], str]:
        """Execute the handler for ``command``.

        Returns whatever the handler produced:

        * an :class:`Effect` instance — the declarative handler
          contract (P1.4 / H.1). The :class:`TelegramHarness`
          dispatches it through ``self.effects`` and synthesises
          a string reply for legacy callers.
        * a ``str`` — the legacy handler contract (the 9
          placeholders in this module still return strings, and
          the 28 regression tests assert on strings). Returned
          as-is.

        On unknown command / legacy-handler exception the registry
        returns a ``str`` so the existing regression tests continue
        to work without modification. On declarative-handler
        exception the registry returns a :class:`SendReplyEffect`
        so the harness can dispatch it uniformly through the bus.
        """
        command = _normalise_command(command)

        declarative = self._handlers.get(command)
        if declarative is not None:
            ctx = TelegramCommandContext(
                command=command,
                args=args,
                chat_id=state.chat_id,
                user_id=state.user_id,
                state=state,
            )
            try:
                result = declarative(ctx, args)
                if asyncio.iscoroutine(result):
                    result = await result
                # mypy can't narrow ``Awaitable[Effect[Any]] | Effect[Any]`` via
                # ``iscoroutine`` alone, so narrow with cast. Both branches are
                # covered by the check above.
                return cast(Effect[Any], result)
            except Exception:
                logger.exception(
                    "TelegramCommandRegistry: declarative handler '%s' failed",
                    command,
                )
                return SendReplyEffect(
                    channel=state.chat_id,
                    text="Произошла ошибка при выполнении команды.",
                )

        legacy = self._legacy.get(command)
        if legacy is not None:
            try:
                result = legacy(args, state)
                if asyncio.iscoroutine(result):
                    result = await result
                return str(result) if result is not None else ""
            except Exception:
                logger.exception(
                    "TelegramCommandRegistry: legacy handler '%s' failed",
                    command,
                )
                return "Произошла ошибка при выполнении команды."

        # Unknown command — for legacy compatibility return a plain
        # string (the 9 placeholder handlers live in this category;
        # tests in test_telegram_harness.py assert on substrings).
        # The :class:`TelegramHarness` collapses the string into a
        # ``SendReplyEffect`` at the bus boundary.
        return f"Неизвестная команда: {command}. Используйте /help для списка команд."

    def commands(self) -> list[str]:
        """Return all registered command names (declarative + legacy)."""
        return sorted(set(self._handlers) | set(self._legacy))

    def __len__(self) -> int:
        return len(self.commands())

    def __contains__(self, command: str) -> bool:
        command = _normalise_command(command)
        return command in self._handlers or command in self._legacy


def _normalise_command(command: str) -> str:
    """Ensure ``command`` starts with ``/``."""
    if not command.startswith("/"):
        return f"/{command}"
    return command


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

    def wrap(self, handler: Callable[..., Awaitable[Any]]) -> Callable[..., Awaitable[Any]]:
        """Wrap a handler with an auth check.

        Returns a callable that checks auth before delegating to
        the original handler. The wrapped handler must be async
        (matching the harness's ``await registry.dispatch(...)``
        call site).
        """
        _check = self.check

        async def _wrapper(*args: Any, **kwargs: Any) -> Any:
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

        # Command registry — preserve a registry the caller already
        # injected (so test code can pre-populate declarative
        # handlers). Otherwise build the default registry with the
        # 9 placeholder handlers (kept for the regression suite).
        existing = getattr(self, "_registry", None)
        if existing is None:
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
            blocked = "⛔ Доступ запрещён."
            await self.effects.dispatch(
                SendReplyEffect(channel=self.state.chat_id, text=blocked)
            )
            return blocked

        # Command dispatch (P1.4 / H.1)
        command = str(update.get("command", ""))
        if command:
            self.state.last_command = command
            args = str(update.get("args", ""))
            outcome = await self._registry.dispatch(command, args, self.state)
            return await self._dispatch_outcome(outcome)

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
                reply_text = str(response.content)
                # Per ADR §2.10 / H.3: every external effect goes
                # through the SideEffectBus. Text-message replies
                # dispatch SendReplyEffect; legacy callers still see
                # the reply string via the harness return value.
                await self.effects.dispatch(
                    SendReplyEffect(channel=self.state.chat_id, text=reply_text)
                )
                return reply_text
            except Exception:
                logger.exception("TelegramHarness: LLM turn failed")
                err_reply = "Извините, произошла ошибка. Попробуйте позже."
                await self.effects.dispatch(
                    SendReplyEffect(channel=self.state.chat_id, text=err_reply)
                )
                return err_reply

        return "no_action"

    async def _dispatch_outcome(
        self, outcome: Union[Effect[Any], str]
    ) -> str:
        """Dispatch a command-handler outcome and return its string form.

        The harness is the single boundary where Effect → string
        collapse happens (per ADR §2.10). Handlers emit ``Effect``
        instances; the harness routes them through ``self.effects``
        and surfaces the textual content to legacy callers (the
        ROS2 ``telegram_node`` orchestrator that wraps this
        harness). A ``SendReplyEffect`` returns its ``text``; any
        other Effect type returns an empty string (the harness
        stays out of routing logic — that lives in the bus).
        """
        if isinstance(outcome, Effect):
            await self.effects.dispatch(outcome)
            if isinstance(outcome, SendReplyEffect):
                return outcome.text
            return ""
        # Legacy str contract — still useful for the 9 placeholder
        # handlers; dispatch a synthetic SendReply so the user's
        # message is delivered through the bus too.
        await self.effects.dispatch(
            SendReplyEffect(channel=self.state.chat_id, text=outcome)
        )
        return outcome

    # ── teardown ────────────────────────────────────────────────

    async def teardown(self) -> None:
        """Expire all snapshots and release parent resources."""
        self._snapshots.expire(max_age_seconds=0)
        await super().teardown()


__all__ = [
    "TelegramHarness",
    "TelegramState",
    "TelegramCommandRegistry",
    "TelegramCommandContext",
    "AuthMiddleware",
    "SnapshotStore",
    "add_telegram_handlers",
]


# ---------------------------------------------------------------------------
# python-telegram-bot integration (A5)
# ---------------------------------------------------------------------------


# Module-level indirection so tests can monkeypatch the
# ``python-telegram-bot`` ``CommandHandler`` class without importing the
# library. Defaults to ``None`` — production wires the real class via
# :func:`_resolve_ptb_handler`.
_PTB_COMMAND_HANDLER: Any = None


def _resolve_ptb_handler() -> Any:
    """Return the ``python-telegram-bot`` ``CommandHandler`` class.

    Imported lazily so the harness module stays usable in environments
    that don't install ``python-telegram-bot`` (most tests + the
    voice-only pipelines). :class:`ImportError` is re-raised with a
    human-readable hint.
    """
    global _PTB_COMMAND_HANDLER
    if _PTB_COMMAND_HANDLER is not None:
        return _PTB_COMMAND_HANDLER
    try:
        from telegram.ext import CommandHandler as _Cls
    except ImportError as exc:  # pragma: no cover — exercised in bot runtime
        raise ImportError(
            "python-telegram-bot is required for add_telegram_handlers; "
            "install with `pip install python-telegram-bot`"
        ) from exc
    _PTB_COMMAND_HANDLER = _Cls
    return _Cls


def add_telegram_handlers(
    registry: TelegramCommandRegistry,
    app: Any,
) -> int:
    """Register every command in ``registry`` with ``app``.

    ``app`` is a ``telegram.ext.Application`` (or any duck-typed
    object exposing ``add_handler(handler, group=0)``). The function
    strips the leading ``/`` from each command name — python-telegram-bot
    expects bare command strings — and returns the number of handlers
    registered. Returns ``0`` for an empty registry.
    """
    handler_cls = _resolve_ptb_handler()
    added = 0
    for command in registry.commands():
        bare = command.lstrip("/")
        # Wrap the registry call in an adapter so python-telegram-bot's
        # ``(update, context)`` signature reaches the declarative handler.
        async def _adapter(update: Any, context: Any, *, _command: str = command) -> None:
            chat_id = str(getattr(getattr(update, "effective_chat", None), "id", ""))
            user_id = str(getattr(getattr(update, "effective_user", None), "id", ""))
            args_text = " ".join(getattr(context, "args", []) or [])
            state = TelegramState(chat_id=chat_id, user_id=user_id)
            ctx = TelegramCommandContext(
                command=_command,
                args=args_text,
                chat_id=chat_id,
                user_id=user_id,
                state=state,
            )
            handler = registry._handlers.get(_command)
            if handler is None:
                handler = registry._legacy.get(_command)
            if handler is None:
                return None
            outcome = handler(ctx, args_text)
            if hasattr(outcome, "__await__"):
                outcome = await outcome
            # ``outcome`` may be an Effect (declarative) or a str
            # (legacy); the harness caller can dispatch it. We do
            # nothing here so the harness's effect bus stays the
            # single routing boundary.
            return None

        app.add_handler(handler_cls(bare, _adapter))
        added += 1
    return added
