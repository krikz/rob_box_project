#!/usr/bin/env python3
"""Local text chat with the robot's dialogue system — no ROS2 required.

The dialogue brain (``DialogCore`` + the LLM provider chain + the tool
catalog + SQLite memory) lives in ``rob_box_harness``, which is
deliberately ROS2-free: ``dialogue_node.py`` is only the shell that wires
it to topics. This script is a second shell — a terminal REPL — around
the *same* core, so what you talk to here is what the robot runs:

* the same system prompt (``src/rob_box_voice/prompts/master_prompt_compact.txt``),
* the same provider chain and defaults (``llm_providers: "minimax,deepseek"``
  → health-aware fallback),
* the same 51-tool catalog the LLM sees on the robot,
* the same DSM transitions per turn (WAKE_WORD → STT_RESULT → DIALOGUE_END),
* the same SQLite memory schema.

What is NOT the same: nothing is executed on hardware. Tool calls are
simulated (see :mod:`local_tools`), so the model *thinks* it drove the
motors — which is exactly what you want when testing prompts, tool
choice and memory.

Usage::

    scripts/dialogue/run.ps1              # Windows
    scripts/dialogue/run.sh               # Linux / macOS / git-bash
    python scripts/dialogue/chat.py --help

API keys are read from ``scripts/dialogue/.env`` (git-ignored; copy
``.env.example``), then from the repo-root ``.env``, then from the real
environment. Nothing already exported is overwritten.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Any, Mapping

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]

#: ROS2-free packages the dialogue core is assembled from. Prepended to
#: ``sys.path`` so the repo checkout always wins over an installed copy —
#: you are testing the code in front of you, not last week's wheel.
_SRC_PACKAGES = (
    "rob_box_core",
    "rob_box_llm",
    "rob_box_harness",
    "rob_box_voice",  # only pure helpers (dialogue_text); never the node
)


# ---------------------------------------------------------------------------
# Bootstrap: stdout encoding, sys.path, .env
# ---------------------------------------------------------------------------


def _force_utf8_stdout() -> None:
    """Make Cyrillic printable on a cp1251/cp1252 Windows console."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            try:
                reconfigure(encoding="utf-8", errors="replace")
            except Exception:  # noqa: BLE001 — best effort, never fatal
                pass


def _enable_ansi_on_windows() -> None:
    """Turn on VT sequences so the colours below are not literal escapes."""
    if os.name != "nt":
        return
    try:
        import ctypes

        kernel32 = ctypes.windll.kernel32  # type: ignore[attr-defined]
        # -11 = STD_OUTPUT_HANDLE, 0x0004 = ENABLE_VIRTUAL_TERMINAL_PROCESSING
        kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    except Exception:  # noqa: BLE001 — colours are cosmetic
        pass


def _add_src_to_path() -> None:
    # local_tools.py sits next to this file; when chat.py is imported
    # (tests) rather than run, sys.path[0] is not this directory.
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    for pkg in _SRC_PACKAGES:
        path = REPO_ROOT / "src" / pkg
        if path.is_dir():
            entry = str(path)
            if entry in sys.path:
                sys.path.remove(entry)
            sys.path.insert(0, entry)


def parse_env_file(path: Path) -> dict[str, str]:
    """Parse a ``KEY=VALUE`` .env file. Tolerant, no dependencies.

    Ignores blank lines and ``#`` comments, strips an optional ``export``
    prefix and one layer of matching quotes. Values are taken verbatim
    otherwise — an API key may legitimately contain ``#`` or ``=``.
    """
    values: dict[str, str] = {}
    try:
        raw = path.read_text(encoding="utf-8-sig")
    except OSError:
        return values
    for line in raw.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("export "):
            stripped = stripped[len("export ") :].lstrip()
        key, sep, value = stripped.partition("=")
        if not sep:
            continue
        key = key.strip()
        if not key:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        values[key] = value
    return values


def load_env_files(paths: list[Path]) -> list[tuple[Path, int]]:
    """Load .env files into ``os.environ`` without clobbering real env vars.

    Earlier files win over later ones; anything already exported in the
    shell wins over every file. Returns ``(path, keys_applied)`` for the
    files that existed, for the startup banner.
    """
    loaded: list[tuple[Path, int]] = []
    for path in paths:
        if not path.is_file():
            continue
        applied = 0
        for key, value in parse_env_file(path).items():
            if value and not os.environ.get(key):
                os.environ[key] = value
                applied += 1
        loaded.append((path, applied))
    return loaded


_force_utf8_stdout()
_enable_ansi_on_windows()
_add_src_to_path()


# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------


class C:
    """ANSI colours; every attribute is blanked by :func:`disable_colour`."""

    DIM = "\033[2m"
    BOLD = "\033[1m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    RESET = "\033[0m"


def disable_colour() -> None:
    for attr in ("DIM", "BOLD", "RED", "GREEN", "YELLOW", "BLUE", "MAGENTA", "CYAN", "RESET"):
        setattr(C, attr, "")


def say(text: str = "") -> None:
    print(text, flush=True)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


DEFAULT_PROVIDERS = "minimax,deepseek"  # docker/vision/config/.../dialogue_node.yaml
DEFAULT_PROMPT_FILE = "master_prompt_compact.txt"
DEFAULT_DB = "~/.rob_box/local_chat.db"

#: Маркеры завершения цикла из master_prompt («after the LAST speak_text
#: → return "done"»). Совпадает со списком в ``dialogue_node``, который
#: по ним глушит авто-TTS.
_DONE_MARKERS = frozenset(
    {"done", "task complete", "task_complete", "готово", "всё", "выполнено"}
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="chat.py",
        description="Локальный текстовый чат с диалоговой системой РОББОКСа (без ROS2).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--providers",
        default=os.environ.get("ROBBOX_LLM_PROVIDERS", DEFAULT_PROVIDERS),
        help="Цепочка LLM-провайдеров через запятую; первый — primary, остальные — fallback.",
    )
    parser.add_argument("--model", default="", help="Переопределить модель primary-провайдера.")
    parser.add_argument(
        "--base-url", default="", help="Переопределить base_url primary-провайдера."
    )
    parser.add_argument(
        "--timeout", type=float, default=90.0, help="Таймаут LLM-запроса, секунд."
    )
    parser.add_argument(
        "--prompt-file",
        default=DEFAULT_PROMPT_FILE,
        help="Файл системного промпта из src/rob_box_voice/prompts/ (или абсолютный путь).",
    )
    parser.add_argument(
        "--db",
        default=os.environ.get("ROBBOX_LOCAL_DB", DEFAULT_DB),
        help="SQLite-файл памяти. ':memory:' — не сохранять между запусками.",
    )
    parser.add_argument("--user-id", default="local", help="Scope памяти (кто говорит).")
    parser.add_argument(
        "--speaker-name",
        default="",
        help="Имя в <system_context><user_profile> — робот считает, что узнал вас по голосу.",
    )
    parser.add_argument(
        "--history-max-turns", type=int, default=20, help="Сколько ходов истории уходит в LLM."
    )
    parser.add_argument(
        "--no-tools",
        action="store_true",
        help="Не показывать LLM каталог инструментов (чистый чат без tool-calls).",
    )
    parser.add_argument(
        "--no-stream",
        action="store_true",
        help="Отключить стриминг (на роботе llm_streaming: true).",
    )
    parser.add_argument(
        "--wake-word",
        action="store_true",
        help="Требовать wake-word («роббокс, ...»), как на роботе через STT.",
    )
    parser.add_argument(
        "--once",
        action="append",
        metavar="TEXT",
        help="Отправить фразу и выйти. Можно повторять — получится сценарий.",
    )
    parser.add_argument("--no-color", action="store_true", help="Без ANSI-цветов.")
    parser.add_argument(
        "--debug", action="store_true", help="DEBUG-логи провайдеров и полные traceback'и."
    )
    return parser


# ---------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------


def resolve_prompt_path(prompt_file: str) -> Path:
    candidate = Path(prompt_file).expanduser()
    if candidate.is_absolute():
        return candidate
    return REPO_ROOT / "src" / "rob_box_voice" / "prompts" / prompt_file


def build_llm(args: argparse.Namespace) -> tuple[Any, list[str]]:
    """Build the provider chain exactly the way ``dialogue_node`` does.

    Providers that fail to build (missing key, bad config) are skipped
    with a warning; a chain of two or more is wrapped in
    :class:`HealthAwareFallbackLLM`, one is used directly.
    """
    from rob_box_harness.health import (
        DEFAULT_HEALTH_TTL_S,
        HealthAwareFallbackLLM,
        HealthCache,
        check_deepseek_balance,
    )
    from rob_box_harness.providers import (
        DEEPSEEK_DEFAULT_BASE_URL,
        LLM_PROVIDER_REGISTRY,
        build_provider,
        known_provider_names,
    )

    names = [n.strip().lower() for n in args.providers.split(",") if n.strip()]
    if not names:
        raise SystemExit("--providers пуст: укажи хотя бы одного провайдера")

    built: list[Any] = []
    chain: list[str] = []
    for index, name in enumerate(names):
        if name not in LLM_PROVIDER_REGISTRY:
            say(
                f"{C.YELLOW}⚠ неизвестный провайдер {name!r} — пропущен. "
                f"Известные: {', '.join(known_provider_names())}{C.RESET}"
            )
            continue
        # --model / --base-url переопределяют только primary: цепочка
        # фолбеков состоит из разных API, общая модель им не подходит.
        is_primary = index == 0
        try:
            provider = build_provider(
                name,
                base_url=args.base_url if is_primary else "",
                model=args.model if is_primary else "",
                timeout_s=args.timeout,
            )
        except Exception as exc:  # noqa: BLE001 — а вдруг соберётся следующий
            entry = LLM_PROVIDER_REGISTRY[name]
            say(
                f"{C.YELLOW}⚠ {entry['display_name']} ({name}) не собран: "
                f"{type(exc).__name__}: {exc}{C.RESET}"
            )
            say(f"{C.DIM}  ключ берётся из {entry['env_key_var']}{C.RESET}")
            continue
        built.append(provider)
        chain.append(name)

    if not built:
        raise SystemExit(
            "Ни один LLM-провайдер не собрался. Проверь ключи в "
            f"{SCRIPT_DIR / '.env'} (см. .env.example)."
        )
    # Переопределения адресованы primary. Если primary не собрался, они
    # НЕ переезжают на фолбек (у него своя модель и свой API) — но и
    # молчать нельзя: именно так «--base-url на мок» уходит в живой API.
    if (args.model or args.base_url) and chain[0] != names[0]:
        say(
            f"{C.YELLOW}⚠ --model/--base-url заданы для {names[0]!r}, "
            f"но он не собрался — активен {chain[0]!r} со своими "
            f"настройками{C.RESET}"
        )
    if len(built) == 1:
        return built[0], chain

    balance_checkers: dict[str, Any] = {}
    if "deepseek" in chain and LLM_PROVIDER_REGISTRY["deepseek"]["has_balance_api"]:
        def _deepseek_balance() -> Any:
            return check_deepseek_balance(
                DEEPSEEK_DEFAULT_BASE_URL,
                os.environ.get("DEEPSEEK_API_KEY", ""),
                timeout_s=5.0,
            )

        balance_checkers["deepseek"] = _deepseek_balance

    llm = HealthAwareFallbackLLM(
        built,
        cache=HealthCache(ttl_s=DEFAULT_HEALTH_TTL_S),
        balance_checkers=balance_checkers,
        logger=logging.getLogger("local-chat.health"),
    )
    return llm, chain


async def build_memory(db: str) -> Any:
    """Open the SQLite voice memory, falling back to in-memory."""
    from rob_box_harness.memory import InMemoryStore, SQLiteVoiceMemory

    if db.strip().lower() in {"none", "off"}:
        store: Any = InMemoryStore()
        await store.init()
        return store
    try:
        store = SQLiteVoiceMemory(db_path=db)
        await store.init()
        return store
    except Exception as exc:  # noqa: BLE001
        say(f"{C.YELLOW}⚠ SQLite ({db}) не открылся: {exc} — память только на сессию{C.RESET}")
        store = InMemoryStore()
        await store.init()
        return store


def build_system_context(speaker_name: str) -> str:
    """The second system message, shaped like ``dialogue_node`` builds it.

    Same tags in the same order as
    ``DialogueNode._build_dynamic_system_context`` — the prompt's RULE
    #SYSCTX teaches the model to read runtime facts from exactly these
    fields, so a differently-shaped snapshot would test a different
    prompt than the robot runs.
    """
    lines = ["<system_context>", "  <user_profile>"]
    lines.append(f"    <name>{speaker_name or 'unknown'}</name>")
    lines.append("  </user_profile>")
    lines.append("  <hardware>")
    lines.append("    <battery>unknown</battery>")
    lines.append("    <tts_voice>local-text</tts_voice>")
    lines.append("    <tts_provider>none</tts_provider>")
    lines.append("  </hardware>")
    lines.append("  <position>unknown</position>")
    lines.append(
        "  <tts_context>текстовый канал: реплики печатаются, "
        "голос не синтезируется</tts_context>"
    )
    lines.append("</system_context>")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Session
# ---------------------------------------------------------------------------


class ChatSession:
    """One REPL session over a live :class:`DialogCore`."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.core: Any = None
        self.dsm: Any = None
        self.memory: Any = None
        self.llm: Any = None
        self.chain: list[str] = []
        self.tools: Any = None
        self.tool_count = 0
        self.turn_index = 0
        self._spoken: list[str] = []

    # ── lifecycle ────────────────────────────────────────────────────

    async def start(self) -> None:
        from rob_box_harness.core.dialog_core import DialogCore
        from rob_box_harness.core.dialogue_state_machine import DialogueStateMachine
        from rob_box_harness.tools import FakeToolProvider

        from local_tools import SimulatedToolProvider

        prompt_path = resolve_prompt_path(self.args.prompt_file)
        try:
            system_prompt = prompt_path.read_text(encoding="utf-8")
        except OSError as exc:
            raise SystemExit(f"Не читается системный промпт {prompt_path}: {exc}") from exc

        self.llm, self.chain = build_llm(self.args)
        self.memory = await build_memory(self.args.db)

        if self.args.no_tools:
            self.tools = FakeToolProvider()
        else:
            self.tools = SimulatedToolProvider(
                memory=self.memory,
                user_id=self.args.user_id,
                on_speak=self._on_speak,
                on_tool=self._on_tool,
            )
        self.tool_count = len(await self.tools.discover())

        self.dsm = DialogueStateMachine(silence_timeout=300.0)
        self.core = DialogCore(
            llm=self.llm,
            tools=self.tools,
            memory=self.memory,
            dsm=self.dsm,
            user_id=self.args.user_id,
            history_trim_limit=self.args.history_max_turns,
            inactivity_timeout=300.0,
            system_prompt=system_prompt,
            use_streaming=not self.args.no_stream,
        )
        self._banner(prompt_path, system_prompt)

    async def close(self) -> None:
        for port in (self.tools, self.memory, self.llm):
            aclose = getattr(port, "aclose", None)
            if aclose is None:
                continue
            try:
                await aclose()
            except Exception:  # noqa: BLE001 — teardown must not raise
                pass

    def _banner(self, prompt_path: Path, system_prompt: str) -> None:
        say()
        say(f"{C.BOLD}{C.CYAN}РОББОКС — локальный диалог{C.RESET} {C.DIM}(без ROS2){C.RESET}")
        say(f"{C.DIM}{'─' * 62}{C.RESET}")
        say(f"  LLM        : {C.BOLD}{' → '.join(self.chain)}{C.RESET}")
        say(f"  промпт     : {prompt_path.name} ({len(system_prompt)} байт)")
        tools_line = "выключены" if self.args.no_tools else f"{self.tool_count} (симуляция)"
        say(f"  инструменты: {tools_line}")
        say(f"  память     : {self.args.db}  scope={self.args.user_id}")
        say(f"  стриминг   : {'нет' if self.args.no_stream else 'да'}")
        if self.args.wake_word:
            say(f"  wake-word  : {C.BOLD}нужен{C.RESET} — начинай фразу с «роббокс»")
        say(f"{C.DIM}{'─' * 62}{C.RESET}")
        say(f"{C.DIM}/help — команды, /exit — выход{C.RESET}")
        say()

    # ── tool callbacks ───────────────────────────────────────────────

    def _on_speak(self, text: str, args: Mapping[str, Any]) -> None:
        self._spoken.append(text)
        extra = []
        if args.get("voice"):
            extra.append(str(args["voice"]))
        if args.get("animation"):
            extra.append(f"anim:{args['animation']}")
        suffix = f"  {C.DIM}[{', '.join(extra)}]{C.RESET}" if extra else ""
        say(f"{C.GREEN}🤖 {text}{C.RESET}{suffix}")

    def _on_tool(self, name: str, args: Mapping[str, Any], result: str) -> None:
        rendered = ", ".join(f"{k}={v!r}" for k, v in args.items())
        say(f"{C.MAGENTA}🔧 {name}({rendered}){C.RESET}")
        if self.args.debug:
            say(f"{C.DIM}   → {result[:400]}{C.RESET}")

    # ── one turn ─────────────────────────────────────────────────────

    async def send(self, text: str) -> None:
        """Run one user turn through the core, mirroring ``_on_stt``."""
        from rob_box_harness.core.dialogue_state_machine import (
            DialogueEvent,
            DialogueStateKind,
        )
        from rob_box_voice.core.dialogue_text import (
            DEFAULT_WAKE_WORDS,
            has_wake_word,
            strip_wake_word,
        )
        from rob_box_voice.core.speak_helpers import strip_done_marker, strip_markdown

        if self.args.wake_word:
            if not has_wake_word(text.lower(), DEFAULT_WAKE_WORDS):
                say(f"{C.DIM}…(без wake-word робот не слушает){C.RESET}")
                return
            text = strip_wake_word(text, DEFAULT_WAKE_WORDS).strip() or text

        # dialogue_node._on_stt: из IDLE переход делает сам wake-word,
        # затем речь двигает LISTENING → DIALOGUE. Передаём готовый
        # event, чтобы DialogCore не переклассифицировал текст (слово
        # «робот» внутри фразы иначе вернёт WAKE_WORD и ход потеряется —
        # issue #1101).
        if self.dsm.current_state == DialogueStateKind.IDLE:
            self.dsm.on_event(DialogueEvent.WAKE_WORD)
        self.dsm.on_event(DialogueEvent.STT_RESULT)

        self.turn_index += 1
        self._spoken = []
        started = time.monotonic()
        result = await self.core.process_input(
            text,
            speaker_tag=self.args.user_id,
            dynamic_system=build_system_context(self.args.speaker_name),
            preclassified_event=DialogueEvent.STT_RESULT,
        )
        elapsed = time.monotonic() - started

        if result.error is not None:
            say(f"{C.RED}✖ {type(result.error).__name__}: {result.error}{C.RESET}")
            if self.args.debug and result.error_traceback:
                say(f"{C.DIM}{result.error_traceback}{C.RESET}")
        else:
            # На роботе речь идёт через speak_text, а текст самого ответа
            # модели — это хвост цикла: маркер «done» и markdown, которые
            # dialogue_node срезает перед авто-TTS теми же хелперами
            # (иначе TTS буквально произносит «дан»).
            spoken = strip_markdown((result.spoken_text or "").strip())
            spoken = strip_done_marker(spoken).strip()
            if spoken.lower() in _DONE_MARKERS:
                spoken = ""
            if spoken and not self._spoken:
                say(f"{C.GREEN}🤖 {spoken}{C.RESET}")
            elif not spoken and not self._spoken:
                reason = result.finish_reason or "пустой ответ"
                say(f"{C.YELLOW}… робот промолчал (finish_reason={reason}){C.RESET}")

        tools = ", ".join(result.tools_called) or "—"
        say(
            f"{C.DIM}[ход {self.turn_index} · {elapsed:.1f}s · state={result.new_state.name}"
            f" · tools: {tools}]{C.RESET}"
        )

        if self.dsm.current_state == DialogueStateKind.DIALOGUE:
            self.dsm.on_event(DialogueEvent.DIALOGUE_END)

    # ── slash commands ───────────────────────────────────────────────

    async def command(self, line: str) -> bool:
        """Handle a ``/command``. Returns ``False`` to stop the REPL."""
        from rob_box_harness.core.dialogue_state_machine import DialogueStateKind

        parts = line.split()
        cmd, rest = parts[0].lower(), parts[1:]

        if cmd in {"/exit", "/quit", "/q"}:
            return False
        if cmd == "/help":
            say(
                f"{C.DIM}"
                "/help            — эта справка\n"
                "/exit /quit /q   — выход (Ctrl+C тоже)\n"
                "/reset           — новая сессия: очистить историю диалога\n"
                "/state           — состояние DSM, провайдер, память\n"
                "/tools [фильтр]  — каталог инструментов, что видит LLM\n"
                "/history [N]     — последние N ходов из памяти (по умолчанию 10)\n"
                "/facts           — факты, которые робот о тебе запомнил"
                f"{C.RESET}"
            )
            return True
        if cmd == "/reset":
            removed = await self.memory.clear_turns(self.args.user_id)
            self.dsm.reset(DialogueStateKind.IDLE)
            self.turn_index = 0
            say(f"{C.DIM}история очищена ({removed} ходов), состояние → IDLE{C.RESET}")
            return True
        if cmd == "/state":
            say(
                f"{C.DIM}state={self.dsm.current_state.name}  "
                f"llm={' → '.join(self.chain)}  "
                f"db={self.args.db}  scope={self.args.user_id}{C.RESET}"
            )
            return True
        if cmd == "/tools":
            specs = await self.tools.discover()
            needle = rest[0].lower() if rest else ""
            shown = [s for s in specs if needle in s.name.lower()]
            for spec in shown:
                say(f"  {C.MAGENTA}{spec.name}{C.RESET} — {spec.description.splitlines()[0][:90]}")
            say(f"{C.DIM}{len(shown)} из {len(specs)}{C.RESET}")
            return True
        if cmd == "/history":
            limit = int(rest[0]) if rest and rest[0].isdigit() else 10
            turns = await self.memory.load_recent(self.args.user_id, limit=limit)
            for turn in turns:
                say(f"  {C.DIM}{turn.role:>9}{C.RESET} {turn.content[:120]}")
            say(f"{C.DIM}{len(turns)} ходов{C.RESET}")
            return True
        if cmd == "/facts":
            facts = await self.memory.list_facts(self.args.user_id, limit=50)
            for fact in facts:
                say(f"  {C.CYAN}{fact.key}{C.RESET} = {fact.value}")
            say(f"{C.DIM}{len(facts)} фактов{C.RESET}")
            return True

        say(f"{C.YELLOW}неизвестная команда {cmd} — /help{C.RESET}")
        return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


async def run(args: argparse.Namespace) -> int:
    session = ChatSession(args)
    await session.start()
    try:
        if args.once:
            for phrase in args.once:
                if not phrase.strip():
                    continue
                say(f"{C.BLUE}› {phrase}{C.RESET}")
                await session.send(phrase)
            return 0
        while True:
            try:
                line = (await asyncio.to_thread(input, f"{C.BLUE}› {C.RESET}")).strip()
            except (EOFError, KeyboardInterrupt):
                say()
                return 0
            if not line:
                continue
            if line.startswith("/"):
                if not await session.command(line):
                    return 0
                continue
            await session.send(line)
    finally:
        await session.close()


def main(argv: list[str] | None = None) -> int:
    # .env читается ДО argparse: дефолты --providers / --db берутся из
    # переменных окружения, а значит файл должен быть уже применён —
    # иначе ROBBOX_LLM_PROVIDERS из .env молча не работал бы.
    loaded = load_env_files([SCRIPT_DIR / ".env", REPO_ROOT / ".env"])

    args = build_arg_parser().parse_args(argv)
    if args.no_color or not sys.stdout.isatty():
        disable_colour()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    # --debug должен показывать НАШИ логи, а не поток DEBUG'а из openai
    # SDK и httpx: там каждый запрос печатается вместе с полным телом
    # (25 КБ промпта), и всё остальное в этом тонет.
    for noisy in ("openai", "httpx", "httpcore", "asyncio"):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    for path, applied in loaded:
        say(f"{C.DIM}env: {path} ({applied} перем.){C.RESET}")
    if not loaded:
        say(
            f"{C.YELLOW}⚠ .env не найден — беру ключи из окружения. "
            f"Шаблон: {SCRIPT_DIR / '.env.example'}{C.RESET}"
        )

    # rob_box_llm печатает в stderr полный контекст каждого запроса
    # (ROBOT_LLM_VERBOSE, по умолчанию «1» — так его видно в docker logs
    # на роботе). В интерактивном чате это 25 КБ промпта на каждый ход,
    # поэтому включаем трассировку только под --debug.
    if args.debug:
        os.environ["ROBOT_LLM_VERBOSE"] = "1"
    else:
        os.environ.setdefault("ROBOT_LLM_VERBOSE", "0")

    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        say()
        return 0
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001 — top-level guard
        say(f"{C.RED}✖ {type(exc).__name__}: {exc}{C.RESET}")
        if args.debug:
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
