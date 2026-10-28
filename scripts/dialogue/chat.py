#!/usr/bin/env python3
"""Консоль диалога робота на ходе v2 - без ROS2, без железа (карточка #4163).

Это тот же стенд, что гоняет марафон без робота (``scripts/dialog/dry_stand.py``): настоящие адресация, сессия, Tier-1,
срез тулов, цепочка LLM с мостом команд (``dialog_v2_ports.assemble_llm``), системный промпт v2 и исполнитель действий.
Тулы исполняют фейковые владельцы (речь, музыка, лицо, голос, статус робота отвечают событием сразу), память -
настоящий ``VoiceMemory`` в e2e-БД. Цикла ``AgentCore`` v1 здесь нет: в v2 его нет (``dialog_v2_ports.py``).

    scripts/dialogue/run.ps1                              # Windows
    scripts/dialogue/run.sh                               # Linux / macOS / git-bash
    python scripts/dialogue/chat.py --once "Сколько у тебя заряда?" --once "Робот, стоп"

На каждый ход печатается: что сказал робот (``said``), вызванные тулы и их исходы («вызван» ещё не «исполнен»),
сработал ли Tier-1 без LLM, число вызовов LLM, время хода, токены и каких владельцев ход коснулся (``touched``).
``--variant B`` - вариант «понимает модель» (Tier-1 только безопасность, срез тулов по состоянию, без слов фразы).

Ключи API читаются из ``scripts/dialogue/.env`` (в git не попадает), затем из корневого ``.env``; уже
экспортированная переменная важнее файла. Значения нигде не печатаются.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import shutil
import sys
import traceback
from pathlib import Path
from typing import Callable, List, Optional, Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dialog"))

import dry_marathon as dm  # noqa: E402
import understanding_variants as uv  # noqa: E402
from dry_stand import DryStand, FakeClock, HearResult, LiveClock, new_workdir  # noqa: E402
from env_files import load_env_files  # noqa: E402

#: Голос консоли для фейковой биометрии: «кто говорит» - один человек за клавиатурой.
CONSOLE_VOICE = "console"
#: Откуда реплика: ``operator`` адресован без вейка (как Telegram/оператор у робота), ``respeaker`` - с вейком (--wake-word).
SOURCE_CONSOLE, SOURCE_VOICE = "operator", "respeaker"
DEFAULT_PROVIDERS = "deepseek"
VARIANTS = (uv.A, uv.B)


class C:
    """ANSI-цвета; :func:`disable_colour` обнуляет все."""

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
    for attr in [a for a in vars(C) if a.isupper()]:
        setattr(C, attr, "")


def _force_utf8_stdout() -> None:
    """Кириллица на консоли Windows с cp1251/cp1252 и включение ANSI-последовательностей."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            try:
                reconfigure(encoding="utf-8", errors="replace")
            except Exception:  # noqa: BLE001 — косметика, не падение
                pass
    if os.name == "nt":
        try:
            import ctypes

            kernel32 = ctypes.windll.kernel32  # type: ignore[attr-defined]
            kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)  # STD_OUTPUT_HANDLE, ENABLE_VIRTUAL_TERMINAL_PROCESSING
        except Exception:  # noqa: BLE001 — цвета необязательны
            pass


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="chat.py", formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Консоль диалога РОББОКСа на ходе v2 (сухой стенд, без ROS2).")
    parser.add_argument("--providers", default=os.environ.get("ROBBOX_LLM_PROVIDERS", DEFAULT_PROVIDERS),
                        help="Цепочка LLM-провайдеров через запятую; первый - primary, остальные - fallback.")
    parser.add_argument("--model", default="", help="Модель цепочки (одна на все провайдеры; по умолчанию - своя).")
    parser.add_argument("--once", action="append", metavar="TEXT",
                        help="Отправить фразу и выйти. Можно повторять - получится сценарий.")
    parser.add_argument("--db", default=os.environ.get("ROBBOX_LOCAL_DB", ""),
                        help="Файл e2e-памяти, который переживает запуск («запомни…» живёт до следующего раза). "
                             "Пусто - временная память. Боевая /data запрещена.")
    parser.add_argument("--variant", choices=VARIANTS, default=uv.A,
                        help="A - как на роботе; B - понимает модель: Tier-1 только безопасность, срез тулов по "
                             "состоянию без слов фразы (только стенд).")
    parser.add_argument("--wake-word", action="store_true",
                        help="Требовать вейк («Робот, …») как на роботе; без него реплика уходит в бэклог.")
    parser.add_argument("--speaker-name", default="",
                        help="Имя собеседника: робот считает, что узнал вас по голосу (фейковая биометрия).")
    parser.add_argument("--night-clock", action="store_true",
                        help="Фейковые часы марафона (24 октября, час сорок ночи) вместо настоящих: чтобы воспроизвести "
                             "сценарий марафона; по умолчанию «который час?» отвечает настоящим временем.")
    parser.add_argument("--deadline-s", type=float, default=None, help="Потолок хода, секунд.")
    parser.add_argument("--no-color", action="store_true", help="Без ANSI-цветов.")
    parser.add_argument("--debug", action="store_true",
                        help="Показывать срез тулов хода, строки движка и traceback; оставить рабочий каталог.")
    return parser


# ── Один ход ──────────────────────────────────────────────────────────────────────────────────────────────────────


def outcome_line(result: HearResult) -> str:
    """Вызванные тулы и их исходы: «вызван» ещё не «исполнен» (``rejected{…}`` - отказ владельца)."""
    if not result.tools:
        return "тулы: нет"
    outcomes = ", ".join(f"{tool}: {status}" + (f"({why})" if why else "") for tool, status, why in result.outcomes)
    return f"тулы: {', '.join(result.tools)}  →  исходы: {outcomes or 'нет данных'}"


def tier1_line(result: HearResult) -> str:
    return (f"Tier-1: {result.tier1} (код решил сам, без LLM)" if result.tier1
            else "Tier-1: не сработал (ход к модели)" if result.llm_calls else "Tier-1: нет, LLM не звали")


def stats_line(result: HearResult, number: int) -> str:
    tokens = f" · токены {result.tokens}" if result.tokens else ""
    touched = ",".join(sorted(result.touched)) or "-"
    return (f"[ход {number} · вариант {result.variant} · {result.latency_ms / 1000:.1f} с · "
            f"вызовов LLM {result.llm_calls}{tokens} · касался: {touched}]")


class Console:
    """Консоль поверх готового стенда: реплика → ``stand.hear`` → печать хода. Стенд приходит снаружи (тесты дают
    стенд с фейковым провайдером), поэтому консоль не знает, откуда модель."""

    def __init__(self, stand: DryStand, args: argparse.Namespace, say: Callable[[str], None] = print) -> None:
        self.stand = stand
        self.args = args
        self.say = say
        self.turn_no = 0
        self.last: Optional[HearResult] = None

    async def send(self, text: str) -> HearResult:
        """Одна реплика человека; печатает ход и возвращает его итог."""
        source = SOURCE_VOICE if self.args.wake_word else SOURCE_CONSOLE
        result = await self.stand.hear(text, CONSOLE_VOICE, source=source)
        self.last = result
        if not result.addressed:
            self.say(f"{C.YELLOW}… не адресовано ({result.reason}): без вейка робот не слушает, фраза ушла в бэклог{C.RESET}")
            return result
        self.turn_no += 1
        self.show(result)
        return result

    def show(self, result: HearResult) -> None:
        for line in result.said:
            self.say(f"{C.GREEN}🤖 {line}{C.RESET}")
        if not result.said:
            status = result.aggregate.status if result.aggregate is not None else "?"
            self.say(f"{C.YELLOW}… робот промолчал (ход: {status}){C.RESET}")
        self.say(f"{C.MAGENTA}🔧 {outcome_line(result)}{C.RESET}")
        self.say(f"{C.CYAN}⚡ {tier1_line(result)}{C.RESET}")
        if self.args.debug:
            self.say(f"{C.DIM}   срез: {len(result.offered)} тулов: {', '.join(result.offered) or '-'}"
                     + (f"; мост добавил: {', '.join(result.widened)}" if result.widened else "") + C.RESET)
        self.say(f"{C.DIM}{stats_line(result, self.turn_no)}{C.RESET}")

    # ── команды ──────────────────────────────────────────────────────────────────────────────────────────────────

    async def command(self, line: str) -> bool:
        """``/команда``; ``False`` - выйти из консоли."""
        parts = line.split()
        cmd, rest = parts[0].lower(), parts[1:]
        if cmd in {"/exit", "/quit", "/q"}:
            return False
        handler = getattr(self, "_cmd_" + cmd[1:], None)
        if handler is None:
            self.say(f"{C.YELLOW}неизвестная команда {cmd} - /help{C.RESET}")
        else:
            handler(rest)
        return True

    def _cmd_help(self, _rest: Sequence[str]) -> None:
        self.say(f"{C.DIM}/help           - эта справка\n/exit /quit /q  - выход (Ctrl+C тоже)\n"
                 "/reset          - новая сессия: окно ходов и бэклог пусты (память остаётся)\n"
                 "/state          - вариант, память, сессия, играет ли музыка\n"
                 "/tools          - срез тулов последнего хода\n"
                 "/variant A|B    - сменить вариант понимания (A как на роботе, B понимает модель)\n"
                 f"/facts          - факты в e2e-памяти{C.RESET}")

    def _cmd_reset(self, _rest: Sequence[str]) -> None:
        self.stand.reset_session()
        self.turn_no = 0
        self.say(f"{C.DIM}новая сессия: эпоха {self.stand.session.snapshot.epoch}, окно ходов пусто{C.RESET}")

    def _cmd_state(self, _rest: Sequence[str]) -> None:
        snap = self.stand.session.snapshot
        silence = "да" if snap.top("Silence") is not None else "нет"
        self.say(f"{C.DIM}вариант={self.stand.variant}  эпоха={snap.epoch}  музыка играет={snap.music_playing}  "
                 f"тишина={silence}  память={self.stand.memory.db_path}  вызовов LLM за сессию={self.stand.counter.calls}"
                 f"{C.RESET}")

    def _cmd_tools(self, _rest: Sequence[str]) -> None:
        if self.last is None or not self.last.offered:
            self.say(f"{C.DIM}срез: у последнего хода тулов не было (Tier-1 или тишина){C.RESET}")
            return
        self.say(f"{C.DIM}срез последнего хода ({len(self.last.offered)}): {', '.join(self.last.offered)}{C.RESET}")

    def _cmd_variant(self, rest: Sequence[str]) -> None:
        if not rest or rest[0].upper() not in VARIANTS:
            self.say(f"{C.YELLOW}/variant {'|'.join(VARIANTS)}{C.RESET}")
            return
        self.stand.variant = uv.check(rest[0].upper())
        self.say(f"{C.DIM}вариант понимания: {self.stand.variant}{C.RESET}")

    def _cmd_facts(self, _rest: Sequence[str]) -> None:
        facts = self.stand.memory.get_facts(limit=50)
        for fact in facts:
            self.say(f"  {C.CYAN}{fact.get('category', '')}{C.RESET} = {fact.get('fact', '')}")
        self.say(f"{C.DIM}{len(facts)} фактов{C.RESET}")

    def banner(self) -> None:
        a = self.args
        self.say(f"\n{C.BOLD}{C.CYAN}РОББОКС - консоль диалога v2{C.RESET} {C.DIM}(сухой стенд, без ROS2){C.RESET}")
        self.say(f"{C.DIM}{'─' * 62}{C.RESET}")
        self.say(f"  LLM        : {C.BOLD}{a.providers}{C.RESET}" + (f"  модель={a.model}" if a.model else ""))
        self.say(f"  вариант    : {self.stand.variant}"
                 + ("  (Tier-1 только безопасность, срез тулов по состоянию)" if self.stand.variant != uv.A else ""))
        self.say(f"  память     : {self.stand.memory.db_path}" + ("" if a.db else "  (временная)"))
        self.say(f"  вейк       : {'нужен' if a.wake_word else 'не нужен (реплика с консоли адресована)'}")
        self.say(f"  часы       : {'фейковые, ночь марафона (--night-clock)' if a.night_clock else 'настоящие'}")
        self.say("  тулы       : исполняют фейковые владельцы (речь, музыка, лицо, голос, статус)")
        self.say(f"{C.DIM}{'─' * 62}\n/help - команды, /exit - выход{C.RESET}\n")


# ── Запуск ────────────────────────────────────────────────────────────────────────────────────────────────────────


def console_clock(args: argparse.Namespace) -> FakeClock:
    """Часы консоли: настоящее время суток; ``--night-clock`` - ночь мастерской, как в сухом марафоне."""
    return FakeClock() if args.night_clock else LiveClock()


def build_stand(args: argparse.Namespace, workdir: Path) -> DryStand:
    """Стенд с настоящей цепочкой LLM; ``--db`` - постоянная e2e-память (вне боевой ``/data``)."""
    memory = Path(args.db).expanduser().resolve() if args.db else None
    providers = [p for p in args.providers.split(",") if p.strip()]
    if not providers:
        raise SystemExit("--providers пуст: укажи хотя бы одного провайдера")
    if len(providers) > 1 and args.model:
        print(f"{C.YELLOW}⚠ --model ставится на всю цепочку {providers}: у фолбека с другим API своя модель не задаётся"
              f"{C.RESET}")
    try:
        stand = dm.build_real_stand(providers, workdir, args.deadline_s, variant=args.variant, memory_path=memory,
                                    model=args.model, clock=console_clock(args))
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    except RuntimeError as exc:
        raise SystemExit(f"{exc}\nПроверь ключи в {SCRIPT_DIR / '.env'} (см. .env.example).") from exc
    if args.speaker_name:
        stand.speakers.register(CONSOLE_VOICE, args.speaker_name)
    return stand


async def loop(console: Console, once: Optional[List[str]]) -> int:
    if once:
        for phrase in once:
            if phrase.strip():
                console.say(f"{C.BLUE}› {phrase}{C.RESET}")
                await console.send(phrase)
        return 0
    console.banner()
    while True:
        try:
            line = (await asyncio.to_thread(input, f"{C.BLUE}› {C.RESET}")).strip()
        except (EOFError, KeyboardInterrupt):
            console.say("")
            return 0
        if not line:
            continue
        if line.startswith("/"):
            if not await console.command(line):
                return 0
            continue
        await console.send(line)


async def run(args: argparse.Namespace) -> int:
    workdir = new_workdir("console_")
    stand = build_stand(args, workdir)
    try:
        return await loop(Console(stand, args), args.once)
    finally:
        stand.close()
        if not args.debug:
            shutil.rmtree(workdir, ignore_errors=True)


def main(argv: Optional[Sequence[str]] = None) -> int:
    # .env читается ДО argparse: дефолты --providers и --db берутся из окружения.
    _force_utf8_stdout()
    loaded = load_env_files([SCRIPT_DIR / ".env", REPO_ROOT / ".env"])
    args = build_arg_parser().parse_args(argv)
    if args.no_color or not sys.stdout.isatty():
        disable_colour()
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")
    for noisy in ("openai", "httpx", "httpcore", "asyncio"):  # в DEBUG они печатают тело каждого запроса
        logging.getLogger(noisy).setLevel(logging.WARNING)
    for path, applied in loaded:
        print(f"{C.DIM}env: {path} ({applied} перем.){C.RESET}")
    if not loaded:
        print(f"{C.YELLOW}⚠ .env не найден - беру ключи из окружения. Шаблон: {SCRIPT_DIR / '.env.example'}{C.RESET}")
    os.environ["ROBOT_LLM_VERBOSE"] = "1" if args.debug else os.environ.get("ROBOT_LLM_VERBOSE", "0")
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print()
        return 0
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001 — верхний предохранитель
        print(f"{C.RED}✖ {type(exc).__name__}: {exc}{C.RESET}")
        if args.debug:
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
