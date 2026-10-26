#!/usr/bin/env python3
"""Разбор событий речи и хода робота для ожидания харнесса E2E (issue #3857, эпик #3683).

Харнесс не спит фиксированные ``E2E_REACTION_WINDOW`` секунд, а опрашивает ``docker logs`` робота и спрашивает
этот модуль: «ответ готов?», «робот ещё говорит?», «наступил якорь перебивания?». Решение принимает код по
строкам лога, а не оператор по таймеру.

Вход (stdin) -- вывод удалённой команды: первая строка ``<epoch робота>`` (``date +%s.%N``), дальше строки
``docker logs``, отфильтрованные ``grep`` по маркерам. Выход -- строки ``ключ=значение`` без пробелов.

Маркеры
=======
v2 (``dialog_engine: v2``):

* ``✅ ПРИНЯТО (...)`` -- фраза принята STT;
* ``📊 [turn] ... stage=done|failed|cancelled|silenced`` -- агрегат хода (``rob_box_dialog.turn_log.AGGREGATE_MARK``).
  Все -- КОНЕЦ хода (#3954): ``failed`` не заставляет харнесс ждать потолок, вердикт ``failed`` несёт причину
  (``status``/``reason``/``llm_calls`` из той же строки). ``cancelled`` -- конец хода, кроме шага-перебивателя
  (``--barge``): там «отменён» -- след ПРОШЛОЙ фразы, и концом хода этого шага он быть не может. ``silenced`` -- ход
  проглотила тишина (#3982: LLM не зовётся, речи нет);
* ``[backlog] accumulated reason=no_wake`` -- фраза принята без вейка и ушла в бэклог: хода не будет (#3989);
* ``🧭 [turn-v2] tier1 kind=<вид>`` -- ход Tier-1 (стоп, тишина, «говори», громкость...);
* ``🗣️ SpeechEvent <kind> speech_id=<8> ...`` -- события владельца речи (``speech_owner._emit``): ``queued`` /
  ``synthesized`` / ``started`` / ``progress`` открывают или ведут реплику, ``finished`` / ``cancelled`` /
  ``rejected`` закрывают её. Пара -- по ``speech_id``. Каждый куплет выступления -- отдельный ``SpeechRequest``
  со своим ``speech_id``, значит и свой ``SpeechEvent started``.

v1: ``🔊 TTS: speech_id=<8>`` открывает реплику, ``Публикую TTS finished event: speech_id=<8>`` (и вариант
``(ошибка)``) закрывает.

В строке ``SpeechEvent`` нет ``turn_id`` и идентификатора выступления: «куплет именно этого выступления»
отличить от обычной реплики по логу нельзя. Окно лога ограничено фразой шага, поэтому ``segment_started``
считается как N-й ``started`` в окне (см. :func:`anchor`).

Восстановление шага (#3989)
===========================
Вердикты ``no_wake`` и ``silenced`` -- не «робот не ответил», а «до хода дело не дошло»: вейк потерян в STT либо
тишина, оставленная прошлым шагом/актом. Харнесс делает ОДНО восстановление (:data:`RECOVERY`) и помечает шаг
``OK recovered_<вид>``; какие виды разрешены шагу -- :func:`recover_kinds` (поле ``recover`` сценария). Тишину на
старте акта харнесс читает из снимка ``/dialog/session`` (:func:`session_report`).
"""

from __future__ import annotations

import json
import re
import sys
from types import MappingProxyType
from typing import Any, Callable, Dict, List, Mapping, NamedTuple, Optional, Tuple

TS_RE = re.compile(r"\[(\d{9,}\.\d+)\]")
SPEECH_V2 = re.compile(r"SpeechEvent (queued|synthesized|started|progress|finished|cancelled|rejected) speech_id=([0-9A-Za-z]+)")
V1_OPEN = re.compile(r"🔊 TTS: speech_id=([0-9a-f]{8})")
V1_CLOSE = re.compile(r"TTS finished event(?: \(ошибка\))?: speech_id=([0-9a-f]{8})")
OPENING = frozenset({"queued", "synthesized", "started"})
CLOSING = frozenset({"finished", "cancelled", "rejected"})
TURN_RE = re.compile(r"📊 \[turn\].*?\bstage=(\w+)")
TURN_FIELDS = ("status", "reason", "llm_calls")
#: Ход, проглоченный тишиной: ``rob_box_dialog.knowledge.SILENCED_TURN_STAGE`` (#3982; drift держит тест #3989).
SILENCED = "silenced"
TURN_END = frozenset({"done", "failed", "cancelled", SILENCED})
NO_WAKE_RE = re.compile(r"\[backlog\] accumulated reason=no_wake\b")
TIER1_RE = re.compile(r"\btier1 kind=(\w+)")
#: Tier-1 «замолчи» (ключ ``rob_box_dialog.knowledge.TIER1_KINDS``; drift держит тест #3989).
SILENCE_TIER1 = "silence"
#: Сколько секунд после конца фразы шага ждать, прежде чем «ушла в бэклог без вейка» окончательно: STT мог порезать
#: фразу, и следующий кусок ещё начнёт ход (#3989).
NO_WAKE_SETTLE_S = 4.0
#: Сцена тишины в снимке сессии: ``rob_box_core.voice_state.SILENCE_SCENE`` (drift держит тест #3989).
SILENCE_SCENE = "Silence"
STALE_S = 60.0
MAX_SKEW_S = 3600.0
ANCHORS = ("speech_started", "llm_request", "segment_started")


def _ts(line: str) -> Optional[float]:
    found = TS_RE.search(line)
    return float(found.group(1)) if found else None


def split_input(text: str) -> Tuple[Optional[float], List[str]]:
    """``(now робота, строки лога)``; нет числа в первой строке -- ``now=None`` (чтение не удалось)."""
    lines = text.splitlines()
    if not lines:
        return None, []
    try:
        return float(lines[0].strip()), lines[1:]
    except ValueError:
        return None, lines


def speech_event(line: str, engine: str) -> Optional[Tuple[str, str]]:
    """``(вид, speech_id)`` события речи в строке или ``None``. Вид: queued/started/progress/finished/..."""
    if engine == "v2":
        found = SPEECH_V2.search(line)
        return (found.group(1), found.group(2)) if found else None
    opened = V1_OPEN.search(line)
    if opened:
        return "started", opened.group(1)
    closed = V1_CLOSE.search(line)
    return ("finished", closed.group(1)) if closed else None


class Window:
    """Состояние окна лога: принято ли, закончен ли ход, какие реплики открыты."""

    def __init__(self, engine: str, barge: bool = False) -> None:
        self.engine = engine
        self.barge = barge
        self.accepted = False
        self.turn_done = False
        self.turn_stage = ""  # стадия, которой закончился ход: done/failed/cancelled/silenced
        self.turn_info: Dict[str, str] = {}
        self.llm = 0
        self.started = 0
        self.ends = 0  # реплики, открытые в окне и закрытые в нём
        self.opened: Dict[str, Optional[float]] = {}  # speech_id -> ts последнего события
        self.closed: set = set()
        self.last_ts: Optional[float] = None  # последнее событие речи
        self.events = 0
        self.no_wake = False  # принятая фраза ушла в бэклог без вейка (#3989)
        self.tier1: List[str] = []  # виды ходов Tier-1 после принятой фразы

    def feed(self, line: str) -> None:
        if "ПРИНЯТО" in line and not self.accepted:
            self.accepted = True
        if self.accepted:
            self._after_accept(line)
        event = speech_event(line, self.engine)
        if event:
            self._speech(event[0], event[1], _ts(line))

    def _after_accept(self, line: str) -> None:
        if "📊 [turn]" in line:
            self._turn(line)
        if "LLM REQUEST START" in line:
            self.llm += 1
        if NO_WAKE_RE.search(line):
            self.no_wake = True
        tier1 = TIER1_RE.search(line)
        if tier1:
            self.tier1.append(tier1.group(1))

    def _turn(self, line: str) -> None:
        found = TURN_RE.search(line)
        stage = found.group(1) if found else ""
        if stage not in TURN_END or (stage == "cancelled" and self.barge) or self.turn_stage == "failed":
            return
        self.turn_done, self.turn_stage = True, stage
        self.turn_info = {k: v for k, v in (m.groups() for m in re.finditer(r"\b(status|reason|llm_calls)=(\S+)", line))
                          if k in TURN_FIELDS}

    def _speech(self, kind: str, speech_id: str, ts: Optional[float]) -> None:
        self.events += 1
        if ts is not None:
            self.last_ts = ts
        if kind in CLOSING:
            if speech_id in self.opened:
                self.opened.pop(speech_id)
                self.closed.add(speech_id)
                self.ends += 1
            return
        if speech_id in self.closed:
            return
        if kind == "started":
            self.started += 1
        self.opened[speech_id] = ts if ts is not None else self.opened.get(speech_id)

    def busy(self) -> bool:
        """В окне есть ход: его конец, запрос к LLM, Tier-1 или речь."""
        return bool(self.turn_done or self.llm or self.tier1 or self.started or self.opened)

    def open_live(self, now: Optional[float]) -> int:
        """Открытые реплики; «протухшие» (последнее событие старше :data:`STALE_S`) не считаются говорящими."""
        if now is None:
            return len(self.opened)
        return sum(1 for ts in self.opened.values() if ts is None or now - ts <= STALE_S)

    def quiet_for(self, now: Optional[float]) -> float:
        """Секунд с последнего события речи; ``-1`` -- не определить (нет времени или часы разошлись)."""
        if now is None or self.last_ts is None:
            return -1.0
        gap = now - self.last_ts
        return gap if 0 <= gap <= MAX_SKEW_S else -1.0


def scan(text: str, engine: str, barge: bool = False) -> Tuple[Window, Optional[float]]:
    now, lines = split_input(text)
    window = Window(engine, barge)
    for line in lines:
        window.feed(line)
    return window, now


def _no_wake(window: Window, engine: str, waited: float) -> bool:
    """v2: фраза принята, ушла в бэклог без вейка, хода нет -- и STT за :data:`NO_WAKE_SETTLE_S` его не начал."""
    return engine == "v2" and window.no_wake and not window.busy() and waited >= NO_WAKE_SETTLE_S


def reply_verdict(window: Window, engine: str, silent_ok: bool, waited: float = 0.0, accept_window: float = 0.0) -> str:
    """``done`` -- ответ состоялся и робот замолчал; ``nospeech`` -- ход закончен, речи не было (ждать льготу);
    ``silent`` -- ход закончен без речи и шаг это разрешает; ``failed`` / ``cancelled`` -- ход закончен сбоем /
    отменой (v2, #3954: терминально, ждать нечего); ``no_wake`` / ``silenced`` -- до хода не дошло: фраза ушла в бэклог
    без вейка / ход проглотила тишина (v2, #3989: терминально, харнесс решает о восстановлении шага; ``silent_ok`` их
    не прощает -- ответа на фразу шага не было); ``no_accept`` -- фразу STT не принял за ``accept_window`` секунд
    (переигрывать, не ждать потолок); ``wait`` -- рано."""
    if not window.accepted:
        if accept_window > 0 and waited >= accept_window:
            return "no_accept"
        return "wait"
    if _no_wake(window, engine, waited):
        return "no_wake"
    if window.opened:
        return "wait"
    if engine == "v2":
        return _turn_verdict(window, silent_ok)
    return "done" if window.ends else "wait"


def _turn_verdict(window: Window, silent_ok: bool) -> str:
    """v2: вердикт по концу хода. Ход, проглоченный тишиной, которую включил САМ шаг («хватит, замолчи, посчитай»:
    ``tier1 kind=silence``, остаток фразы -- уже в тишине), -- не утечка, а ход без речи по просьбе шага."""
    if not window.turn_done:
        return "wait"
    if window.turn_stage in ("failed", "cancelled"):
        return window.turn_stage
    if window.turn_stage == SILENCED and SILENCE_TIER1 not in window.tier1:
        return SILENCED
    if window.ends:
        return "done"
    if window.started:
        return "wait"
    return "silent" if silent_ok else "nospeech"


def state(text: str, engine: str, silent_ok: bool, barge: bool = False, waited: float = 0.0,
          accept_window: float = 0.0) -> Dict[str, object]:
    window, now = scan(text, engine, barge)
    live = window.open_live(now)
    if window.opened and live == 0:  # все открытые реплики протухли: говорящих нет
        window.opened.clear()
    verdict = reply_verdict(window, engine, silent_ok, waited, accept_window)
    info = window.turn_info
    return {
        "reason": "no_wake" if verdict == "no_wake" else _reason(window), "llm_calls": info.get("llm_calls", "-"),
        "fetch": 0 if now is None else 1, "verdict": verdict, "accepted": int(window.accepted),
        "turn_done": int(window.turn_done), "started": window.started, "ends": window.ends,
        "open": live, "llm": window.llm, "quiet_for": f"{window.quiet_for(now):.2f}",
        "sig": f"{window.events}.{window.started}.{window.ends}.{int(window.turn_done)}",
    }


def _reason(window: Window) -> str:
    """Причина конца хода из строки ``📊 [turn]`` без пробелов: ``failed:status=failed,llm_calls=3``; ``-`` если ход не кончился."""
    if not window.turn_stage:
        return "-"
    parts = [f"{k}={v}" for k, v in window.turn_info.items() if k != "llm_calls"]
    return window.turn_stage + (":" + ",".join(parts) if parts else "")


def quiet(text: str, engine: str) -> Dict[str, object]:
    window, now = scan(text, engine)
    return {"fetch": 0 if now is None else 1, "open": window.open_live(now),
            "quiet_for": f"{window.quiet_for(now):.2f}", "sig": f"{window.events}"}


def anchor(text: str, engine: str, after: str, n: int) -> Dict[str, object]:
    """N-й ``SpeechEvent started`` (``speech_started``/``segment_started``) или N-я ``LLM REQUEST START``
    (``llm_request``) ПОСЛЕ первой принятой фразы окна. Метка времени -- из лога робота (0, если нет)."""
    now, lines = split_input(text)
    accepted, count = False, 0
    for line in lines:
        if "ПРИНЯТО" in line:
            accepted = True
            continue
        if not accepted:
            continue
        hit, speech_id = _anchor_hit(line, engine, after)
        if not hit:
            continue
        count += 1
        if count >= n:
            ts = _ts(line)
            return {"fetch": 1, "found": 1, "ts": f"{ts or 0:.3f}", "speech_id": speech_id or "-",
                    "now": f"{now or 0:.3f}"}
    return {"fetch": 0 if now is None else 1, "found": 0, "ts": "0", "speech_id": "-", "now": f"{now or 0:.3f}"}


def _anchor_hit(line: str, engine: str, after: str) -> Tuple[bool, Optional[str]]:
    if after == "llm_request":
        return ("LLM REQUEST START" in line), None
    event = speech_event(line, engine)
    if event and event[0] == "started":
        return True, event[1]
    return False, None


# -- восстановление шага (#3989) --------------------------------------------------------------------------


class Recovery(NamedTuple):
    """Что харнесс говорит роботу и повторяет ли потом фразу шага."""

    phrase: str
    replay: bool


#: ОДНО восстановление шага (#3989, решения Шифу 23.10 и 24.10) -- так сделал бы человек. ``no_wake`` -- фраза ушла
#: в бэклог без вейка: голое «Робот!» подхватывает свежий бэклог (В12, ``pickup``: фразы не старше
#: ``BACKLOG_PICKUP_S``), фразу шага не повторяем -- ответ на неё придёт по подхвату. ``silenced`` -- ход проглотила
#: тишина: снимает её только «Робот, говори» (вейк тишину не снимает), затем фраза шага ещё раз.
RECOVERY: Mapping[str, Recovery] = MappingProxyType({
    "no_wake": Recovery("Робот!", False),
    SILENCED: Recovery("Робот, говори.", True),
})
RECOVERABLE: Tuple[str, ...] = tuple(RECOVERY)


def _norm(text: str) -> str:
    """Слова фразы в нижнем регистре без пунктуации: «Робот, говори.» -> «робот говори»."""
    return " ".join(re.findall(r"\w+", str(text).lower()))


def _kinds(raw: Any) -> Tuple[str, ...]:
    """Поле ``recover``: ``true`` -- все виды, ``false``/``[]`` -- никаких, список -- только эти."""
    if raw is True:
        return RECOVERABLE
    if raw is False:
        return ()
    if isinstance(raw, list) and all(isinstance(k, str) and k in RECOVERY for k in raw):
        return tuple(k for k in RECOVERABLE if k in raw)
    raise ValueError(f"recover: ожидается true|false|список из {list(RECOVERABLE)}, а не {raw!r}")


def recover_kinds(scenario: Mapping[str, Any], step: Mapping[str, Any]) -> Tuple[str, ...]:
    """Какие восстановления разрешены шагу. Поле ``recover`` шага сильнее поля акта; нет ни того ни другого -- все.

    Кодом, без поля: шаг-перебиватель (``barge_in``) не восстанавливается -- речи, которую он перебивал, после
    восстановления уже нет, подхват бэклога встанет в очередь после неё; шаг, чья фраза и есть восстановление (n805b
    «Робот, говори.»), этим восстановлением не лечится -- иначе харнесс сделал бы шаг за робота."""
    kinds = _kinds(step["recover"] if "recover" in step else scenario.get("recover", True))
    if step.get("barge_in"):
        return ()
    own = _norm(step.get("text", ""))
    return tuple(k for k in kinds if _norm(RECOVERY[k].phrase) != own)


def recover_policy(scenario: Any) -> List[Tuple[str, Tuple[str, ...]]]:
    """``[(label, виды)]`` по шагам сценария; ошибка схемы поля ``recover`` -- ``ValueError``."""
    if not isinstance(scenario, Mapping):
        raise ValueError("сценарий -- не JSON-объект")
    _kinds(scenario.get("recover", True))
    steps = [s for s in scenario.get("steps", []) if isinstance(s, Mapping)]
    return [(str(s.get("label", f"s{i + 1}")), recover_kinds(scenario, s)) for i, s in enumerate(steps)]


# -- тишина в снимке сессии (#3989 п.5) -------------------------------------------------------------------


def silence_frame(scenes: Optional[List[Mapping[str, Any]]], now: float) -> Optional[Mapping[str, Any]]:
    """Живой кадр тишины в снимке ``/dialog/session`` или ``None``. Кадр с истёкшим ``ttl_until`` -- не тишина:
    владелец снимает истёкшие кадры лениво (``Session.expire`` на следующем ходе), latched-снимок остаётся прежним."""
    for scene in scenes or ():
        if scene.get("kind") != SILENCE_SCENE:
            continue
        ttl = float(scene.get("ttl_until") or 0.0)
        if ttl <= 0 or ttl > now:
            return scene
    return None


def _describe(frame: Optional[Mapping[str, Any]], now: float) -> str:
    if frame is None:
        return "тишины нет"
    ttl = float(frame.get("ttl_until") or 0.0)
    left = f"{int(ttl - now)}s" if ttl > 0 else "-"
    return f"{SILENCE_SCENE} set_by={frame.get('set_by') or '?'} ttl_left={left}"


def session_report(report: Any) -> Dict[str, object]:
    """Ответ рабочего процесса харнесса (``snapshot``, ``scenes``, ``now``; после реплики снятия -- ``after``) ->
    ``fetch`` (1 -- ответ разобран), ``silenced``, ``after`` (``-`` -- снятия не было), ``detail``.
    Нет latched-снимка -- тишины нет: тишина -- изменение сессии, а каждое изменение публикуется."""
    if not isinstance(report, Mapping) or "snapshot" not in report:
        return {"fetch": 0, "silenced": 0, "after": "-", "detail": "нет ответа"}
    now = float(report.get("now") or 0.0)
    frame = silence_frame(report.get("scenes"), now)
    after = report.get("after", "-")
    if after != "-":
        after = int(silence_frame(after, now) is not None)
    detail = _describe(frame, now) if report.get("snapshot") else "снимка нет (сессию с запуска ноды не меняли)"
    return {"fetch": 1, "silenced": int(frame is not None), "after": after, "detail": detail}


# -- CLI ---------------------------------------------------------------------------------------------------


def _emit(values: Dict[str, object]) -> None:
    for key, value in values.items():
        sys.stdout.write(f"{key}={value}\n")


def _num(value: str) -> float:
    try:
        return float(value)
    except ValueError:
        return 0.0


def _cmd_state(opts: Dict[str, str], flags: List[str]) -> int:
    _emit(state(sys.stdin.read(), opts["engine"], "--silent-ok" in flags, "--barge" in flags, _num(opts["--waited"]),
                _num(opts["--accept-window"])))
    return 0


def _cmd_quiet(opts: Dict[str, str], flags: List[str]) -> int:
    _emit(quiet(sys.stdin.read(), opts["engine"]))
    return 0


def _cmd_anchor(opts: Dict[str, str], flags: List[str]) -> int:
    try:
        n = max(1, int(opts["--n"]))
    except ValueError:
        n = 1
    _emit(anchor(sys.stdin.read(), opts["engine"], opts["--after"], n))
    return 0


def _cmd_policy(opts: Dict[str, str], flags: List[str]) -> int:
    """``label<TAB>no_wake,silenced`` (``-`` -- нельзя) по шагам; ошибка схемы -- rc 2 и причина в stderr."""
    try:
        with open(opts["--scenario"], encoding="utf-8") as fh:
            rows = recover_policy(json.load(fh))
    except (OSError, ValueError) as exc:
        sys.stderr.write(f"{opts['--scenario']}: {exc}\n")
        return 2
    for label, kinds in rows:
        sys.stdout.write(f"{label}\t{','.join(kinds) or '-'}\n")
    return 0


def _cmd_recovery(opts: Dict[str, str], flags: List[str]) -> int:
    """``--kind K`` -> ``phrase=`` / ``replay=``; без вида -- ``kinds=`` (все виды)."""
    recovery = RECOVERY.get(opts["--kind"])
    if recovery is None:
        _emit({"kinds": ",".join(RECOVERABLE)})
        return 0 if not opts["--kind"] else 2
    _emit({"phrase": recovery.phrase, "replay": int(recovery.replay)})
    return 0


def _cmd_session(opts: Dict[str, str], flags: List[str]) -> int:
    try:
        report = json.loads(sys.stdin.read())
    except ValueError:
        report = None
    _emit(session_report(report))
    return 0


COMMANDS: Mapping[str, Callable[[Dict[str, str], List[str]], int]] = MappingProxyType({
    "state": _cmd_state, "quiet": _cmd_quiet, "anchor": _cmd_anchor,
    "policy": _cmd_policy, "recovery": _cmd_recovery, "session": _cmd_session,
})
USAGE = ("usage: e2e_events.py state|quiet|anchor --engine v1|v2 [--silent-ok] [--barge] [--waited S --accept-window S]"
         " [--after A --n N] | policy --scenario FILE | recovery [--kind K] | session < report.json\n")


def main(argv: List[str]) -> int:
    if len(argv) < 2 or argv[1] not in COMMANDS:
        sys.stderr.write(USAGE)
        return 2
    for stream in (sys.stdin, sys.stdout):  # фразы и метки -- кириллица; Windows-консоль по умолчанию не utf-8
        try:
            stream.reconfigure(encoding="utf-8", newline="\n" if stream is sys.stdout else None)
        except (AttributeError, ValueError):
            pass
    opts = {"--engine": "v1", "--after": "speech_started", "--n": "1", "--waited": "0", "--accept-window": "0",
            "--scenario": "", "--kind": ""}
    flags = argv[2:]
    for i, tok in enumerate(flags):
        if tok in opts and i + 1 < len(flags):
            opts[tok] = flags[i + 1]
    opts["engine"] = "v2" if opts["--engine"] == "v2" else "v1"
    return COMMANDS[argv[1]](opts, flags)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
