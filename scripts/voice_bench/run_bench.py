#!/usr/bin/env python3
"""Прогон бенчмарка на живом роботе: реплика в топик, ответ из лога.

Запускается ВНУТРИ контейнера ``voice-assistant`` (там есть rclpy и
ROS-окружение):

    docker exec voice-assistant bash -lc \\
      "source /ws/install/setup.bash && python3 /config/voice_bench/run_bench.py \\
       --cases /config/voice_bench/cases.yaml --out /tmp/bench_on.json --repeat 3"

Как устроено. Реплика публикуется в ``/voice/stt/result`` — тот же топик,
в который пишет ``stt_node``, поэтому робот обрабатывает её как обычную
речь (wake-word в тексте обязателен, иначе сработает gate). Результат
берётся из лог-файла ноды: там лежит и ``spoken``, и список вызванных
инструментов, и предупреждения о ретраях — то есть вся траектория, а не
только финальная фраза. Оценка по траектории — принцип τ-bench и BFCL.

Скрипт НИЧЕГО не оценивает: он собирает сырьё. Баллы ставит ``score.py``,
чтобы правила оценки можно было менять и перепроверять, не трогая прогон.

Три грабли, на которые он наступил на живом роботе — не наступайте снова:

1. ``ros2 topic pub --once`` теряет сообщение: одноразовый паблишер
   закрывается раньше, чем подписчик его обнаружит. Отсюда ``Speaker``.
2. ``TextIOWrapper.seek`` принимает не байты, а cookie от ``tell()`` —
   чтение с байтового смещения молча уходило в никуда.
3. Ходы РОБОТА идут вперемешку с нашими: DJ-режим сам генерирует
   переходы каждые 45-75 секунд. Ждать «первое завершение после
   публикации» нельзя — кейс заберёт чужой ход. Корреляция строго по
   ``user_input`` в строке ``handle_result``.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path
from typing import Any

LOG_DIR = Path("/root/.ros/log")

#: Строка с результатом хода, несущая И ответ, И исходную реплику —
#: единственная в логе, по которой ход можно связать со своей репликой.
RESULT_RE = re.compile(
    r"\[handle_result\] spoken=(?P<spoken>.*?) \(len=\d+\) "
    r"tools=(?P<tools>\[.*?\]) user_input='(?P<user>.*?)'",
    re.DOTALL,
)
TOOLCALL_RE = re.compile(r"ToolCall\(id='[^']*', name='(?P<name>\w+)'")

#: Ходы, которые робот генерирует сам (автопереходы диджея). Их нельзя
#: путать с ответами на наши реплики.
_ROBOT_OWN = ("[DJ_AUTO", "[SYSTEM CORRECTION")

_WAKE_WORDS = ("робот", "робокс", "робобокс")

#: Фраза, которой глушим музыку, если кейс оставил диджея включённым.
QUIET_PHRASE = "робот хватит диджеить выключи музыку"


def _tail(path: Path, size: int) -> str:
    try:
        with path.open("rb") as fh:
            fh.seek(0, 2)
            fh.seek(max(0, fh.tell() - size))
            return fh.read().decode("utf-8", "replace")
    except OSError:
        return ""


def find_log() -> Path:
    """Файл, в который пишет dialogue_node прямо сейчас.

    Цепляемся за имя ноды, а не за наличие ходов: сразу после рестарта
    ходов ещё нет, и поиск «по ходам» привязался бы к прошлой сессии.
    """
    candidates = [
        p for p in LOG_DIR.glob("*.log") if "[dialogue_node]" in _tail(p, 200_000)
    ]
    if not candidates:
        raise SystemExit(
            "не найден лог dialogue_node в /root/.ros/log — нода поднялась?"
        )
    return max(candidates, key=lambda p: p.stat().st_mtime)


def _read_from(log: Path, offset: int) -> str:
    """Прочитать хвост файла с байтового смещения (строго бинарно)."""
    with log.open("rb") as fh:
        fh.seek(offset)
        return fh.read().decode("utf-8", "replace")


def _key_of(phrase: str) -> str:
    """Отпечаток реплики, по которому её видно в ``user_input``.

    Нода печатает реплику без wake-word и обрезает длинные — поэтому
    берём начало фразы, а не всю.
    """
    words = phrase.split()
    if words and words[0].lower().strip(",") in _WAKE_WORDS:
        words = words[1:]
    return " ".join(words).strip()[:28]


class Speaker:
    """Публикует реплики в ``/voice/stt/result`` от лица «распознанной речи»."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._String = String
        rclpy.init()
        self.node = rclpy.create_node("voice_bench_speaker")
        self.pub = self.node.create_publisher(
            String,
            "/voice/stt/result",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.pub.get_subscription_count() > 0:
                return True
            self._rclpy.spin_once(self.node, timeout_sec=0.2)
        return False

    def say(self, text: str) -> None:
        self.pub.publish(self._String(data=text))
        for _ in range(6):
            self._rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self) -> None:
        self.node.destroy_node()
        self._rclpy.shutdown()


def _match_ours(chunk: str, key: str) -> re.Match[str] | None:
    """Найти ход, который отвечает ИМЕННО на нашу реплику."""
    for m in RESULT_RE.finditer(chunk):
        user = m.group("user")
        if any(user.startswith(marker) for marker in _ROBOT_OWN):
            continue  # ход робота о себе самом (автопереход диджея)
        if key and key[:20] in user:
            return m
    return None


def _stats(text: str) -> dict[str, Any]:
    return {
        "llm_requests": _llm_calls(text),
        "truncated": text.count("TRUNCATED_TOOL_ARGS"),
        "retry_no_music": text.count("В прошлом цикле ты НЕ вызвал"),
        "provider_fallback": text.count("UNCLASSIFIED failure"),
        "suppressed_speech": text.count("issue 1708"),
        "dj_auto_turns": text.count("DJ auto-transition"),
        "skill": _skill(text),
    }


def collect(log: Path, offset: int, timeout: float, phrase: str) -> dict[str, Any]:
    """Дождаться завершения НАШЕГО хода и вытащить его траекторию."""
    key = _key_of(phrase)
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = _read_from(log, offset)
        m = _match_ours(chunk, key)
        if m:
            body = chunk[: m.end()]
            return {
                "spoken": m.group("spoken").strip().strip("'"),
                "tools": re.findall(r"'(\w+)'", m.group("tools")),
                "tool_calls": TOOLCALL_RE.findall(body),
                "error": None,
                "raw": body[-4000:],
                **_stats(body),
            }
        time.sleep(0.7)

    chunk = _read_from(log, offset)
    delivered = key[:20] in chunk
    return {
        "spoken": "",
        "tools": [],
        "tool_calls": [],
        "error": "TIMEOUT" if delivered else "NOT_DELIVERED",
        "raw": chunk[-4000:],
        **_stats(chunk),
    }


def _llm_calls(text: str) -> int:
    """Сколько раз ход ходил в LLM.

    ``LLM REQUEST START`` печатается в stdout контейнера, а не в лог ноды,
    поэтому считаем по строке health-обёртки: она пишется штатным
    логгером на каждое обращение, включая ретраи и уход на запасного.
    """
    return max(
        1, len(re.findall(r"streaming from provider|complete from provider", text))
    )


def _skill(text: str) -> str:
    """Какой доменный скилл оказался активен на ходу (если скиллы включены)."""
    found = re.findall(r"\[СКИЛЛ ([\w-]+)", text)
    return found[-1] if found else "none"


def _wait_idle(log: Path, limit: float = 25.0) -> None:
    """Подождать, пока лог перестанет расти: ход и его TTS доиграли."""
    deadline = time.time() + limit
    last = -1
    while time.time() < deadline:
        size = log.stat().st_size
        if size == last:
            return
        last = size
        time.sleep(1.5)


def _quiet_down(speaker: Speaker, log: Path, turn: dict[str, Any]) -> bool:
    """Заглушить диджея, если кейс оставил его включённым.

    Иначе автопереходы диджея идут вперемешку с остальными кейсами и
    портят весь дальнейший прогон — так уехал первый прогон со
    скиллами выключенными.
    """
    if "set_dj_mode" not in turn.get("tools", []) and not turn.get("dj_auto_turns"):
        return False
    speaker.say(QUIET_PHRASE)
    time.sleep(4)
    _wait_idle(log)
    return True


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cases", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--repeat", type=int, default=1, help="повторов кейса (pass^k)")
    ap.add_argument("--timeout", type=float, default=60.0, help="ожидание хода, сек")
    ap.add_argument("--pause", type=float, default=4.0, help="пауза между кейсами, сек")
    ap.add_argument("--only", help="id кейсов через запятую")
    ap.add_argument("--label", default="", help="метка конфигурации в отчёте")
    args = ap.parse_args(argv)

    import yaml

    cases = yaml.safe_load(args.cases.read_text(encoding="utf-8"))
    if args.only:
        wanted = {s.strip() for s in args.only.split(",")}
        cases = [c for c in cases if str(c["id"]) in wanted]

    log = find_log()
    print(f"лог ноды: {log}", flush=True)

    speaker = Speaker()
    if not speaker.wait_ready():
        print("⚠️  подписчика на /voice/stt/result нет — dialogue_node жив?", flush=True)

    turns: list[dict[str, Any]] = []
    for rep in range(1, args.repeat + 1):
        for case in cases:
            _wait_idle(log)
            offset = log.stat().st_size
            print(f"[{rep}/{args.repeat}] {case['id']}: {case['say']}", flush=True)
            speaker.say(str(case["say"]))
            turn = collect(log, offset, args.timeout, str(case["say"]))
            turn.update(case_id=str(case["id"]), repeat=rep, say=str(case["say"]))
            turns.append(turn)
            print(
                f"    → tools={turn['tools']} spoken={turn['spoken'][:48]!r} "
                f"skill={turn['skill']} llm×{turn['llm_requests']}"
                + ("  [глушу диджея]" if _quiet_down(speaker, log, turn) else ""),
                flush=True,
            )
            time.sleep(args.pause)

    speaker.close()
    payload = {
        "config": {
            "label": args.label,
            "repeat": args.repeat,
            "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
        "turns": turns,
    }
    args.out.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"\nсырьё записано: {args.out} ({len(turns)} ходов)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
