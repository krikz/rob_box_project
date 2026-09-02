#!/usr/bin/env python3
"""Прогон бенчмарка на живом роботе: реплика в топик, ответ из лога.

Запускается ВНУТРИ контейнера ``voice-assistant`` (там есть rclpy и
ROS-окружение):

    docker exec voice-assistant bash -lc \\
      "source /ws/install/setup.bash && python3 /config/voice_bench/run_bench.py \\
       --cases /config/voice_bench/cases.yaml --out /tmp/bench_on.json --repeat 3"

Как устроено. Реплика публикуется в ``/voice/stt/result`` — тот же топик,
в который пишет stt_node, поэтому робот обрабатывает её как обычную речь
(wake-word в тексте обязателен, иначе сработает gate). Результат хода
берётся из лог-файла ноды: там лежит и ``spoken``, и список вызванных
инструментов, и предупреждения о ретраях — то есть вся траектория, а не
только финальная фраза. Оценка по траектории, а не по словам — принцип
τ-bench и BFCL.

Скрипт НИЧЕГО не оценивает: он собирает сырьё. Баллы ставит ``score.py``,
чтобы правила оценки можно было менять и перепроверять, не трогая прогон.
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
#: Хвост строки завершения хода в разных сборках разный: где-то
#: ``error=None``, где-то ``finish_reason='stop'``. Цепляемся за то, что
#: стабильно — spoken и tools, — а хвост разбираем отдельно.
TURN_RE = re.compile(
    r"process_input returned: spoken=(?P<spoken>.*?)\[:60\] "
    r"tools=(?P<tools>\[.*?\])(?P<tail>.*)"
)
TOOLCALL_RE = re.compile(r"ToolCall\(id='[^']*', name='(?P<name>\w+)'")


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


def _tail(path: Path, size: int) -> str:
    try:
        with path.open("rb") as fh:
            fh.seek(0, 2)
            fh.seek(max(0, fh.tell() - size))
            return fh.read().decode("utf-8", "replace")
    except OSError:
        return ""


class Speaker:
    """Публикует реплики в ``/voice/stt/result`` от лица «распознанной речи».

    Через ``ros2 topic pub --once`` это не работает: одноразовый паблишер
    успевает закрыться раньше, чем подписчик его обнаружит, и сообщение
    теряется молча (первые прогоны бенчмарка собрали нули именно так).
    Поэтому свой узел, живущий весь прогон, и явное ожидание подписчика.
    """

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


def _read_from(log: Path, offset: int) -> str:
    """Прочитать хвост файла с байтового смещения.

    Строго в бинарном режиме: ``TextIOWrapper.seek`` принимает не байты, а
    непрозрачный cookie от ``tell()``, и произвольное смещение уводит
    чтение в никуда — на этом первый прогон бенчмарка молча собрал нули.
    """
    with log.open("rb") as fh:
        fh.seek(offset)
        return fh.read().decode("utf-8", "replace")


def _key_of(phrase: str) -> str:
    """Отпечаток реплики, по которому её видно в логе.

    Нода печатает ``user_input`` уже без wake-word и с префиксом
    ``[Speaker:...]``, поэтому цепляемся за остаток фразы.
    """
    words = phrase.split()
    if words and words[0].lower().strip(",") in ("робот", "робокс", "робобокс"):
        words = words[1:]
    return " ".join(words).strip()


def collect(log: Path, offset: int, timeout: float, phrase: str) -> dict[str, Any]:
    """Дождаться завершения ИМЕННО НАШЕГО хода и вытащить его траекторию.

    Ждать «первое завершение после публикации» нельзя: если предыдущий
    ход ещё доигрывал, кейс забирал ЕГО результат, а свой оставлял
    следующему. Первый полный прогон на роботе именно так и уехал —
    ответы сдвинулись на один кейс, и таблица стала мусором, выглядящим
    правдоподобно. Поэтому сначала ищем в логе свою реплику, и только
    после неё — завершение хода.
    """
    key = _key_of(phrase)
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = _read_from(log, offset)
        start = chunk.find(key)
        if start != -1:
            mine = chunk[start:]
            m = TURN_RE.search(mine)
            if m:
                return _parse(mine, m)
        time.sleep(0.7)
    chunk = _read_from(log, offset)
    if key not in chunk:
        chunk = f"[реплика не дошла до ноды] {chunk}"
    return {
        "spoken": "",
        "tools": [],
        "error": "NOT_DELIVERED" if "не дошла" in chunk else "TIMEOUT",
        "llm_requests": _llm_calls(chunk),
        "truncated": chunk.count("TRUNCATED_TOOL_ARGS"),
        "retry_no_music": chunk.count("В прошлом цикле ты НЕ вызвал"),
        "provider_fallback": chunk.count("UNCLASSIFIED failure"),
        "suppressed_speech": chunk.count("issue 1708"),
        "skill": _skill(chunk),
        "raw": chunk[-4000:],
    }


def _parse(chunk: str, m: re.Match[str]) -> dict[str, Any]:
    body = chunk[: m.end()]
    spoken = m.group("spoken").strip().strip("'")
    tools = re.findall(r"'(\w+)'", m.group("tools"))
    return {
        "spoken": spoken,
        "tools": tools,
        "tool_calls": TOOLCALL_RE.findall(body),
        "error": _error_of(m.group("tail")),
        "llm_requests": _llm_calls(body),
        "truncated": body.count("TRUNCATED_TOOL_ARGS"),
        "retry_no_music": body.count("В прошлом цикле ты НЕ вызвал"),
        "provider_fallback": body.count("UNCLASSIFIED failure"),
        "suppressed_speech": body.count("issue 1708"),
        "skill": _skill(body),
        "raw": body[-4000:],
    }


def _llm_calls(text: str) -> int:
    """Сколько раз ход ходил в LLM.

    ``LLM REQUEST START`` печатается в stdout контейнера, а не в лог ноды,
    поэтому считаем по строке health-обёртки: она пишется штатным
    логгером на каждое обращение, включая ретраи и уход на запасного
    провайдера.
    """
    return max(1, len(re.findall(r"streaming from provider|complete from provider", text)))


def _error_of(tail: str) -> str | None:
    """Ошибка хода из хвоста строки, если сборка её туда пишет."""
    m = re.search(r"error=(\S+)", tail)
    if not m or m.group(1).rstrip(",") == "None":
        return None
    return m.group(1).rstrip(",")


def _skill(text: str) -> str:
    """Какой доменный скилл оказался активен на ходу (если скиллы включены)."""
    found = re.findall(r"\[СКИЛЛ ([\w-]+)", text)
    return found[-1] if found else "none"


def _wait_idle(log: Path, extra: float, limit: float = 25.0) -> None:
    """Подождать, пока лог перестанет расти: ход и его TTS доиграли.

    Без этого следующая реплика уходит роботу, пока он ещё говорит
    предыдущую, и ходы наслаиваются.
    """
    deadline = time.time() + limit
    last = -1
    while time.time() < deadline:
        size = log.stat().st_size
        if size == last:
            return
        last = size
        time.sleep(1.5)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cases", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--repeat", type=int, default=1, help="повторов каждого кейса (pass^k)")
    ap.add_argument("--timeout", type=float, default=60.0, help="ожидание хода, секунд")
    ap.add_argument("--pause", type=float, default=6.0, help="пауза между кейсами, секунд")
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
            offset = log.stat().st_size
            print(f"[{rep}/{args.repeat}] {case['id']}: {case['say']}", flush=True)
            speaker.say(str(case["say"]))
            turn = collect(log, offset, args.timeout, str(case['say']))
            turn.update(case_id=str(case["id"]), repeat=rep, say=str(case["say"]))
            turns.append(turn)
            print(
                f"    → tools={turn['tools']} spoken={turn['spoken'][:50]!r} "
                f"skill={turn['skill']} llm×{turn['llm_requests']}",
                flush=True,
            )
            time.sleep(args.pause)
            _wait_idle(log, args.pause)

    payload = {
        "config": {
            "label": args.label,
            "repeat": args.repeat,
            "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
        "turns": turns,
    }
    speaker.close()
    args.out.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"\nсырьё записано: {args.out} ({len(turns)} ходов)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
