#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""phrase_budget.py -- бюджет реплики человека в окне VAD робота (#4077 П2, ADR-0148: знание в одном модуле).

Зачем. ``audio_node`` отбрасывает фразу длиннее ``speech_max_duration`` (15 с) до STT; харнесс сравнивает длительность
готового eq-файла с бюджетом 12.5 с (15.0 минус накладка VAD 2.5, ``e2e_voice_test.sh`` ``run_step`` п. 1b) и валит шаг
``FAIL scenario_too_long`` -- но уже на стенде, посреди акта (27.10: n302 13.78 с, n702 12.89 с; n607 влез с запасом
0.12 с). Этот модуль решает то же ДО прогона, по двум источникам:

* замер -- словарь ``phrase_eq_measured.json`` (``label -> eq_s`` для данного текста): длительность из лога прогона
  (строка харнесса ``реплика X s`` либо интервал ``PLAY`` -> ``PLAY_DONE``: на n607 12.39 с против 12.379 с в логе);
  замер действует, только пока текст шага тот же (``text_sha1``) и голос тот же;
* оценка -- символы * худший наблюдённый темп голоса (:data:`SEC_PER_CHAR`), для фраз без замера (новых и правленых).

Темп плавает от 0.08 до 0.195 с/символ (паузы, числа, знаки), поэтому оценка нарочно грубая и жёсткая: фраза длиннее
:func:`char_limit` без замера -- красная, а замер, выше порога, -- красный независимо от длины.
"""

from __future__ import annotations

import hashlib
import json
import re
from datetime import datetime
from typing import Dict, Iterable, List, Mapping, NamedTuple, Optional, Tuple

#: Бюджет eq-файла в окне VAD: ``speech_max_duration`` 15.0 с минус накладка 2.5 с (дефолты ``run_step``). Равенство
#: ``audio_node.yaml`` и харнессу держит CHECK 1 ``scripts/testing/test_e2e_scenario_playable.sh`` (``G-Run Tests``):
#: поменяли окно в yaml или накладку в харнессе -- гард краснеет, пока не поправят это число.
VAD_BUDGET_S = 12.5
#: Замер выше -- красный: запас меньше 0.5 с, а повторный синтез на холодном кэше может чуть растянуть фразу.
EQ_ERROR_S = VAD_BUDGET_S - 0.5
#: Замер выше -- предупреждение (запас меньше 1 с, как ``⚠️ запас < 1s`` харнесса); оценка без замера выше -- красная.
EQ_TIGHT_S = VAD_BUDGET_S - 1.0

#: Секунд eq-файла (adelay 1.5 с + речь) на символ: худший темп голоса среди фраз >= 55 символов прогона 27.10
#: (MiniMax, ``tts_provider=minimax``), округлён вверх. anton -- n607 (0.163), ermil -- n706 (0.118), zahar -- n709
#: (0.094), filipp -- n305 (0.126), arina -- n721 (0.127). Аудит 27.10 брал 0.145 -- n607 (0.163) такая оценка не ловит.
SEC_PER_CHAR: Mapping[str, float] = {
    "anton": 0.165,
    "ermil": 0.12,
    "zahar": 0.10,
    "filipp": 0.13,
    "arina": 0.13,
}
#: Голос без замеров: самый медленный из известных.
SEC_PER_CHAR_DEFAULT = max(SEC_PER_CHAR.values())

OK, TIGHT, TOO_LONG = "ok", "tight", "too_long"
MEASURED, ESTIMATE = "measured", "estimate"


def text_key(text: str) -> str:
    """Отпечаток текста реплики: замер действует только для того текста, который мерили."""
    return hashlib.sha1(text.encode("utf-8")).hexdigest()[:12]


def sec_per_char(voice: str) -> float:
    return SEC_PER_CHAR.get(voice, SEC_PER_CHAR_DEFAULT)


def char_limit(voice: str) -> int:
    """Самая длинная реплика (символов) голоса, которую можно пускать без замера."""
    return int(EQ_TIGHT_S / sec_per_char(voice))


class Measure(NamedTuple):
    """Замер одной реплики из лога прогона."""

    label: str
    voice: str
    text: str
    eq_s: float
    how: str  # "harness" -- строка ``реплика X s`` харнесса, "play" -- интервал PLAY -> PLAY_DONE


class Verdict(NamedTuple):
    label: str
    voice: str
    chars: int
    eq_s: float
    source: str  # MEASURED | ESTIMATE
    level: str  # OK | TIGHT | TOO_LONG

    def describe(self) -> str:
        what = "замер" if self.source == MEASURED else "оценка без замера"
        return (
            f"{self.label} [{self.voice}]: {self.chars} симв, {what} {self.eq_s:.2f} с"
        )


def level_of(eq_s: float, measured: bool) -> str:
    """Замер: больше :data:`EQ_ERROR_S` -- красный, больше :data:`EQ_TIGHT_S` -- предупреждение. Оценка: больше
    :data:`EQ_TIGHT_S` -- красная (темп угадан грубо, запас нужен весь)."""
    if measured:
        return TOO_LONG if eq_s > EQ_ERROR_S else TIGHT if eq_s > EQ_TIGHT_S else OK
    return TOO_LONG if eq_s > EQ_TIGHT_S else OK


def verdict(
    label: str, voice: str, text: str, store: Mapping[str, Mapping[str, object]]
) -> Verdict:
    """Вердикт по реплике: замер из ``store`` (если текст и голос те же), иначе оценка по символам."""
    entry = store.get(label)
    if (
        entry
        and entry.get("text_sha1") == text_key(text)
        and entry.get("voice") == voice
    ):
        eq_s = float(entry["eq_s"])  # type: ignore[arg-type]
        return Verdict(label, voice, len(text), eq_s, MEASURED, level_of(eq_s, True))
    eq_s = len(text) * sec_per_char(voice)
    return Verdict(label, voice, len(text), eq_s, ESTIMATE, level_of(eq_s, False))


# -- разбор лога прогона ------------------------------------------------------------------------------------------

_STAMP = re.compile(r"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})(?:\.(\d+))?Z")
_HEADER = re.compile(r'=== STEP (\S+) \(safe=[^)]*\): voice=(\S+) text="(.*)" ===\s*$')
_PLAY = re.compile(r"STEP (\S+): PLAY(?: \w+)? attempt \d+/\d+")
_HARNESS_EQ = re.compile(r"STEP (\S+): (?:\S+ )?реплика ([0-9]+(?:\.[0-9]+)?)s")
#: Кроме трёх повторов лога в артефакте акта: пара короче этого -- копия строк, а не проигрыш.
_MIN_PLAY_S = 0.5


def _stamp(line: str) -> Optional[datetime]:
    found = _STAMP.search(line)
    if not found:
        return None
    base, frac = found.group(1), (found.group(2) or "0")[:6].ljust(6, "0")
    return datetime.fromisoformat(f"{base}.{frac}")


def parse_run_log(log: str) -> List[Measure]:
    """Замеры реплик из лога акта (``actN_<run>.log`` артефакта прогона).

    Текст и голос -- из заголовка ``=== STEP <label> (safe=…): voice=… text="…" ===`` (то, что харнесс реально играл),
    длительность -- строка харнесса ``реплика X s`` (точная, пишется для запаса < 1 с и для отказа) либо самый длинный
    интервал ``PLAY`` -> ``PLAY_DONE`` шага. Лог акта содержит один и тот же вывод трижды -- копии отсекаются
    (:data:`_MIN_PLAY_S`)."""
    header: Dict[str, Tuple[str, str]] = {}
    exact: Dict[str, float] = {}
    played: Dict[str, float] = {}
    open_play: Optional[Tuple[str, datetime]] = None
    for line in log.splitlines():
        head = _HEADER.search(line)
        if head:
            header.setdefault(head.group(1), (head.group(2), head.group(3)))
            continue
        play = _PLAY.search(line)
        if play:
            at = _stamp(line)
            open_play = (play.group(1), at) if at else None
            continue
        if "PLAY_DONE" in line and open_play:
            at = _stamp(line)
            label, since = open_play
            open_play = None
            if at and (at - since).total_seconds() >= _MIN_PLAY_S:
                played[label] = max(
                    played.get(label, 0.0), (at - since).total_seconds()
                )
            continue
        eq = _HARNESS_EQ.search(line)
        if eq:
            exact[eq.group(1)] = max(exact.get(eq.group(1), 0.0), float(eq.group(2)))
    out: List[Measure] = []
    for label, (voice, text) in header.items():
        if label in exact:
            out.append(Measure(label, voice, text, exact[label], "harness"))
        elif label in played:
            out.append(Measure(label, voice, text, round(played[label], 2), "play"))
    return out


# -- словарь замеров ----------------------------------------------------------------------------------------------

STORE_COMMENT = [
    "Замеры длительности реплик (eq-файл, секунды) для scripts/e2e/phrase_budget.py -- гард длины фраз до прогона (#4077 П2).",
    "Замер действует, пока text_sha1 и voice шага те же; правка текста обнуляет его, и шаг идёт по оценке (символы * темп).",
    "Обновление: py -3 scripts/e2e/gen_night_marathon.py --learn <actN_<run>.log ...> (логи акта из артефакта прогона).",
    "how: harness -- строка «реплика X s» харнесса, play -- интервал PLAY -> PLAY_DONE (на n607 12.39 с против 12.379 с).",
]


def merge_measures(
    store: Mapping[str, Mapping[str, object]],
    measures: Iterable[Measure],
    current: Mapping[str, Tuple[str, str]],
    run: str = "",
) -> Tuple[Dict[str, Dict[str, object]], List[str]]:
    """Новый словарь и список пропущенных меток. В словарь идут замеры тех шагов, у которых ТЕКСТ и голос в логе равны
    текущим (``current``: ``label -> (voice, text)``); у правленого шага замер старого текста не нужен, и устаревшая
    запись из ``store`` тоже уходит."""
    merged: Dict[str, Dict[str, object]] = {
        k: dict(v)
        for k, v in store.items()
        if k in current
        and v.get("text_sha1") == text_key(current[k][1])
        and v.get("voice") == current[k][0]
    }
    skipped: List[str] = []
    for m in measures:
        cur = current.get(m.label)
        if cur is None or cur != (m.voice, m.text):
            skipped.append(m.label)
            continue
        merged[m.label] = {
            "voice": m.voice,
            "chars": len(m.text),
            "text_sha1": text_key(m.text),
            "eq_s": round(m.eq_s, 2),
            "how": m.how,
            **({"run": run} if run else {}),
        }
    return merged, skipped


def load_store(path: str) -> Dict[str, Dict[str, object]]:
    """Словарь замеров с диска; файла нет или он битый -- пусто (все фразы пойдут по оценке, громче, а не мягче)."""
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return {}
    steps = data.get("steps") if isinstance(data, dict) else None
    return (
        {k: v for k, v in steps.items() if isinstance(v, dict)}
        if isinstance(steps, dict)
        else {}
    )


def dump_store(store: Mapping[str, Mapping[str, object]]) -> str:
    payload = {
        "_comment": STORE_COMMENT,
        "vad_budget_s": VAD_BUDGET_S,
        "error_s": EQ_ERROR_S,
        "tight_s": EQ_TIGHT_S,
        "steps": {k: store[k] for k in sorted(store)},
    }
    return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
