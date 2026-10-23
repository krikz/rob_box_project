"""Четыре подхода к попаданию речи в такт (ADR-0150 §5.6.3 О11) — офлайн, на синтезе MiniMax и бите движка.

Общая раскладка выступления (как ``plan.sync.layout``): такт 0 — вступление бита, куплет 1 — с такта 1, каждый
следующий — на первом такте после конца предыдущего. Сетка: доля 0 звучит в t=0.

1. ``quantize`` — **как сейчас в коде (PR-7c):** начало файла реплики-куплета на такте; внутри куплета речь идёт
   своим темпом. ``quantize_lines`` (1′) — дешёвое продолжение той же идеи: реплика = строка, на такт ставится
   первый онсет строки, а не начало файла (тишина в начале синтеза ~0.1–0.2 с).
2. ``beat_to_line`` — идея Шифу: синтез → онсеты → подбор темпа бита (``BPM_RANGE``) по захвату онсетов сеткой 16-х
   (R16) и сдвиг каждого куплета на долю 16-й по средней фазе. Бит движка рисуется на выбранном темпе (ноты, а не
   растяжение аудио).
3. ``stretch`` — time-stretch речи под сетку: онсеты → ближайшие 16-е (интервал квантуется целым числом 16-х в пределах
   ``RATIO``), кусочная карта времени; WSOLA (точная карта) или rubberband (ffmpeg ``asendcmd`` меняет ``tempo``).
4. ``markup`` — «ритмическая подача» средствами провайдера: у MiniMax/Yandex/Silero нет режима рэпа или пения
   (первоисточники — отчёт), есть только паузы (``<#x#>`` / ``sil<[ms]>`` / ``<break>``) и скорость. Паузы
   между строками считаются по меткам времени первого синтеза так, чтобы каждая строка начиналась на такте.
   Выбран Шифу (О11, 23.10): формула и лимиты переехали в продукт — ``rob_box_dialog.plan.rap_pauses``; здесь
   только длины строк по меткам букв (у робота меток нет — там оценка по слогам).
"""

from __future__ import annotations

import math
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import soundfile as sf

from common import SR, VOWELS, lines
from metric import resultant
from rob_box_dialog.plan import rap_pauses

BPM_RANGE = (80.0, 140.0)
RATIO = (0.6, 1.7)
LINE_LEAD_S = 0.03
#: P-центр слога (воспринимаемый момент) позже акустического онсета (Morton, Marcus & Frankish 1976: 20–60 мс);
#: на долю ставится ``онсет + P_CENTER_S``. Константа из литературы, на слух Шифу не подобрана.
P_CENTER_S = 0.03


def bar_s(bpm: float) -> float:
    return 240.0 / bpm


def next_bar(t: float, bpm: float) -> float:
    b = bar_s(bpm)
    return math.ceil(t / b - 1e-9) * b


# --- 1 / 1′ -------------------------------------------------------------------------------------------------------
def quantize(verses: Sequence[np.ndarray], bpm: float) -> List[Tuple[float, np.ndarray]]:
    """Начало файла каждого куплета — на такте (текущий код)."""
    out, t = [], bar_s(bpm)
    for x in verses:
        out.append((t, x))
        t = next_bar(t + len(x) / SR, bpm)
    return out


def line_cuts(words: Sequence[dict], verse: str) -> List[float]:
    """Начала строк куплета (с) по меткам букв MiniMax: первая буква каждой строки."""
    starts, i, letters = [], 0, [w for w in words if w["word"].strip() and w["word"].isalpha()]
    for ln in lines(verse):
        starts.append(letters[i]["t0"] if i < len(letters) else letters[-1]["t1"])
        i += sum(ch.isalpha() for ch in ln)
    return starts


def quantize_lines(verses: Sequence[np.ndarray], cuts: Sequence[Sequence[float]], onsets: Sequence[np.ndarray],
                   bpm: float) -> List[Tuple[float, np.ndarray]]:
    """1′: реплика = строка; первый онсет строки — на такте."""
    out, t = [], bar_s(bpm)
    for x, cs, on in zip(verses, cuts, onsets):
        edges = [max(0.0, c - LINE_LEAD_S) for c in cs] + [len(x) / SR]
        for a, b in zip(edges[:-1], edges[1:]):
            piece = x[int(a * SR):int(b * SR)]
            first = next((o - a for o in on if a <= o < b), 0.0) + P_CENTER_S
            start = next_bar(t + first, bpm) - first
            out.append((start, piece))
            t = start + len(piece) / SR
        t = next_bar(t, bpm)
    return out


# --- 2 ------------------------------------------------------------------------------------------------------------
def fit_bpm(onsets: Sequence[np.ndarray], lo: float = BPM_RANGE[0], hi: float = BPM_RANGE[1]) -> Tuple[float, float]:
    """Темп, при котором сетка 16-х лучше всего захватывает онсеты всех куплетов (взвешенный R16)."""
    best = (lo, -1.0)
    for bpm in np.arange(lo, hi + 1e-9, 0.25):
        s16 = 15.0 / bpm
        r = sum(len(o) * resultant(o, 0.0, s16) for o in onsets) / max(1, sum(len(o) for o in onsets))
        if r > best[1]:
            best = (float(bpm), float(r))
    return best


def beat_to_line(verses: Sequence[np.ndarray], onsets: Sequence[np.ndarray],
                 bpm: float) -> List[Tuple[float, np.ndarray]]:
    """Куплет на такт так, чтобы средняя фаза его онсетов легла на 16-ю (сдвиг ≤ половины 16-й)."""
    s16 = 15.0 / bpm
    out, t = [], bar_s(bpm)
    for x, on in zip(verses, onsets):
        on = np.asarray(on) + P_CENTER_S
        phase = np.angle(np.mean(np.exp(2j * np.pi * np.asarray(on) / s16))) / (2 * np.pi) * s16
        first = float(on[0]) if len(on) else 0.0
        start = next_bar(t + first, bpm) - first
        d = (first - phase) / s16  # start + phase ≡ 0 (mod 16-я): средняя фаза онсетов → узел сетки
        start += (d - round(d)) * s16
        out.append((start, x))
        t = start + len(x) / SR
        t = next_bar(t, bpm)
    return out


# --- 3 ------------------------------------------------------------------------------------------------------------
def stretch_map(on: np.ndarray, length_s: float, bpm: float) -> List[Tuple[float, float]]:
    """Карта времени (вход, выход): онсеты → 16-е, паузы между строками → доля; первый онсет остаётся на месте."""
    s16, beat = 15.0 / bpm, 60.0 / bpm
    anchors = [(0.0, 0.0), (float(on[0]), float(on[0]))]
    g = float(on[0])
    for prev, cur in zip(on[:-1], on[1:]):
        ioi = float(cur - prev)
        if ioi >= 0.4:  # пауза между строками: следующая строка — на доле
            target = math.ceil((g + ioi * 0.9 - on[0]) / beat) * beat + on[0]
        else:
            n = min(range(1, 9), key=lambda k: abs(math.log(k * s16 / ioi)))  # ближайшее в логарифме
            while n * s16 / ioi > RATIO[1] and n > 1:
                n -= 1
            while n * s16 / ioi < RATIO[0]:
                n += 1
            target = g + n * s16
        anchors.append((float(cur), target))
        g = target
    anchors.append((length_s, g + (length_s - float(on[-1]))))
    return anchors


def _interp_out_to_in(anchors: Sequence[Tuple[float, float]], t_out: float) -> float:
    a_in, a_out = zip(*anchors)
    return float(np.interp(t_out, a_out, a_in))


def wsola(x: np.ndarray, anchors: Sequence[Tuple[float, float]], win: int = 480, tol: int = 80) -> np.ndarray:
    """WSOLA по кусочной карте времени (окно 30 мс, шаг 15 мс, поиск ±5 мс)."""
    hop = win // 2
    w = np.hanning(win + 1)[:win]
    n_out = int(anchors[-1][1] * SR)
    y = np.zeros(n_out + win)
    xp = np.concatenate([np.zeros(tol + win), x, np.zeros(tol + 2 * win)])
    prev = None
    for m in range(0, n_out, hop):
        p = int(_interp_out_to_in(anchors, m / SR) * SR) + tol + win
        if prev is None:
            best = p
        else:
            nat = xp[prev + hop:prev + hop + win]
            cand = np.array([np.dot(nat, xp[q:q + win]) for q in range(p - tol, p + tol + 1, 4)])
            best = p - tol + 4 * int(np.argmax(cand))
        y[m:m + win] += xp[best:best + win] * w
        prev = best
    return y[:n_out].astype(np.float32)


def rubberband(x: np.ndarray, anchors: Sequence[Tuple[float, float]]) -> np.ndarray:
    """ffmpeg ``rubberband`` с переключением ``tempo`` по входному времени (``asendcmd``) — карта приблизительная."""
    cmds = []
    for (i0, o0), (i1, o1) in zip(anchors[:-1], anchors[1:]):
        if i1 - i0 > 1e-4 and o1 - o0 > 1e-4:
            cmds.append(f"{i0:.4f} rubberband tempo {(i1 - i0) / (o1 - o0):.4f}")
    with tempfile.TemporaryDirectory() as d:
        src, dst, cmd = Path(d) / "in.wav", Path(d) / "out.wav", Path(d) / "cmd.txt"
        sf.write(str(src), x, SR, subtype="PCM_16")
        cmd.write_text(";\n".join(cmds) + ";\n", encoding="utf-8")
        filt = ("asetnsamples=n=64,asendcmd=f=cmd.txt,"
                "rubberband=tempo=1:transients=crisp:detector=compound:window=short:pitchq=quality")
        subprocess.run(["ffmpeg", "-y", "-v", "error", "-i", src.name, "-af", filt, dst.name], check=True, cwd=d)
        y, _ = sf.read(str(dst), dtype="float32")
    return y


def stretch(verses: Sequence[np.ndarray], onsets: Sequence[np.ndarray], bpm: float,
            engine: str = "wsola") -> Tuple[List[Tuple[float, np.ndarray]], Dict[str, float]]:
    out, t, ratios, maps = [], bar_s(bpm), [], []
    for x, on in zip(verses, onsets):
        anchors = stretch_map(on, len(x) / SR, bpm)
        maps.append(anchors)
        for (i0, o0), (i1, o1) in zip(anchors[1:-2], anchors[2:-1]):
            ratios.append((o1 - o0) / (i1 - i0))
        y = wsola(x, anchors) if engine == "wsola" else rubberband(x, anchors)
        first = float(on[0]) + P_CENTER_S
        start = next_bar(t + first, bpm) - first
        out.append((start, y))
        t = next_bar(start + len(y) / SR, bpm)
    r = np.asarray(ratios)
    return out, {"ratio_p05": round(float(np.percentile(r, 5)), 2), "ratio_p50": round(float(np.median(r)), 2),
                 "ratio_p95": round(float(np.percentile(r, 95)), 2),
                 "outside_0.8_1.25": round(float(np.mean((r < 0.8) | (r > 1.25))), 3), "maps": maps}


def expected_lines(kind: str, placed: Sequence[Tuple[float, np.ndarray]], cuts: Sequence[Sequence[float]],
                   maps: Sequence = ()) -> List[float]:
    """Где по раскладке подхода звучит начало каждой строки (метки первых букв ``cuts`` по куплетам)."""
    if kind == "q_lines":  # кусок строки начинается за LINE_LEAD_S до её первой буквы
        return [s + LINE_LEAD_S for s, _ in placed]
    out = []
    for k, ((s, _), cs) in enumerate(zip(placed, cuts)):
        for c in cs:
            if maps:
                a_in, a_out = zip(*maps[k])
                c = float(np.interp(c, a_in, a_out))
            out.append(s + c)
    return out


# --- 4 ------------------------------------------------------------------------------------------------------------
def markup_text(verse: str, words: Sequence[dict], bpm: float, max_speed: float = 1.35,
                fix: Sequence[float] = ()) -> Tuple[str, float, int]:
    """Текст с паузами MiniMax ``<#x#>`` между строками и скорость: каждая строка — с такта.

    Длины строк — по меткам букв первого синтеза; строка влезает в ``n`` тактов (1, если хватает скорости ≤
    ``max_speed``, иначе 2), пауза = ``n·такт − длина/скорость``; ``fix`` — поправка пауз после замера прошлого
    синтеза (с, на сколько шаг строки вышел длиннее ``n`` тактов) — второй проход калибровки."""
    letters = [w for w in words if w["word"].strip() and w["word"].isalpha()]
    spans, i = [], 0
    for ln in lines(verse):
        k = sum(ch.isalpha() for ch in ln)
        spans.append(letters[i + k - 1]["t1"] - letters[i]["t0"])
        i += k
    # спайк мерил паузы как есть: поправка на «длиннее заказанной» (PAUSE_OVERHEAD_S) выведена из его же замера
    pauses, speed, n = rap_pauses.layout(spans, bar_s(bpm), max_speed, fix, overhead_s=0.0)
    return rap_pauses.mark(lines(verse), pauses), speed, n


def vowel_times(words: Sequence[dict]) -> List[float]:
    return [w["t0"] for w in words if w["word"] in VOWELS]


def mix(placed: Sequence[Tuple[float, np.ndarray]], beat: np.ndarray, vocal_db: float = -18.0,
        beat_db: float = -25.0) -> Tuple[np.ndarray, np.ndarray]:
    """Сведение: (микс, вокальная дорожка в той же шкале времени)."""
    n = max(len(beat), max(int(s * SR) + len(x) for s, x in placed))
    voc = np.zeros(n, dtype=np.float32)
    for s, x in placed:
        i = int(round(s * SR))
        voc[i:i + len(x)] += x
    from common import gain_to

    voc = gain_to(voc, vocal_db)
    b = np.zeros(n, dtype=np.float32)
    b[:len(beat)] = gain_to(beat, beat_db)
    return voc + b, voc
