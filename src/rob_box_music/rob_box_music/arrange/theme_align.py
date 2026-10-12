"""Годность партитуры к эталону темы: выравнивание нот с пропусками (#3573, ADR-0157 §1.2 п.6).

Прежнее сравнение — точное совпадение интервалов в окне длины эталона без вставок и пропусков: одна лишняя или
потерянная нота (мордент ``A–A–G`` вместо ``A–G–A–G``) сдвигала остаток окна, и вся оставшаяся часть эталона
считалась промахом (7 из 11 интервалов вместо 10 из 11). Здесь — локальное выравнивание (Смит–Уотерман) нот эталона
по нотам голоса:

* совпадение — высота с точностью до октавы (``% 12``) при общей для всего окна транспозиции: перенос мотива
  октавой (каскад ``A5 → A4 → A3``) совпадение не ломает;
* лишняя нота голоса и потерянная нота эталона стоят :data:`GAP` каждая; промах (нота другой высоты на месте
  эталонной) — это пара пропусков, отдельной цены у него нет;
* годность — доля длины эталона, которую выравнивание набрало (``0..1``); дробная, а не ``k/11``.

Дорого искать по всему голосу всеми двенадцатью транспозициями, поэтому окна-кандидаты берутся по голосам
интервалов: каждый совпавший интервал эталона голосует за диагональ ``место в голосе − место в эталоне``, окна
строятся вокруг лучших кластеров диагоналей (допуск :data:`SEED_TOLERANCE` покрывает вставки и пропуски), а
транспозиции — самые частые среди голосов кластера. Выравнивание, найденное только по расстановке нот без общих
интервалов (орнаментика, где от эталона остался скелет), окнами не находится — это осознанная плата за бюджет
времени M7 (ADR-0154), см. #3573.

Чистые функции без состояния.
"""

from __future__ import annotations

import heapq
import itertools
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

#: Цена пропуска ноты (лишней в голосе или потерянной из эталона), в долях совпавшей ноты.
GAP = 0.5
#: Цена промаха: нота другой высоты на месте эталонной (0 — как в прежнем сравнении, где промах только не засчитывался).
MISS = 0.0
#: Окон-кандидатов на голос и эталон: лучшие кластеры диагоналей по голосам интервалов.
SEED_WINDOWS = 3
#: Допуск диагонали: вставка или пропуск нескольких нот сдвигают остаток окна.
SEED_TOLERANCE = 3
#: Транспозиций (по высотному классу) на окно: самые частые среди голосов кластера.
OFFSETS_PER_WINDOW = 2


def pitches(contour: Sequence[int]) -> List[int]:
    """Высоты контура интервалов относительно первой ноты (контур ``n`` интервалов — ``n + 1`` нот)."""
    out = [0]
    for step in contour:
        out.append(out[-1] + step)
    return out


def _align(vpc: Sequence[int], rpc: Sequence[int]) -> Tuple[float, int]:
    """``(счёт, номер первой совпавшей ноты голоса)`` лучшего локального выравнивания эталона ``rpc`` целиком в
    ``vpc`` (оба — высотные классы одной транспозиции): совпадение +1, промах :data:`MISS`, пропуск ноты с любой
    стороны −:data:`GAP`. Промах дёшев, поэтому выравнивание может начаться раньше первого совпадения; начало
    считается по первой совпавшей ноте."""
    n = len(vpc)
    score = [0.0] * (n + 1)
    first = [-1] * (n + 1)
    for r in rpc:
        ns = [score[0] - GAP] * (n + 1)
        nf = [first[0]] * (n + 1)
        for j in range(1, n + 1):
            s, f = score[j] - GAP, first[j]
            g = ns[j - 1] - GAP
            if g > s:
                s, f = g, nf[j - 1]
            hit = vpc[j - 1] == r
            m = score[j - 1] + (1.0 if hit else MISS)
            if m > s:
                s, f = m, (j - 1 if hit and first[j - 1] < 0 else first[j - 1])
            ns[j], nf[j] = s, f
        score, first = ns, nf
    top = max(range(n + 1), key=score.__getitem__)
    return score[top], max(first[top], 0)


def _voice_index(voice: Sequence[int]) -> Dict[int, List[int]]:
    """Места интервалов голоса: интервал → номера нот, с которых он начинается."""
    idx: Dict[int, List[int]] = {}
    for j in range(len(voice) - 1):
        idx.setdefault(voice[j + 1] - voice[j], []).append(j)
    return idx


def _votes(idx: Mapping[int, List[int]], voice: Sequence[int], ref: Sequence[int]) -> Dict[int, List[int]]:
    """Голоса за диагонали ``место в голосе − место в эталоне``: каждый совпавший интервал эталона даёт транспозицию
    ``голос − эталон`` по высотному классу."""
    votes: Dict[int, List[int]] = {}
    for q in range(len(ref) - 1):
        for j in idx.get(ref[q + 1] - ref[q], ()):
            votes.setdefault(j - q, []).append((voice[j] - ref[q]) % 12)
    return votes


def _cluster_weights(votes: Mapping[int, List[int]]) -> Dict[int, int]:
    """Число голосов в кластере (диагональ ± :data:`SEED_TOLERANCE`) для каждой диагонали с голосами — за один проход
    по префиксным суммам."""
    first = min(votes) - SEED_TOLERANCE
    last = max(votes) + SEED_TOLERANCE
    pref = [0]
    pref.extend(itertools.accumulate(len(votes.get(d, ())) for d in range(first, last + 1)))
    return {d: pref[d - first + SEED_TOLERANCE + 1] - pref[d - first - SEED_TOLERANCE] for d in votes}


def _seeds(votes: Mapping[int, List[int]], span: int) -> List[Tuple[int, List[int]]]:
    """Кандидаты-окна ``(диагональ, транспозиции)``: кластеры диагоналей (допуск :data:`SEED_TOLERANCE`) с наибольшим
    числом голосов, ничья — раньше; не ближе ``span`` друг к другу; транспозиции — самые частые среди голосов кластера
    (настоящее совпадение даёт одну и ту же транспозицию много раз, случайное — разные)."""
    if not votes:
        return []
    near = range(-SEED_TOLERANCE, SEED_TOLERANCE + 1)
    weight = _cluster_weights(votes)
    ranked = heapq.nsmallest(SEED_WINDOWS * 8, weight, key=lambda d: (-weight[d], d))
    out: List[Tuple[int, List[int]]] = []
    for d in ranked:
        if all(abs(d - o) > span for o, _ in out):
            count: Dict[int, int] = {}
            for t in near:
                for o in votes.get(d + t, ()):
                    count[o] = count.get(o, 0) + 1
            out.append((d, sorted(count, key=lambda o: (-count[o], o))[:OFFSETS_PER_WINDOW]))
        if len(out) >= SEED_WINDOWS:
            break
    return out


def _share(voice: Sequence[int], idx: Mapping[int, List[int]], ref: Sequence[int]) -> Tuple[float, int]:
    """``(доля эталона в лучшем выравнивании, номер ноты начала)``; ни одного общего интервала — ``(0.0, 0)``."""
    rpc = [y % 12 for y in ref]
    best = (0.0, 0)
    for d, offsets in _seeds(_votes(idx, voice, ref), len(ref) // 2):
        lo, hi = max(0, d - SEED_TOLERANCE), min(len(voice), d + len(ref) + SEED_TOLERANCE + 2)
        for o in offsets:
            score, start = _align([(x - o) % 12 for x in voice[lo:hi]], rpc)
            if score / len(ref) > best[0]:
                best = (score / len(ref), lo + start)
    return best


def best_match(voices: Mapping[str, Sequence[int]], refs: Sequence[Sequence[int]]
               ) -> Optional[Tuple[float, str, int]]:
    """Лучшая годность эталонов ``refs`` (высоты нот, :func:`pitches`) среди голосов ``voices`` (высоты нот):
    ``(годность, голос, номер ноты начала)``. Голос короче эталона не рассматривается; нет ни одной пары — ``None``.
    Ничья — голос раньше в ``voices``, затем раньше в пьесе."""
    best: Optional[Tuple[float, str, int]] = None
    unique = list(dict.fromkeys(tuple(r) for r in refs))
    for name, voice in voices.items():
        idx = _voice_index(voice)
        for ref in unique:
            if len(ref) < 2 or len(voice) < len(ref):
                continue
            share, start = _share(voice, idx, ref)
            if best is None or share > best[0]:
                best = (share, name, start)
    return best


def contour_fit(steps: Sequence[int], ref_steps: Sequence[int]) -> float:
    """Годность контура ``steps`` к эталону ``ref_steps`` (интервалы; транспозиционно-инвариантно): та же мера, что
    у партитуры (:func:`best_match`), для двух готовых контуров."""
    got = best_match({"": pitches(steps)}, [pitches(ref_steps)])
    return got[0] if got else 0.0


__all__ = ["GAP", "OFFSETS_PER_WINDOW", "SEED_TOLERANCE", "SEED_WINDOWS", "best_match", "contour_fit", "pitches"]
