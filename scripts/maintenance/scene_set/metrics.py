#!/usr/bin/env python3
"""metrics.py — метрики «2.0 лучше 1.1» по размеченной сцене (ADR-0144 §6).

Вход — ``scene.yaml`` (разметка, ``scene_spec.py``) и нормализованный
журнал ``journal.jsonl`` (``extract.py``): по строке на событие

    {"t": 12.4, "channel": "face"|"voice", "kind": "detection"|"encounter"|"speaker",
     "id": "<person_id|speaker_id>", "name": "<уверенное имя или ''>",
     "tentative_name": "", "bbox_cx": 0.41|null}

``t`` — секунды от начала бэга СЦЕНЫ (извлекатель уже вычел смещение
часов прогона, ADR-0144 §5.3). Считалка не знает ни ROS, ни версии
конвейера: 2.0 приносит свой извлекатель, линейка остаётся той же.

Вывод — raw-таблица (ADR-0018) плюс строки ``not covered`` (ADR-0144 §3.2).
Это измерение, а не гейт: код выхода 0, если всё посчиталось; ``--strict``
даёт 1 при ложных именах или двойных приветствиях.

Запуск (где угодно, нужен только PyYAML):
    python3 metrics.py --case ~/scenes/s01/scene.yaml:~/scenes/_replay/R1/s01/journal.jsonl \\
                       --case ... --label v1.1-replay
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import scene_spec as ss  # noqa: E402

#: Допуск присутствия: команда дирижёра опережает движение человека.
DEFAULT_GRACE_S = 1.5
#: Результат голоса приходит после конца фразы (эмбеддинг, очередь).
DEFAULT_VOICE_LAG_S = 8.0

CHANNELS = ("face", "voice")
KINDS = ("detection", "encounter", "speaker")


class JournalError(ValueError):
    """Строка журнала не соответствует формату."""


# ── журнал ───────────────────────────────────────────────────────────────────


def normalize_record(raw: Mapping[str, Any], lineno: int = 0) -> Dict[str, Any]:
    where = f"journal:{lineno}"
    try:
        t = float(raw["t"])
    except (KeyError, TypeError, ValueError):
        raise JournalError(f"{where}: нет числового t")
    channel = raw.get("channel")
    kind = raw.get("kind")
    if channel not in CHANNELS or kind not in KINDS:
        raise JournalError(f"{where}: channel={channel!r} kind={kind!r}")
    cx = raw.get("bbox_cx")
    return {
        "t": t,
        "channel": channel,
        "kind": kind,
        "id": str(raw.get("id") or ""),
        "name": str(raw.get("name") or ""),
        "tentative_name": str(raw.get("tentative_name") or ""),
        "bbox_cx": None if cx is None else float(cx),
    }


def parse_journal(lines: Iterable[str]) -> List[Dict[str, Any]]:
    out = []
    for i, line in enumerate(lines, 1):
        line = line.strip()
        if line:
            out.append(normalize_record(json.loads(line), i))
    return sorted(out, key=lambda r: r["t"])


# ── атрибуция (ADR-0144 §6.1) ────────────────────────────────────────────────


def _in_anchor(p: ss.Participant, cx: Optional[float]) -> bool:
    return p.anchor_cx is not None and cx is not None and p.anchor_cx[0] <= cx <= p.anchor_cx[1]


def attribute_face(
    scene: ss.Scene, visits: Sequence[ss.Visit], t: float, cx: Optional[float], grace: float
) -> Optional[str]:
    """Чьё это лицо: единственный присутствующий, статичный по коридору или единственный живой."""
    present = sorted({v.who for v in visits if v.contains(t, grace)})
    parts = [scene.participants[w] for w in present]
    if len(parts) == 1:
        p = parts[0]
        if p.is_static and p.anchor_cx is not None and not _in_anchor(p, cx):
            return None
        return p.label
    by_anchor = [p.label for p in parts if p.is_static and _in_anchor(p, cx)]
    if by_anchor:
        return by_anchor[0] if len(by_anchor) == 1 else None
    live = [p.label for p in parts if not p.is_static]
    return live[0] if len(live) == 1 else None


def attribute_voice(windows: Sequence[ss.Event], t: float, lag: float) -> Optional[str]:
    """К последнему начавшемуся окну ``say``, в которое попадает ``t`` (с лагом)."""
    hits = [w for w in windows if w.t <= t <= (w.until or w.t) + lag]
    if not hits:
        return None
    return max(hits, key=lambda w: w.t).who


def attribute(
    scene: ss.Scene,
    journal: Sequence[Mapping[str, Any]],
    grace: float = DEFAULT_GRACE_S,
    voice_lag: float = DEFAULT_VOICE_LAG_S,
) -> List[Tuple[Optional[str], Mapping[str, Any]]]:
    vis = ss.visits(scene)
    says = ss.say_windows(scene)
    out = []
    for r in journal:
        if r["channel"] == "face":
            who = attribute_face(scene, vis, r["t"], r["bbox_cx"], grace)
        else:
            who = attribute_voice(says, r["t"], voice_lag)
        out.append((who, r))
    return out


# ── метрики (ADR-0144 §6.2) ──────────────────────────────────────────────────


@dataclass
class SceneMetrics:
    scene: str
    label: str
    false_face: int = 0
    false_voice: int = 0
    false_pairs: Dict[str, int] = field(default_factory=dict)
    spoof_accepted: int = 0
    ids_face: Dict[str, List[str]] = field(default_factory=dict)
    ids_voice: Dict[str, List[str]] = field(default_factory=dict)
    fragmented_face: List[str] = field(default_factory=list)
    fragmented_voice: List[str] = field(default_factory=list)
    nameable_visits: int = 0
    recognized_visits: int = 0
    latencies: List[float] = field(default_factory=list)
    latency_unverified: int = 0
    double_greetings: int = 0
    unattributed: int = 0
    unattributed_named: int = 0
    records: int = 0


def _count_names(m: SceneMetrics, scene: ss.Scene, attributed, windows) -> None:
    for who, r in attributed:
        if who is None:
            m.unattributed += 1
            m.unattributed_named += 1 if r["name"] else 0
            continue
        if not r["name"] or r["name"] in ss.allowed_names(windows[who], r["t"]):
            continue
        if scene.participants[who].kind == "photo":
            m.spoof_accepted += 1
            continue
        if r["channel"] == "face":
            m.false_face += 1
        else:
            m.false_voice += 1
        key = f"{who}<-{r['name']} ({r['channel']})"
        m.false_pairs[key] = m.false_pairs.get(key, 0) + 1


def _count_fragmentation(m: SceneMetrics, scene: ss.Scene, attributed) -> None:
    for channel, ids, frag in (("face", m.ids_face, m.fragmented_face), ("voice", m.ids_voice, m.fragmented_voice)):
        for label in scene.participants:
            seen = sorted({r["id"] for who, r in attributed if who == label and r["channel"] == channel and r["id"]})
            if seen:
                ids[label] = seen
            if len(seen) > ss.expected_int(scene, "max_records", label, 1):
                frag.append(label)


def _visit_face_records(attributed, v: ss.Visit, grace: float):
    return [r for who, r in attributed if who == v.who and r["channel"] == "face" and v.contains(r["t"], grace)]


def _count_visit(m: SceneMetrics, scene: ss.Scene, v: ss.Visit, recs, windows) -> None:
    greetings = sum(1 for r in recs if r["kind"] == "encounter")
    m.double_greetings += max(0, greetings - ss.expected_int(scene, "max_greetings_per_visit", v.who, 1))
    if scene.participants[v.who].kind == "photo":
        return
    allowed_from = ss.first_allowed_at(windows, v.start, v.end)
    if allowed_from is None:
        return
    m.nameable_visits += 1
    hits = [r["t"] for r in recs if r["name"] and r["name"] in ss.allowed_names(windows, r["t"])]
    if not hits:
        return
    m.recognized_visits += 1
    if not v.verified_empty:
        m.latency_unverified += 1
        return
    # Раньше отметки входа (в пределах grace) — «сразу»: человек уже шагнул.
    m.latencies.append(max(0.0, min(hits) - max(v.start, allowed_from)))


def compute(
    scene: ss.Scene,
    journal: Sequence[Mapping[str, Any]],
    label: str = "",
    grace: float = DEFAULT_GRACE_S,
    voice_lag: float = DEFAULT_VOICE_LAG_S,
) -> SceneMetrics:
    m = SceneMetrics(scene=scene.scene, label=label, records=len(journal))
    windows = {p: ss.name_windows(scene, p) for p in scene.participants}
    attributed = attribute(scene, journal, grace, voice_lag)
    _count_names(m, scene, attributed, windows)
    _count_fragmentation(m, scene, attributed)
    for v in ss.visits(scene):
        _count_visit(m, scene, v, _visit_face_records(attributed, v, grace), windows[v.who])
    return m


# ── отчёт ────────────────────────────────────────────────────────────────────

HEADER = (
    "scene", "run", "recs", "false f/v", "spoof", "frag f/v",
    "recognized", "doubles", "lat med/max s", "lat unverif", "unattr (named)",
)


def _fmt_latency(xs: Sequence[float]) -> str:
    if not xs:
        return "-"
    return f"{statistics.median(xs):.1f}/{max(xs):.1f}"


def row(m: SceneMetrics) -> Tuple[str, ...]:
    return (
        m.scene,
        m.label or "-",
        str(m.records),
        f"{m.false_face}/{m.false_voice}",
        str(m.spoof_accepted),
        f"{len(m.fragmented_face)}/{len(m.fragmented_voice)}",
        f"{m.recognized_visits}/{m.nameable_visits}",
        str(m.double_greetings),
        _fmt_latency(m.latencies),
        str(m.latency_unverified),
        f"{m.unattributed} ({m.unattributed_named})",
    )


def total(ms: Sequence[SceneMetrics], label: str = "") -> SceneMetrics:
    t = SceneMetrics(scene="TOTAL", label=label)
    for m in ms:
        t.records += m.records
        t.false_face += m.false_face
        t.false_voice += m.false_voice
        t.spoof_accepted += m.spoof_accepted
        t.fragmented_face += [f"{m.scene}:{x}" for x in m.fragmented_face]
        t.fragmented_voice += [f"{m.scene}:{x}" for x in m.fragmented_voice]
        t.nameable_visits += m.nameable_visits
        t.recognized_visits += m.recognized_visits
        t.latencies += m.latencies
        t.latency_unverified += m.latency_unverified
        t.double_greetings += m.double_greetings
        t.unattributed += m.unattributed
        t.unattributed_named += m.unattributed_named
    return t


def render_table(rows: Sequence[Sequence[str]]) -> str:
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    lines = ["  ".join(c.ljust(w) for c, w in zip(r, widths)).rstrip() for r in rows]
    lines.insert(1, "  ".join("-" * w for w in widths))
    return "\n".join(lines)


def render_details(m: SceneMetrics) -> List[str]:
    out = []
    for pair, n in sorted(m.false_pairs.items()):
        out.append(f"  {m.scene}: FALSE NAME {pair} x{n}")
    for channel, ids in (("face", m.ids_face), ("voice", m.ids_voice)):
        for who, lst in sorted(ids.items()):
            out.append(f"  {m.scene}: {channel} ids {who}: {', '.join(x[:8] for x in lst)}")
    return out


def render_report(ms: Sequence[SceneMetrics], label: str = "") -> str:
    rows = [HEADER] + [row(m) for m in ms] + [row(total(ms, label))]
    parts = [render_table(rows), "", "details:"]
    for m in ms:
        parts += render_details(m)
    parts += ["", "not covered (ADR-0144 §3.2 — нужен живой второй человек):"]
    parts += [f"  - {x}" for x in ss.NOT_COVERED]
    return "\n".join(parts)


# ── CLI ──────────────────────────────────────────────────────────────────────


def _split_case(case: str) -> Tuple[str, str]:
    # «:» есть и в Windows-путях — режем по ПОСЛЕДНЕМУ «.yaml:».
    idx = case.rfind(".yaml:")
    if idx < 0:
        raise SystemExit(f"--case {case!r}: ожидается <scene.yaml>:<journal.jsonl>")
    return os.path.expanduser(case[: idx + 5]), os.path.expanduser(case[idx + 6:])


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--case", action="append", required=True, help="<scene.yaml>:<journal.jsonl>, повторяется")
    ap.add_argument("--label", default="", help="метка прогона (v1.1-replay / v2.0-replay / live-control)")
    ap.add_argument("--grace-s", type=float, default=DEFAULT_GRACE_S)
    ap.add_argument("--voice-lag-s", type=float, default=DEFAULT_VOICE_LAG_S)
    ap.add_argument("--strict", action="store_true", help="exit 1 при ложных именах или двойных приветствиях")
    args = ap.parse_args(argv)

    ms = []
    for case in args.case:
        scene_path, journal_path = _split_case(case)
        scene = ss.load_scene(scene_path)
        with open(journal_path, encoding="utf-8") as f:
            journal = parse_journal(f)
        ms.append(compute(scene, journal, args.label, args.grace_s, args.voice_lag_s))
    print(render_report(ms, args.label))
    tot = total(ms)
    bad = tot.false_face + tot.false_voice + tot.double_greetings
    return 1 if (args.strict and bad) else 0


if __name__ == "__main__":
    sys.exit(main())
