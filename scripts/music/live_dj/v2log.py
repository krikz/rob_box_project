"""Разбор строк лога движка v2 (ADR-0149): ROS-штамп, started трека, план reasoner.

Лог пишет ``[node] [INFO] [<epoch>.<ns>] [logger]: <текст>``; JSON-событий в нём нет.
"""
import json
import re

TS = re.compile(r"\[(\d{10})\.(\d+)\]")
KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")


def ros_ts(line):
    """Epoch-секунды из ROS-штампа строки или ``None``."""
    m = TS.search(line)
    return float(m.group(1) + "." + m.group(2)[:9]) if m else None


def _kv(text):
    return {k: v for k, v in KV.findall(text)}


def parse_started(line):
    """Трек v2 из ``[music v2] started …`` или ``None``."""
    if "[music v2] started" not in line:
        return None
    t = ros_ts(line)
    kv = _kv(line.split("started", 1)[1])
    if t is None or "track_id" not in kv:
        return None
    return {"t_abs": t, "track_id": kv["track_id"], "bpm": round(float(kv.get("bpm", 0))) or None,
            "deck": kv.get("deck", ""), "source": "—"}


def parse_set_started(line):
    """``(track_id, source)`` из ``[set v2] … started`` или ``None``."""
    if "[set v2]" not in line or " started " not in line:
        return None
    kv = _kv(line.split(" started ", 1)[1])
    if "track_id" not in kv:
        return None
    return kv["track_id"], kv.get("source", "—")


def parse_composition(line):
    """Состав трека из ``[set v2] … started … composition={…}`` (ADR-0152 §2.3) или ``None``.

    JSON разбирается ``raw_decode`` от ``composition=``, а не ``KV``: значения (id хука) могут содержать пробелы."""
    if "[set v2]" not in line or " started " not in line or "composition=" not in line:
        return None
    try:
        value, _end = json.JSONDecoder().raw_decode(line.split("composition=", 1)[1])
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


def parse_plan(line):
    """План reasoner: ``{row, mode, hooks}`` или ``None``."""
    if "[reasoner]" not in line or "план применён" not in line:
        return None
    kv = _kv(line.split("план применён", 1)[1])
    return {"row": kv.get("row", ""), "mode": kv.get("mode", ""), "hooks": kv.get("hooks", "")}
