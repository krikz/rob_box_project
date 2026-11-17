#!/usr/bin/env python3
"""extract.py — выходы конвейера 1.1 из бэга → нормализованный журнал (ADR-0144 §6.1).

Извлекатель ВЕРСИИ 1.1: читает ``/vision/hailo/events`` (``VisionEvent``,
``event_type="face"``, маркер Встречи в ``attributes_json``) и
``/voice/speaker/result`` (JSON ``SpeakerMatch``) и пишет ``journal.jsonl``
для ``metrics.py``. Для 2.0 будет свой извлекатель (треки,
``Acquaintance.id``) с тем же форматом строк.

Два режима времени:
  * ``--mode control`` — бэг самой сцены, выходы живой 1.1: время приёма
    уже по часам сцены, смещение 0.
  * ``--mode replay`` — бэг выходов офлайн-прогона: всё в нём — по часам
    прогона (``VisionEvent.stamp`` = ``now()``, ``vision_hailo_node.py:523``).
    Смещение часов считается по ``camera_info``, который есть в ОБОИХ бэгах
    с одним и тем же ``header.stamp`` кадра: медиана (приём − stamp) в
    прогоне минус та же медиана в сцене (ADR-0144 §5.3). Задержка самого
    конвейера в смещение не попадает — иначе задержка «вошёл → confirmed»
    у прогона была бы короче, чем у живой 1.1, на время обработки.

``t`` в журнале — секунды от начала бэга сцены (``starting_time`` из его
``metadata.yaml``).

Запуск (нужны rosbag2_py и rob_box_perception_msgs — образ ``vision-hailo``):
    python3 extract.py --bag /replay/out_bag --scene-bag /scene/bag --mode replay \\
                       --out /replay/journal.jsonl
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple

VISION_TOPIC = "/vision/hailo/events"
SPEAKER_TOPIC = "/voice/speaker/result"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"


# ── чистые функции ───────────────────────────────────────────────────────────


def bag_start_s(metadata: Mapping[str, Any]) -> float:
    """``starting_time`` бэга (секунды эпохи) из разобранного ``metadata.yaml``."""
    info = metadata.get("rosbag2_bagfile_information") or {}
    ns = (info.get("starting_time") or {}).get("nanoseconds_since_epoch")
    if ns is None:
        raise ValueError("metadata.yaml: нет starting_time.nanoseconds_since_epoch")
    return int(ns) / 1e9


def estimate_offset(pairs: Sequence[Tuple[float, float]]) -> Optional[float]:
    """Медиана (время приёма − stamp кадра); ``None``, если пар нет."""
    if not pairs:
        return None
    return statistics.median(recv - stamp for recv, stamp in pairs)


def replay_clock_offset(
    replay_pairs: Sequence[Tuple[float, float]], scene_pairs: Sequence[Tuple[float, float]]
) -> Optional[float]:
    """Сдвиг «часы прогона − часы сцены» по одному и тому же входному топику.

    Обе медианы включают только доставку сообщения, не обработку конвейером.
    """
    replay, scene = estimate_offset(replay_pairs), estimate_offset(scene_pairs)
    if replay is None or scene is None:
        return None
    return replay - scene


def _encounter_marker(attributes_json: str) -> Dict[str, Any]:
    if not attributes_json:
        return {}
    try:
        data = json.loads(attributes_json)
    except ValueError:
        return {}
    return data if isinstance(data, dict) and data.get("encounter") == "start" else {}


def vision_event_record(ev: Mapping[str, Any], t: float) -> Optional[Dict[str, Any]]:
    """``VisionEvent`` (как dict) → строка журнала; не-лицо → ``None``.

    Одно событие → ровно одна строка: с маркером Встречи — ``encounter``,
    без — ``detection``. Так ложное имя на кадре Встречи не считается дважды.
    """
    if ev.get("event_type") != "face":
        return None
    marker = _encounter_marker(str(ev.get("attributes_json") or ""))
    return {
        "t": round(t, 3),
        "channel": "face",
        "kind": "encounter" if marker else "detection",
        "id": str(ev.get("embedding_id") or marker.get("person_id") or ""),
        "name": str(ev.get("display_name") or marker.get("name") or ""),
        "tentative_name": "",
        "bbox_cx": float(ev.get("bbox_cx", 0.0)),
    }


def speaker_result_record(data: str, t: float) -> Optional[Dict[str, Any]]:
    """JSON ``/voice/speaker/result`` → строка журнала.

    Подтверждения регистрации (``"event": "registered"`` и пр.) — не
    опознание говорящего, пропускаются. Незнакомец (``is_known=false``)
    пишется без имени — он нужен для атрибуции и дробления.
    """
    try:
        payload = json.loads(data)
    except ValueError:
        return None
    if not isinstance(payload, dict) or "event" in payload:
        return None
    return {
        "t": round(t, 3),
        "channel": "voice",
        "kind": "speaker",
        "id": str(payload.get("speaker_id") or ""),
        "name": str(payload.get("name") or ""),
        "tentative_name": str(payload.get("tentative_name") or ""),
        "bbox_cx": None,
    }


def build_journal(
    messages: Iterable[Tuple[str, float, Any]],
    scene_start: float,
    offset: float,
) -> List[Dict[str, Any]]:
    """(топик, время приёма, сообщение-как-dict|str) → журнал по часам сцены."""
    out = []
    for topic, recv, msg in messages:
        t = recv - offset - scene_start
        if topic == VISION_TOPIC:
            rec = vision_event_record(msg, t)
        elif topic == SPEAKER_TOPIC:
            rec = speaker_result_record(msg, t)
        else:
            rec = None
        if rec is not None:
            out.append(rec)
    return sorted(out, key=lambda r: r["t"])


# ── чтение бэга (rosbag2_py — только в контейнере) ───────────────────────────


def _read_metadata(bag_dir: str) -> Dict[str, Any]:
    import yaml

    with open(os.path.join(bag_dir, "metadata.yaml"), encoding="utf-8") as f:
        return yaml.safe_load(f)


def _iter_bag(bag_dir: str, topics: Sequence[str]) -> Iterator[Tuple[str, float, Any]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in topics if t in types]
    if not wanted:
        return  # пустой StorageFilter = «все топики», а не «ни одного»
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        yield topic, ts / 1e9, deserialize_message(data, classes[topic])


def _as_plain(topic: str, msg: Any) -> Any:
    if topic == SPEAKER_TOPIC:
        return msg.data
    if topic == VISION_TOPIC:
        return {k: getattr(msg, k) for k in (
            "event_type", "embedding_id", "display_name", "attributes_json", "bbox_cx")}
    return msg


def _stamp_pairs(bag_dir: str, topic: str) -> List[Tuple[float, float]]:
    pairs = []
    for _topic, recv, msg in _iter_bag(bag_dir, [topic]):
        st = msg.header.stamp
        pairs.append((recv, st.sec + st.nanosec / 1e9))
    return pairs


def _replay_offset(bag_dir: str, scene_bag: str) -> Tuple[float, int]:
    replay_pairs = _stamp_pairs(bag_dir, CAMERA_INFO_TOPIC)
    off = replay_clock_offset(replay_pairs, _stamp_pairs(scene_bag, CAMERA_INFO_TOPIC))
    if off is None:
        raise SystemExit(
            f"replay: нет {CAMERA_INFO_TOPIC} в бэге прогона или сцены — смещение часов "
            "не оценить (ADR-0144 §5.3); задай --offset-s явно"
        )
    return off, len(replay_pairs)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="бэг с выходами (сцены — для control, прогона — для replay)")
    ap.add_argument("--scene-bag", help="бэг сцены (для starting_time); по умолчанию = --bag")
    ap.add_argument("--mode", choices=("control", "replay"), required=True)
    ap.add_argument("--offset-s", type=float, help="смещение часов вручную (replay без наблюдений)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    scene_start = bag_start_s(_read_metadata(args.scene_bag or args.bag))
    if args.mode == "control":
        offset, basis = 0.0, "control: 0"
    elif args.offset_s is not None:
        offset, basis = args.offset_s, "manual"
    else:
        if not args.scene_bag:
            raise SystemExit("replay: нужен --scene-bag (camera_info сцены для смещения часов)")
        offset, n = _replay_offset(args.bag, args.scene_bag)
        basis = f"camera_info, {n} msgs"
    msgs = ((tp, recv, _as_plain(tp, m)) for tp, recv, m in _iter_bag(args.bag, [VISION_TOPIC, SPEAKER_TOPIC]))
    journal = build_journal(msgs, scene_start, offset)
    with open(args.out, "w", encoding="utf-8") as f:
        for rec in journal:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    faces = sum(1 for r in journal if r["channel"] == "face")
    print(f"extract: {len(journal)} records (face={faces}, voice={len(journal) - faces}) "
          f"offset={offset:.3f}s ({basis}) -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
