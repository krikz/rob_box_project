#!/usr/bin/env python3
"""Достаёт /evaluation из rosbag2 sqlite и печатает тики для eval_stall_verdict.

Запускать внутри образа с ROS (rob_box_sim:hil): на хосте нет rclpy.
  docker run --rm --network none -v $BAG:/bag:ro -v $OUT:/out \
    rob_box_sim:hil python3 decode_eval_bag.py /bag/bag_0.db3 /out/ticks.json

Топик по умолчанию /FollowPath/evaluation (publisher.cpp:104, имя плагина
FollowPath). Если в bag другое имя — --topic.

Окно стояния задаётся снаружи: --t-lo/--t-hi в наносекундах bag, либо файл
с уже известными метками. Скрипт не решает, какой тик стояний: это делает
разбор GT, здесь только декодирование.
"""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys


def ticks_of(connection, topic_id: int, message_cls, t_lo: int, t_hi: int) -> list:
    from rclpy.serialization import deserialize_message

    ticks = []
    query = (
        "SELECT timestamp, data FROM messages WHERE topic_id=? "
        "AND timestamp BETWEEN ? AND ? ORDER BY timestamp"
    )
    for timestamp, blob in connection.execute(query, (topic_id, t_lo, t_hi)):
        msg = deserialize_message(blob, message_cls)
        samples = []
        for index, twist in enumerate(msg.twists):
            samples.append(
                {
                    "ts": timestamp,
                    "wz": twist.traj.velocity.theta,
                    "vx": twist.traj.velocity.x,
                    "chosen": index == msg.best_index,
                    "total": twist.total,
                    "scores": [
                        {"name": score.name, "raw_score": score.raw_score, "scale": score.scale}
                        for score in twist.scores
                    ],
                }
            )
        ticks.append({"ts": timestamp, "samples": samples})
    return ticks


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("bag")
    parser.add_argument("out")
    parser.add_argument("--topic", default="/FollowPath/evaluation")
    parser.add_argument("--t-lo", type=int, default=0)
    parser.add_argument("--t-hi", type=int, default=2**63 - 1)
    args = parser.parse_args(argv)

    import rclpy
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    connection = sqlite3.connect(args.bag)
    row = connection.execute("SELECT id, type FROM topics WHERE name=?", (args.topic,)).fetchone()
    if row is None:
        names = [item[0] for item in connection.execute("SELECT name FROM topics")]
        sys.stderr.write(f"топика {args.topic} нет. В bag: {names}\n")
        return 1
    topic_id, type_name = row
    ticks = ticks_of(connection, topic_id, get_message(type_name), args.t_lo, args.t_hi)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump({"topic": args.topic, "ticks": ticks}, handle)
    sys.stderr.write(f"тиков {len(ticks)} -> {args.out}\n")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
