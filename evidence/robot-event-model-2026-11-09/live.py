"""Живые ходы из Loki (7 дней): то же разбиение, что turns.py; ходы марафонов отделяются по времени (#4512)."""
import collections
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import turns as T  # noqa: E402

HERE = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()  # каталог с выгрузкой loki_pull.py
rows = []
for name in ("loki_live.tsv", "loki_live2.tsv"):
    for line in open(os.path.join(HERE, name), encoding="utf-8"):
        ts, _, rest = line.partition("\t")
        rows.append((int(ts), rest))
rows.sort()
merged = os.path.join(HERE, "loki_live.log")
with open(merged, "w", encoding="utf-8") as fo:
    for _, rest in rows:
        fo.write(rest)

MARATHON_END = 1793927900  # 2026-11-06 01:18 UTC, последний прогон L: E2E Voice Test
tr = [t for t in T.parse_log(merged) if t["final"]]
live = [t for t in tr if (t.get("final_ts") or 0) > MARATHON_END]
mar = [t for t in tr if (t.get("final_ts") or 0) <= MARATHON_END]
for name, ts in (("все 7 дней", tr), ("до конца марафонов", mar), ("после 06.11 01:18 (живые)", live)):
    llm = [t for t in ts if int(t["final"]["llm_calls"]) > 0]
    print(name, "| ходов", len(ts), "tier1", sum(1 for t in ts if t["tier1"]), "LLM", len(llm),
          "| llm_calls", sorted(collections.Counter(int(t["final"]["llm_calls"]) for t in ts).items()),
          "| status", dict(collections.Counter((t["final"]["stage"], t["final"]["status"]) for t in ts)),
          "| provider", dict(collections.Counter(t["final"]["llm_provider"] for t in ts)),
          "| tokens med", statistics.median([int(t["final"]["tokens"]) for t in llm]) if llm else None,
          "lat med", statistics.median([int(t["final"]["latency_ms"]) for t in llm]) if llm else None,
          "| signals", dict(collections.Counter(s for t in ts for s in t["signals"])))
T1 = {"stop_music", "set_volume", "set_voice", "set_speed", "set_pitch"}
n = collections.Counter()
for t in live:
    ex = {x for x, _ in t["executed"]}
    if int(t["final"]["llm_calls"]) and ex and ex <= T1 and all(c["verdict"] == "command" for c in t["cmds"]):
        n[tuple(sorted(ex))] += 1
print("живые LLM-ходы только с Tier-1-подобным тулом:", dict(n))
print("живые: размер среза", sorted(collections.Counter(len(t["slices"][0]) for t in live if t["slices"]).items()))
print("живые: исполненные тулы", collections.Counter(x for t in live for x, _ in t["executed"]).most_common(20))
print("живые: дни", sorted(collections.Counter(int((t["final_ts"] - 1793836800) // 86400) for t in live).items()))
