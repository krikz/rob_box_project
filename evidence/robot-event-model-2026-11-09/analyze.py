"""База правил по трём задачам на ходах архива марафонов (#4512). Читает turns.jsonl из turns.py."""
import collections
import json
import statistics
import sys

rows = [json.loads(x) for x in open(sys.argv[1], encoding="utf-8")]
by = {}
for r in rows:  # логи соседних актов перекрываются: один ход — одна запись, с шагом важнее
    if r["turn_id"] not in by or r.get("steps"):
        by[r["turn_id"]] = r
turns = list(by.values())
fin = [t for t in turns if t["final"]]
print("== общее")
print("ходов уникальных", len(turns), "с итогом 📊", len(fin))
print("stage/status", collections.Counter((t["final"]["stage"], t["final"]["status"]) for t in fin))
calls = collections.Counter(int(t["final"]["llm_calls"]) for t in fin)
print("llm_calls", sorted(calls.items()))
print("tier1", sum(1 for t in turns if t["tier1"]), collections.Counter(t["tier1"] for t in turns if t["tier1"]))
print("provider", collections.Counter(t["final"]["llm_provider"] for t in fin))
llm = [t for t in fin if int(t["final"]["llm_calls"]) > 0]
tok = [int(t["final"]["tokens"]) for t in llm]
lat = [int(t["final"]["latency_ms"]) for t in llm]
lat0 = [int(t["final"]["latency_ms"]) for t in fin if int(t["final"]["llm_calls"]) == 0]
print("LLM-ходов", len(llm), "tokens median", statistics.median(tok), "sum", sum(tok),
      "latency median", statistics.median(lat), "p90", sorted(lat)[int(0.9 * len(lat))])
print("0-LLM ходов", len(lat0), "latency median", statistics.median(lat0) if lat0 else None)
print("verdicts", collections.Counter(c["verdict"] for t in turns for c in t["cmds"]))
print("signals", collections.Counter(s for t in turns for s in t["signals"]))
sizes = [len(t["slices"][0]) for t in turns if t["slices"]]
print("slice size", sorted(collections.Counter(sizes).items()))

steps = [(t, s) for t in turns for s in t.get("steps", []) if s.get("status")]
print("\n== шаги, сопоставленные ходам:", len(steps), collections.Counter(s["status"] for _, s in steps))

print("\n== (б) срез тулов: шаги с ожидаемым тулом")
TOOLS_KNOWN = set()
for t in turns:
    for sl in t["slices"]:
        TOOLS_KNOWN |= set(sl)
res = collections.Counter()
miss_examples = collections.Counter()
for t, s in steps:
    exp = [e for e in s["expected"] if e in TOOLS_KNOWN or e.replace("_", "").isalpha()]
    exp = [e for e in exp if " " not in e]
    if not exp:
        continue
    offered = set(x for sl in t["slices"] for x in sl)
    executed = set(x for x, _ in t["executed"])
    for e in exp:
        if t["tier1"]:
            k = "tier1 (кодом)" + (" + тул исполнен" if e in executed else " без этого тула")
        elif not t["slices"]:
            k = "нет LLM-запроса"
        elif e in offered:
            k = "в срезе, исполнен" if e in executed else "в срезе, НЕ исполнен (промах модели)"
        else:
            k = "НЕ в срезе (промах правил)"
            miss_examples[(e, s["label"])] += 1
        res[k] += 1
        res["status " + s["status"] + " | " + k] += 1
for k, v in sorted(res.items()):
    print(f"  {v:4d}  {k}")
print("  промахи правил:", miss_examples.most_common(15))

print("\n== (а) звать ли LLM")
llm_only_tier1like = 0
TIER1_LIKE = {"stop_music", "set_volume", "set_voice", "set_speed", "set_pitch", "play_animation"}
passed = 0
for t in llm:
    v = [c["verdict"] for c in t["cmds"]]
    ex = {x for x, _ in t["executed"]}
    if v and all(x == "passed" for x in v):
        passed += 1
    if ex and ex <= TIER1_LIKE and all(x == "command" for x in v):
        llm_only_tier1like += 1
print("LLM-ходы, где все ответы passed:", passed)
print("LLM-ходы только с тулами из набора Tier-1 и без речи:", llm_only_tier1like)
st_tier = collections.Counter()
for t, s in steps:
    st_tier[("tier1" if t["tier1"] else ("llm" if t["final"] and int(t["final"]["llm_calls"]) else "0llm/нет итога"),
             s["status"])] += 1
print("шаги по пути и вердикту", sorted(st_tier.items()))

print("\n== (в) норма или сбой: шаги FAIL и сигналы кода")
fc = collections.Counter()
for t, s in steps:
    sig = bool(t["signals"]) or (t["final"] is None) or (t["final"] and t["final"]["status"] != "finished")
    fc[(s["status"], s.get("fail_class") or "-", "сигнал кода" if sig else "тихо")] += 1
for k, v in sorted(fc.items()):
    print(f"  {v:4d}  {k}")
