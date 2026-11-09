"""Сколько тулов среза ход реально использовал (#4512): точность среза на архиве марафонов."""
import collections
import json
import statistics
import sys

rows = [json.loads(x) for x in open(sys.argv[1], encoding="utf-8")]
by = {}
for r in rows:
    if r["turn_id"] not in by or r.get("steps"):
        by[r["turn_id"]] = r
ALWAYS = {"command", "memory_save", "memory_search", "get_current_time", "search_web", "play_animation"}
extra, used_extra, n_used_any = [], [], 0
sz_tok = collections.defaultdict(list)
n = 0
for t in by.values():
    if not t["slices"]:
        continue
    n += 1
    offered = set(t["slices"][0])
    ex = {x for x, _ in t["executed"]}
    e = offered - ALWAYS
    extra.append(len(e))
    used_extra.append(len(e & ex))
    n_used_any += bool(ex)
    if t["final"] and int(t["final"]["llm_calls"]) == 1:
        sz_tok[len(offered)].append(int(t["final"]["tokens"]))
print("LLM-ходов со срезом", n, "ход исполнил хоть один тул", n_used_any)
print("тулов сверх всегда-видимых: в среднем", round(statistics.mean(extra), 2),
      "из них исполнено в среднем", round(statistics.mean(used_extra), 2))
print("ходов, где не исполнен ни один тул сверх всегда-видимых:", sum(1 for u in used_extra if u == 0))
print("токены хода с 1 вызовом LLM по размеру среза (медиана, n):",
      {k: (statistics.median(v), len(v)) for k, v in sorted(sz_tok.items())})
