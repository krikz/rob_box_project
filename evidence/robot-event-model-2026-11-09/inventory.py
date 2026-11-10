"""Инвентарь архива e2e (#4512): сколько прогонов, шагов, ходов, по датам и веткам. Только чтение."""
import collections
import glob
import json
import os
import sys

ROOT = sys.argv[1]
kinds = collections.Counter()
dates = collections.defaultdict(list)
branches = collections.Counter()
steps = collections.Counter()
nsteps_runs = 0
step_runs = set()
verdicts = collections.Counter()
fail_kinds = collections.Counter()
expires = collections.defaultdict(list)
for meta in glob.glob(os.path.join(ROOT, "*", "*", "_artifact.json")):
    m = json.load(open(meta, encoding="utf-8"))
    kind = m["name"].rsplit("-", 1)[0]
    kinds[kind] += 1
    dates[kind].append(m["created_at"])
    expires[kind].append(m["expires_at"])
    if kind == "e2e-voice-artifacts":
        branches[m.get("head_branch")] += 1
for sj in glob.glob(os.path.join(ROOT, "*", "e2e-voice-artifacts-*", "*", "steps.jsonl")):
    run = sj.split(os.sep)[-4] if os.sep in sj else sj
    step_runs.add(os.path.basename(os.path.dirname(os.path.dirname(sj))))
    for line in open(sj, encoding="utf-8"):
        line = line.strip()
        if line:
            steps[json.loads(line)["status"]] += 1
for sm in glob.glob(os.path.join(ROOT, "*", "e2e-voice-artifacts-*", "*", "summary.json")):
    s = json.load(open(sm, encoding="utf-8"))
    verdicts[s.get("verdict")] += 1
    fail_kinds[s.get("fail_kind")] += 1
for k in sorted(kinds):
    print(f"{k}: {kinds[k]} runs, created {min(dates[k])} .. {max(dates[k])}, expires {min(expires[k])} .. {max(expires[k])}")
print("artifacts head_branch:", branches.most_common(8))
print("step runs:", len(step_runs), "steps:", dict(steps), "total", sum(steps.values()))
print("act verdicts:", dict(verdicts), "fail_kind:", dict(fail_kinds))
