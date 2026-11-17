import os
import sys, itertools
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.stdout.reconfigure(encoding="utf-8")
import analyze26 as A

by, n_old, miss = A.load(True)
cfgs = sys.argv[1].split(",")
cent = {}
for c in cfgs:
    v = np.mean([x for _, x, _ in by[c]], axis=0)
    cent[c] = v / np.linalg.norm(v)
for c in cfgs:
    print("==", c, "n=", len(by[c]))
    for k, v, pid in by[c]:
        own = float(v @ cent[c])
        others = {o[2:10]: round(float(v @ cent[o]), 3) for o in cfgs if o != c}
        flag = ""
        if own < 0.75 or max(others.values()) > own - 0.05:
            flag = "  <<<"
        if flag:
            print("  %-34s own=%.3f others=%s%s" % (pid, own, others, flag))

