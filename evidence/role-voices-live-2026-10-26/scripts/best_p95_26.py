import os
import sys, itertools
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.stdout.reconfigure(encoding="utf-8")
import analyze26 as A

by, n_old, miss = A.load(True)
MIN = int(sys.argv[1])
cfgs = [c for c in by if len(by[c]) >= MIN]
print("конфигов n>=%d: %d" % (MIN, len(cfgs)))
P = {}
for a, b in itertools.combinations(cfgs, 2):
    cs = np.array([float(x @ y) for _, x, _ in by[a] for _, y, _ in by[b]])
    P[(a, b)] = P[(b, a)] = (float(np.percentile(cs, 95)), float(cs.max()), float(np.median(cs)))
res = []
for s in itertools.combinations(cfgs, 4):
    pr = [P[(a, b)] for a, b in itertools.combinations(s, 2)]
    res.append((max(p[0] for p in pr), max(p[1] for p in pr), max(p[2] for p in pr), s))
res.sort()
print("TOP по худшему p95 пары:")
for p95, mx, md, s in res[:12]:
    print("  p95=%.3f max=%.3f med=%.3f  %s" % (p95, mx, md, "  ".join(s)))
res.sort(key=lambda r: r[1])
print("TOP по худшему max пары:")
for p95, mx, md, s in res[:8]:
    print("  max=%.3f p95=%.3f med=%.3f  %s" % (mx, p95, md, "  ".join(s)))

