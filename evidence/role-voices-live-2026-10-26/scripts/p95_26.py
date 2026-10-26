import os
import sys, itertools
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.stdout.reconfigure(encoding="utf-8")
import analyze26 as A

by, n_old, miss = A.load(True)
MIN = int(sys.argv[2]) if len(sys.argv) > 2 else 6
fixed = sys.argv[1].split(",")


def st(a, b):
    cs = np.array([float(x @ y) for _, x, _ in by[a] for _, y, _ in by[b]])
    return cs.max(), np.percentile(cs, 95), float(np.median(cs)), float((cs > 0.72).mean()) * 100, len(cs)


print("fixed pairs:")
for a, b in itertools.combinations(fixed, 2):
    mx, p95, md, pc, n = st(a, b)
    print("  %-22s %-22s max=%.3f p95=%.3f med=%.3f >0.72: %4.1f%%  n=%d" % (a, b, mx, p95, md, pc, n))
rows = []
for c in by:
    if c in fixed or len(by[c]) < MIN:
        continue
    ss = [st(c, f) for f in fixed]
    rows.append((max(s[1] for s in ss), c, len(by[c]), ss))
rows.sort()
print("\nкандидаты (n>=%d) по худшему p95 против фиксированных:" % MIN)
for w, c, n, ss in rows[:15]:
    print("  p95max=%.3f  %-26s n=%2d  " % (w, c, n) + "  ".join("[max %.2f p95 %.2f >.72 %3.0f%%]" % (s[0], s[1], s[3]) for s in ss))

