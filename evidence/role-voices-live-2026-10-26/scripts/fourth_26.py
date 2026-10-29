import os
import sys, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.stdout.reconfigure(encoding="utf-8")
import analyze26 as A

by, n_old, miss = A.load(True)
trio = sys.argv[1].split(",")
rows = []
for c in by:
    if c in trio or len(by[c]) < 2:
        continue
    ps = [A.pair(by, c, t) for t in trio]
    rows.append((max(p[0] for p in ps), max(p[1] for p in ps), c, len(by[c]), [round(p[0], 3) for p in ps]))
rows.sort()
print("trio:", trio)
for mx, md, c, n, ps in rows[:25]:
    print("  %.3f (мед %.3f)  %-26s n=%2d  %s" % (mx, md, c, n, ps))

