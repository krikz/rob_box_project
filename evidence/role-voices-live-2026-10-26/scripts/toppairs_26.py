import os
import sys, itertools
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.stdout.reconfigure(encoding="utf-8")
import analyze26 as A

by, n_old, miss = A.load(True)
cfgs = sys.argv[1].split(",")
for a, b in itertools.combinations(cfgs, 2):
    cs = sorted(((float(x @ y), pa, pb) for _, x, pa in by[a] for _, y, pb in by[b]), reverse=True)[:4]
    print("%s x %s" % (a, b))
    for c, pa, pb in cs:
        print("   %.3f  %s  %s" % (c, pa, pb))

