"""final_stats26.py a,b,c,d [--vs=x,y,...] : итоговая статистика пар на живом канале (старые записи 23.10 + новые 26.10).
Для каждой пары голосов по ВСЕМ парам записей: макс / p95 / медиана / доля пар выше порога опознания 0.72 / число пар.
--vs: дополнительные голоса (например запасные), которые сравниваются только с основными."""
import itertools, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.stdout.reconfigure(encoding="utf-8")
import analyze26 as A

by, n_old, miss = A.load(True)
print("исключено чисткой ошибочных привязок (cos к своему центроиду < %.1f): %s\n" % (A.JUNK, A.EXCLUDED or "нет"))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
opts = {a.split("=")[0]: a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--") and "=" in a}
main = args[0].split(",")
extra = [c for c in opts.get("--vs", "").split(",") if c]


def st(a, b):
    cs = np.array([float(x @ y) for _, x, _ in by[a] for _, y, _ in by[b]])
    return cs.max(), float(np.percentile(cs, 95)), float(np.median(cs)), float((cs > 0.72).mean()) * 100, len(cs)


def selfst(c):
    cs = np.array([float(by[c][i][1] @ by[c][j][1]) for i, j in itertools.combinations(range(len(by[c])), 2)])
    return cs.min(), float(np.median(cs)), len(by[c])


print("СВОЯ К СВОЕЙ, все пары записей одного голоса: мин / медиана (записей)")
for c in main + extra:
    mn, md, n = selfst(c)
    print("  %-28s %.3f / %.3f  (%d)" % (c, mn, md, n))
print("\nК ЧУЖИМ, все пары записей: макс / p95 / медиана / доля >0.72 / n пар")
for a, b in itertools.combinations(main, 2):
    mx, p95, md, pc, n = st(a, b)
    print("  %-26s x %-26s %.3f / %.3f / %.3f / %5.1f%% / %d" % (a, b, mx, p95, md, pc, n))
if extra:
    print("\nЗАПАСНЫЕ против основных:")
    for e in extra:
        print("  %s" % e)
        for m in main:
            mx, p95, md, pc, n = st(e, m)
            print("     x %-26s %.3f / %.3f / %.3f / %5.1f%% / %d" % (m, mx, p95, md, pc, n))
        for e2 in extra:
            if e2 > e:
                mx, p95, md, pc, n = st(e, e2)
                print("     x %-26s %.3f / %.3f / %.3f / %5.1f%% / %d  (запасной с запасным)" % (e2, mx, p95, md, pc, n))
