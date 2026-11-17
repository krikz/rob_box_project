"""final_stats.py cfgs_csv : итоговая статистика по выбранным конфигам (те же привязки, что matrix.py)"""
import json, re, sys, os, wave, collections, itertools
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sel = sys.argv[1].split(",")
emb = json.load(open(os.path.join(HERE, "emb.json")))
plays = [json.loads(l) for l in open(os.path.join(HERE, "playlog_all6.jsonl"), encoding="utf-8") if l.strip()]
plays.sort(key=lambda x: x["t_start"])
caps = sorted((int(k.split("_")[0]) / 1000.0, k, v) for k, v in emb.items() if v)
used = set()
by = collections.defaultdict(list)
for p in plays:
    with wave.open(os.path.join(HERE, "eq", p["id"] + ".wav")) as w:
        d = w.getnframes() / w.getframerate()
    for t, k, v in caps:
        if k not in used and p["t_start"] <= t <= p["t_end"] + 9 and -3.0 <= v["dur"] - d <= 4.5:
            used.add(k)
            m = re.match(r"(.+)_p(\d+)(?:r\d+)?$", p["id"])
            e = np.array(v["emb"])
            by[m.group(1)].append((int(m.group(2)), e / (np.linalg.norm(e) + 1e-9)))
            break
GAL = 5
print("СВОЯ К СВОЕЙ (профиль = фразы 1..5, max по профилю; тест = фразы 6+)")
for c in sel:
    g = [v for ph, v in by[c] if ph <= GAL]
    t = [v for ph, v in by[c] if ph > GAL]
    if g and t:
        sc = [max(float(a @ b) for b in g) for a in t]
        print("  %-6s профиль=%d тест=%d  min=%.3f мед=%.3f макс=%.3f" % (c, len(g), len(t), min(sc), np.median(sc), max(sc)))
    else:
        print("  %-6s профиль=%d тест=%d  (мало данных)" % (c, len(g), len(t)))
print("\nК ЧУЖИМ: все пары записей A x B: min / медиана / МАКС   (n пар)")
for a, b in itertools.combinations(sel, 2):
    cs = [float(x @ y) for _, x in by[a] for _, y in by[b]]
    print("  %-6s x %-6s  %.3f / %.3f / %.3f   (%d)" % (a, b, min(cs), np.median(cs), max(cs), len(cs)))
