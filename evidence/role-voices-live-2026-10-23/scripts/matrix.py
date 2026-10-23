"""matrix.py emb.json cap_dir_listing.txt playlog... [--gal N] : живая матрица косинусов.
Каждому проигрышу назначается первая запись speech_audio в окне [t_start, t_end+9] с длительностью
в пределах ±3 с от длительности eq-файла. Конфиг = id без _pNN (и без _rK повтора).
Профиль = первые N (по умолчанию 5) фраз конфига; 'тест' = остальные."""
import json, sys, re, os, collections, itertools
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
GAL = 5
for a in sys.argv[1:]:
    if a.startswith("--gal="):
        GAL = int(a.split("=")[1])
emb = json.load(open(args[0]))
plays = []
for p in args[1:]:
    for l in open(p, encoding="utf-8"):
        if l.strip():
            plays.append(json.loads(l))
plays.sort(key=lambda x: x["t_start"])
caps = []
for k, v in emb.items():
    if not v:
        continue
    t = int(k.split("_")[0]) / 1000.0
    caps.append((t, k, v))
caps.sort()

import wave
def wav_dur(pid):
    with wave.open(os.path.join(HERE, "eq", pid + ".wav")) as w:
        return w.getnframes() / w.getframerate()

used = set()
items = []  # (cfg, phr, rep, vec, voiced)
miss = []
for p in plays:
    d = wav_dur(p["id"])
    got = None
    for t, k, v in caps:
        if k in used:
            continue
        if p["t_start"] <= t <= p["t_end"] + 9 and -3.0 <= v["dur"] - d <= 4.5:
            got = (k, v)
            break
    if not got:
        miss.append(p["id"])
        continue
    used.add(got[0])
    m = re.match(r"(.+)_p(\d+)(?:r\d+)?$", p["id"])
    cfg, ph = m.group(1), int(m.group(2))
    items.append((cfg, ph, np.array(got[1]["emb"]), got[1]["voiced"]))
print("plays=%d matched=%d unmatched=%s" % (len(plays), len(items), miss))

def unit(v):
    return v / (np.linalg.norm(v) + 1e-9)

by = collections.OrderedDict()
for cfg, ph, v, vo in items:
    by.setdefault(cfg, []).append((ph, unit(v)))
cfgs = list(by)
print("\nСВОЯ К СВОЕЙ: профиль=первые %d фраз (по номеру), тест=остальные; cos = max по профилю" % GAL)
for c in cfgs:
    # дубли одного номера фразы (повторы) тоже идут в тест/профиль по номеру
    gal = [v for ph, v in by[c] if ph <= GAL]
    tst = [v for ph, v in by[c] if ph > GAL]
    if not gal or not tst:
        print("%-10s мало данных gal=%d test=%d" % (c, len(gal), len(tst)))
        continue
    sc = [max(float(np.dot(t, g)) for g in gal) for t in tst]
    print("%-10s n_test=%2d  min=%.3f median=%.3f max=%.3f" % (c, len(sc), min(sc), float(np.median(sc)), max(sc)))
print("\nК ЧУЖИМ: тест-фразы A против профиля B (max по профилю B), затем max по всем тест-фразам A; строка=A, столбец=B")
print("%-10s " % "" + " ".join("%7s" % c for c in cfgs))
for a in cfgs:
    tst = [v for ph, v in by[a] if ph > GAL]
    row = []
    for b in cfgs:
        if a == b:
            row.append("   -   ")
            continue
        gal = [v for ph, v in by[b] if ph <= GAL]
        if not tst or not gal:
            row.append("  n/a  ")
            continue
        row.append("%7.3f" % max(float(np.dot(t, g)) for t in tst for g in gal))
    print("%-10s " % a + " ".join(row))
print("\nМАКС по всем парам записей A x B (жёстко, без разделения профиль/тест)")
print("%-10s " % "" + " ".join("%7s" % c for c in cfgs))
for a in cfgs:
    row = []
    for b in cfgs:
        if a == b:
            row.append("   -   ")
            continue
        row.append("%7.3f" % max(float(np.dot(x, y)) for _, x in by[a] for _, y in by[b]))
    print("%-10s " % a + " ".join(row))
