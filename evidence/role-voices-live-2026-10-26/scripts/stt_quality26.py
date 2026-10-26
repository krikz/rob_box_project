"""stt_quality26.py wake_all.txt : из вывода wake26.py - по каждому конфигу доля принятых STT фраз и сходство распознанного текста
с исходным (difflib ratio после нормализации: нижний регистр, без пунктуации, ё->е, без слова «робот»).
Нужно, чтобы видеть, не разваливает ли голос смысл фраз (а не только вейк)."""
import collections, difflib, json, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.stdout.reconfigure(encoding="utf-8")
w = json.load(open(os.path.join(HERE, "..", "phrases.json"), encoding="utf-8"))
n = json.load(open(os.path.join(HERE, "..", "phrases_nw.json"), encoding="utf-8"))


def norm(s):
    s = s.lower().replace("ё", "е")
    s = re.sub(r"[^\w\s]", " ", s)
    s = re.sub(r"\bробот\b", " ", s)
    return " ".join(s.split())


ref = {}
for i, t in enumerate(w):
    ref[i + 1] = norm(t)
for i, t in enumerate(n):
    ref[i + 21] = norm(t)
rows = collections.defaultdict(list)
for l in open(sys.argv[1], encoding="utf-8"):
    m = re.match(r"(\S+)_p(\d+)\s+(HIT|MISS|NO_STT|NO_CAP)\s+(?:first)?\s*\|\s*(.*)$", l.rstrip("\n"))
    if not m:
        continue
    cfg, ph, verdict, text = m.group(1), int(m.group(2)), m.group(3), m.group(4)
    sim = difflib.SequenceMatcher(None, norm(text), ref[ph]).ratio() if verdict in ("HIT", "MISS") else 0.0
    rows[cfg].append((verdict, sim))
print("%-28s %4s %9s %9s" % ("конфиг", "n", "принято", "сходство"))
for c, rs in sorted(rows.items()):
    acc = [r for r in rs if r[0] in ("HIT", "MISS")]
    print("%-28s %4d %4d/%-4d %9.2f" % (c, len(rs), len(acc), len(rs), sum(s for _, s in acc) / max(1, len(acc))))
