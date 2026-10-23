"""analyze_wake2.py logs.txt caps.txt playlog.jsonl... : вейк по привязке проигрыша к записи speech_audio.
caps.txt = `ls ~/vp/cap` (имя t_ms_bytes.raw). Проигрыш -> первая запись в окне [t_start, t_end+9] c длительностью
±3 с от eq-файла; её ПРИНЯТО = первое событие stt в [t_cap, t_cap+10]; MISS если сразу после него (<=1.5 с) no_wake."""
import json, re, sys, datetime, collections, os, wave

HERE = os.path.dirname(os.path.abspath(__file__))
logf, capf = sys.argv[1], sys.argv[2]
plays = []
for p in sys.argv[3:]:
    for l in open(p, encoding="utf-8"):
        if l.strip():
            plays.append(json.loads(l))
plays.sort(key=lambda x: x["t_start"])

TS = re.compile(r"^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d+)Z ")


def epoch(s):
    return datetime.datetime.strptime(s[:26], "%Y-%m-%dT%H:%M:%S.%f").replace(tzinfo=datetime.timezone.utc).timestamp()


ev = []
for l in open(logf, encoding="utf-8", errors="replace"):
    m = TS.match(l)
    if not m:
        continue
    t = epoch(m.group(1))
    if "ПРИНЯТО (respeaker):" in l:
        ev.append((t, "stt", l.split("ПРИНЯТО (respeaker):", 1)[1].strip()))
    elif "reason=no_wake" in l:
        ev.append((t, "nowake", ""))
ev.sort()
caps = []
for l in open(capf):
    m = re.match(r"(\d+)_(\d+)\.raw", l.strip())
    if m:
        caps.append((int(m.group(1)) / 1000.0, int(m.group(2)) / 32000.0, l.strip()))
caps.sort()
used = set()


def wav_dur(pid):
    with wave.open(os.path.join(HERE, "eq", pid + ".wav")) as w:
        return w.getnframes() / w.getframerate()


rows = []
for p in plays:
    d = wav_dur(p["id"])
    cap = None
    for t, dur, name in caps:
        if name not in used and p["t_start"] <= t <= p["t_end"] + 9 and -3.0 <= dur - d <= 4.5:
            cap = (t, dur, name)
            break
    if not cap:
        rows.append({"id": p["id"], "verdict": "NO_CAP", "text": "", "first": False})
        continue
    used.add(cap[2])
    stts = [e for e in ev if e[1] == "stt" and cap[0] <= e[0] <= cap[0] + 10]
    if not stts:
        rows.append({"id": p["id"], "verdict": "NO_STT", "text": "", "first": False})
        continue
    t0, _, text = stts[0]
    nw = any(e[1] == "nowake" and 0 <= e[0] - t0 <= 1.5 for e in ev)
    rows.append({"id": p["id"], "verdict": "MISS" if nw else "HIT", "text": text[:90],
                 "first": text.startswith("робот")})

cfg = collections.OrderedDict()
for r in rows:
    cfg.setdefault(re.sub(r"_p\d+(r\d+)?$", "", r["id"]), []).append(r)
    print("%-12s %-7s %s | %s" % (r["id"], r["verdict"], "first" if r["first"] else "     ", r["text"]))
print()
for c, rs in cfg.items():
    h = sum(r["verdict"] == "HIT" for r in rs)
    f = sum(r["first"] for r in rs)
    print("CONFIG %-8s wake %d/%d  (робот первым словом: %d; MISS=%d NO_STT=%d NO_CAP=%d)" % (
        c, h, len(rs), f, sum(r["verdict"] == "MISS" for r in rs), sum(r["verdict"] == "NO_STT" for r in rs),
        sum(r["verdict"] == "NO_CAP" for r in rs)))
