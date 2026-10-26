"""wake26.py robot_log.txt caps.txt eqdir playlog.jsonl... : вейк по привязке проигрыша к записи speech_audio.
Как analyze_wake2.py 23.10: проигрыш -> первая запись в окне [t_start, t_end+9] с длительностью +-3 с от eq-файла;
её ПРИНЯТО = первое событие stt в [t_cap, t_cap+10]; HIT если сразу после него (<=1.5 с) НЕТ `[backlog] accumulated
reason=no_wake`, MISS если есть, NO_STT если ПРИНЯТО не было. Для фраз без «Робот,» (p21+) вейк не считается."""
import collections, datetime, json, os, re, sys, wave

logf, capf, eqdir = sys.argv[1], sys.argv[2], sys.argv[3]
plays = []
for p in sys.argv[4:]:
    for l in open(p, encoding="utf-8"):
        if l.strip():
            plays.append(json.loads(l))
last = {}
for p in plays:
    last[p["id"]] = p
plays = sorted(last.values(), key=lambda x: x["t_start"])
TS = re.compile(r"\[(\d{10}\.\d+)\]")


def ts_of(l):
    m = TS.search(l)
    return float(m.group(1)) if m else None


ev = []
for l in open(logf, encoding="utf-8", errors="replace"):
    if "ПРИНЯТО (respeaker):" in l:
        ev.append((ts_of(l), "stt", l.split("ПРИНЯТО (respeaker):", 1)[1].strip()))
    elif "reason=no_wake" in l:
        ev.append((ts_of(l), "nowake", ""))
ev = [e for e in ev if e[0]]
ev.sort()
caps = []
for l in open(capf):
    m = re.match(r"(\d+)_(\d+)\.raw", l.strip())
    if m:
        caps.append((int(m.group(1)) / 1000.0, int(m.group(2)) / 32000.0, l.strip()))
caps.sort()
used = set()


def wav_dur(pid):
    with wave.open(os.path.join(eqdir, pid + ".wav")) as w:
        return w.getnframes() / w.getframerate()


rows = []
for p in plays:
    m = re.match(r"(.+)_p(\d+)(?:r\d+)?$", p["id"])
    nwake = int(m.group(2)) > 20
    d = wav_dur(p["id"])
    cap = None
    for t, dur, name in caps:
        if name not in used and p["t_start"] <= t <= p["t_end"] + 9 and -3.0 <= dur - d <= 4.5:
            cap = (t, dur, name)
            break
    if not cap:
        rows.append({"id": p["id"], "verdict": "NO_CAP", "text": "", "first": False, "nowake_phrase": nwake})
        continue
    used.add(cap[2])
    stts = [e for e in ev if e[1] == "stt" and cap[0] - 1.0 <= e[0] <= cap[0] + 10]
    if not stts:
        rows.append({"id": p["id"], "verdict": "NO_STT", "text": "", "first": False, "nowake_phrase": nwake})
        continue
    t0, _, text = stts[0]
    nw = any(e[1] == "nowake" and 0 <= e[0] - t0 <= 1.5 for e in ev)
    rows.append({"id": p["id"], "verdict": "MISS" if nw else "HIT", "text": text[:90],
                 "first": text.startswith("робот"), "nowake_phrase": nwake})

cfg = collections.OrderedDict()
for r in rows:
    cfg.setdefault(re.sub(r"_p\d+(r\d+)?$", "", r["id"]), []).append(r)
    print("%-28s %-7s %s | %s" % (r["id"], r["verdict"], "first" if r["first"] else "     ", r["text"]))
print()
tot_h = tot_n = 0
for c, rs in cfg.items():
    ws = [r for r in rs if not r["nowake_phrase"]]
    if not ws:
        print("CONFIG %-28s (без вейка: %d проигрышей, NO_CAP=%d NO_STT=%d)" % (c, len(rs), sum(r["verdict"] == "NO_CAP" for r in rs), sum(r["verdict"] == "NO_STT" for r in rs)))
        continue
    h = sum(r["verdict"] == "HIT" for r in ws)
    f = sum(r["first"] for r in ws)
    tot_h += h
    tot_n += len(ws)
    print("CONFIG %-28s wake %d/%d  (робот первым словом: %d; MISS=%d NO_STT=%d NO_CAP=%d)" % (
        c, h, len(ws), f, sum(r["verdict"] == "MISS" for r in ws), sum(r["verdict"] == "NO_STT" for r in ws),
        sum(r["verdict"] == "NO_CAP" for r in ws)))
print("ВСЕГО вейк %d/%d" % (tot_h, tot_n))
