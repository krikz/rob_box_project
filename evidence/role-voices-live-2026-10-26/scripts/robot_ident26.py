"""robot_ident26.py robot_log.txt playlog.jsonl... : что решил САМ speaker_id_node (боевой код в e2e-базе) на каждом проигрыше.
Для каждого проигрыша берётся первая строка `👤 Speaker:` в окне [t_start, t_end+14] с, по возможности, строкой
`identify candidates:` перед ней (best/second/gap). Итог: какие профили завёл робот на какой голос и не склеил ли
он два голоса в один / не раздвоил ли один."""
import collections, json, re, sys

sys.stdout.reconfigure(encoding="utf-8")
log = sys.argv[1]
plays = {}
for p in sys.argv[2:]:
    for l in open(p, encoding="utf-8"):
        if l.strip():
            d = json.loads(l)
            plays[d["id"]] = d
plays = sorted(plays.values(), key=lambda x: x["t_start"])
TS = re.compile(r"\[(\d{10}\.\d+)\]")
ev = []
for l in open(log, encoding="utf-8", errors="replace"):
    m = TS.search(l)
    if not m:
        continue
    t = float(m.group(1))
    if "👤 Speaker:" in l:
        m2 = re.search(r"id=([0-9a-f]{8})(.*?)(?:confidence=([0-9.]+))?\s*\(\d+ ms\)", l)
        if m2:
            ev.append((t, "spk", m2.group(1), "новый" in m2.group(2), m2.group(3) or ""))
    elif "identify candidates:" in l:
        m3 = re.search(r"best='[^']*'\(([0-9a-f]{8})\) score=([0-9.]+)(?: \| second='[^']*'\(([0-9a-f]{8})\) score=([0-9.]+))?", l)
        if m3:
            ev.append((t, "cand", m3.group(1), m3.group(2), m3.group(3) or "", m3.group(4) or ""))
ev.sort(key=lambda e: e[0])
rows = []
for p in plays:
    cfg = re.sub(r"_p\d+$", "", p["id"])
    spk = None
    for i, e in enumerate(ev):
        if e[1] == "spk" and p["t_start"] <= e[0] <= p["t_end"] + 14:
            cand = None
            for c in reversed(ev[:i]):
                if c[1] == "cand" and e[0] - c[0] < 3:
                    cand = c
                    break
            spk = (e, cand)
            break
    rows.append((p["id"], cfg, spk))
by = collections.OrderedDict()
for pid, cfg, spk in rows:
    if spk is None:
        print("%-28s  (решения нет)" % pid)
        continue
    e, cand = spk
    extra = ""
    if cand:
        extra = " best=%s score=%s second=%s %s" % (cand[2], cand[3], cand[4], cand[5])
    print("%-28s -> профиль %s %s conf=%s%s" % (pid, e[2], "(НОВЫЙ)" if e[3] else "", e[4], extra))
    by.setdefault(cfg, collections.Counter())[e[2]] += 1
print()
for cfg, c in by.items():
    print("%-24s профили: %s" % (cfg, dict(c)))
ids = collections.defaultdict(set)
for cfg, c in by.items():
    for pid in c:
        ids[pid].add(cfg)
merged = {pid: sorted(cfgs) for pid, cfgs in ids.items() if len(cfgs) > 1}
print("профилей всего: %d ; профилей, в которые попали ДВА И БОЛЕЕ голосов (склейка): %s" % (len(ids), merged or "нет"))
