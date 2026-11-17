"""analyze26.py : матрица косинусов ЖИВОГО канала, старые записи 23.10 + новые записи 26.10 вместе.
Привязка проигрыш -> запись /audio/speech_audio как в matrix.py 23.10: первая необработанная запись в окне
[t_start, t_end+9] с длительностью в пределах [-3.0, +4.5] с от длительности eq-файла.

  analyze26.py screen                 - все пары (макс/медиана по всем парам записей) для новых против всех
  analyze26.py best K [--pool a,b,..] - лучшие K-наборы по худшей паре (макс по всем парам записей)
  analyze26.py report a,b,c,d [--gal 5] - своя к своей (профиль=первые N записей конфига, тест=остальные;
                                         cos=макс по профилю) и к чужим (макс/медиана по всем парам + профиль/тест)
Данные берутся из закоммиченных файлов рядом (воспроизводимо без робота):
  data/emb_2026-10-23.json + data/playlog_2026-10-23.jsonl   - 254 проигрыша 23.10 (эмбеддинги записей ReSpeaker)
  data/emb_2026-10-26.json + playlogs/playlog_*.jsonl         - проигрыши 26.10
  data/eq_durations.json                                      - длительности eq-файлов для привязки
(эмбеддинги округлены до 5 знаков). VP26_RAW=1 отключает чистку ошибочных привязок."""
import collections, glob, itertools, json, os, re, sys
import numpy as np

EV = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
DATA = os.path.join(EV, "data")
PLAYLOGS = os.path.join(EV, "playlogs")
DUR = json.load(open(os.path.join(DATA, "eq_durations.json")))

# имена конфигов 23.10 -> читаемые; конфиг 26.10 с тем же голосом сливается с ним (если слияние включено)
OLD_NAMES = {
    "RM00": "m-ReliableMan", "BT2": "m-Bad-temperedBoy+2", "HC00": "m-HandsomeChildhoodFriend", "AG00": "m-AttractiveGuy",
    "PG00": "m-PessimisticGirl", "PF": "m-presenter_female", "A1": "m-audiobook_male_1", "A2": "m-audiobook_male_2",
    "UB": "m-cute_boy", "CB": "m-clever_boy", "LJ": "m-male-qn-jingying", "LB": "m-male-qn-badao", "LD": "m-male-qn-daxuesheng",
    "PM": "m-presenter_male", "QQ": "m-male-qn-qingse", "FY": "m-female-yujie", "FC": "m-female-chengshu",
    "SEU": "s-eugene", "SAI": "s-aidar", "SKS": "s-kseniya", "SXE": "s-xenia", "SBA": "s-baya",
    "BT00": "m-Bad-temperedBoy+0", "BT4": "m-Bad-temperedBoy+4",
}
SKIP_OLD = {"RMc", "BTc", "HCc", "AGc", "RMf80", "RMf88", "RMf114", "RMf125"}
JUNK = 0.6        # порог чистки ошибочных привязок (см. load())
EXCLUDED = []     # (конфиг, id проигрыша, cos к центроиду) - что исключила чистка


def bind(emb, plays):
    plays = sorted(plays, key=lambda x: x["t_start"])
    caps = sorted((int(k.split("_")[0]) / 1000.0, k, v) for k, v in emb.items() if v)
    used, items, miss = set(), [], []
    for p in plays:
        d = DUR[p["id"]]
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
        e = np.array(got[1]["emb"])
        items.append((m.group(1), int(m.group(2)), e / (np.linalg.norm(e) + 1e-9), p["id"], got[0]))
    return items, miss


def load(merge=True):
    by = collections.OrderedDict()
    # старые
    emb = json.load(open(os.path.join(DATA, "emb_2026-10-23.json")))
    plays = [json.loads(l) for l in open(os.path.join(DATA, "playlog_2026-10-23.jsonl"), encoding="utf-8") if l.strip()]
    items, miss = bind(emb, plays)
    for cfg, ph, v, pid, cap in items:
        if cfg in SKIP_OLD:
            continue
        name = OLD_NAMES.get(cfg, cfg)
        by.setdefault(name if merge else name + "@23", []).append((("a", ph), v, pid))
    n_old = sum(len(x) for x in by.values())
    # новые
    p26 = os.path.join(DATA, "emb_2026-10-26.json")
    miss26 = []
    if os.path.exists(p26):
        emb = json.load(open(p26))
        plays = []
        for f in sorted(glob.glob(os.path.join(PLAYLOGS, "playlog_*.jsonl"))):
            plays += [json.loads(l) for l in open(f, encoding="utf-8") if l.strip()]
        # повтор одного id (перепроигрыш) - берём последний
        last = {}
        for p in sorted(plays, key=lambda x: x["t_start"]):
            last[p["id"]] = p
        items, miss26 = bind(emb, list(last.values()))
        for cfg, ph, v, pid, cap in items:
            by.setdefault(cfg, []).append((("b", ph), v, pid))
    for k in by:
        by[k].sort(key=lambda x: x[0])
    # Чистка ошибочных привязок: запись, у которой cos к центроиду СВОЕГО голоса < JUNK (при >= 8 записях у голоса), - это
    # не наш проигрыш, а чужой звук в окне (ответ самого робота, разговор в мастерской). Исключаются и ПЕЧАТАЮТСЯ.
    if os.environ.get("VP26_RAW") != "1":
        for k, lst in list(by.items()):
            if len(lst) < 8:
                continue
            c = np.mean([v for _, v, _ in lst], axis=0)
            c = c / np.linalg.norm(c)
            keep = [x for x in lst if float(x[1] @ c) >= JUNK]
            for x in lst:
                if float(x[1] @ c) < JUNK:
                    EXCLUDED.append((k, x[2], round(float(x[1] @ c), 3)))
            by[k] = keep
    return by, n_old, miss26


def pair(by, a, b):
    cs = [float(x @ y) for _, x, _ in by[a] for _, y, _ in by[b]]
    return max(cs), float(np.median(cs)), float(np.mean(cs)), len(cs)


def prof_test(by, a, b, gal):
    """тест-записи a против профиля b (первые gal записей b): макс по профилю, затем (макс, медиана) по тестам."""
    g = [v for _, v, _ in by[b][:gal]]
    t = [v for _, v, _ in by[a][gal:]]
    if not g or not t:
        return None
    sc = [max(float(x @ y) for y in g) for x in t]
    return max(sc), float(np.median(sc)), len(sc)


def cmd_screen(by):
    cfgs = [c for c in by]
    newc = [c for c in cfgs if any(k[0] == "b" for k, _, _ in by[c])]
    print("конфиги:", len(cfgs), " новых:", len(newc))
    print("\nСВОЯ К СВОЕЙ (все пары записей одного голоса): мин / медиана")
    for c in newc:
        cs = [float(by[c][i][1] @ by[c][j][1]) for i, j in itertools.combinations(range(len(by[c])), 2)]
        print("  %-26s n=%2d  min=%.3f med=%.3f" % (c, len(by[c]), min(cs) if cs else float("nan"), float(np.median(cs)) if cs else float("nan")))
    print("\nК ЧУЖИМ для каждого нового: 6 самых далёких конфигов (макс / медиана по всем парам записей) и 3 самых близких")
    for c in newc:
        rows = sorted(((pair(by, c, o)[0], pair(by, c, o)[1], o) for o in cfgs if o != c))
        print("  %s (n=%d)" % (c, len(by[c])))
        print("     дальние : " + "  ".join("%s %.2f/%.2f" % (o, mx, md) for mx, md, o in rows[:6]))
        print("     близкие : " + "  ".join("%s %.2f/%.2f" % (o, mx, md) for mx, md, o in rows[-3:]))


MINREC = 3


def cmd_best(by, K, pool):
    cfgs = pool or [c for c in by if len(by[c]) >= MINREC]
    P = {}
    for a, b in itertools.combinations(cfgs, 2):
        P[(a, b)] = P[(b, a)] = pair(by, a, b)
    res = []
    for s in itertools.combinations(cfgs, K):
        mx = max(P[(a, b)][0] for a, b in itertools.combinations(s, 2))
        md = max(P[(a, b)][1] for a, b in itertools.combinations(s, 2))
        res.append((mx, md, s))
    res.sort()
    print("TOP K=%d из %d конфигов (по худшей паре, макс по всем парам записей)" % (K, len(cfgs)))
    for mx, md, s in res[:25]:
        print("  %.3f (худшая медиана %.3f)  %s" % (mx, md, "  ".join(s)))


def cmd_report(by, cfgs, gal):
    print("СВОЯ К СВОЕЙ (профиль = первые %d записей, тест = остальные; cos = макс по профилю)" % gal)
    for c in cfgs:
        pt = prof_test(by, c, c, gal) if len(by[c]) > gal else None
        if len(by[c]) <= gal:
            print("  %-28s записей=%d (мало)" % (c, len(by[c])))
            continue
        g = [v for _, v, _ in by[c][:gal]]
        sc = [max(float(x @ y) for y in g) for _, x, _ in by[c][gal:]]
        print("  %-28s тест=%2d  min=%.3f мед=%.3f макс=%.3f" % (c, len(sc), min(sc), float(np.median(sc)), max(sc)))
    print("\nК ЧУЖИМ, профиль/тест: тест-записи A против профиля B (макс по профилю), затем МАКС / медиана по тестам A")
    print("%-28s " % "" + " ".join("%14s" % c[:14] for c in cfgs))
    for a in cfgs:
        row = []
        for b in cfgs:
            if a == b:
                row.append("%14s" % "-")
                continue
            r = prof_test(by, a, b, gal)
            row.append("%14s" % ("n/a" if r is None else "%.3f/%.3f" % (r[0], r[1])))
        print("%-28s " % a + " ".join(row))
    print("\nК ЧУЖИМ, все пары записей A x B: МАКС / медиана / (n пар)")
    print("%-28s " % "" + " ".join("%18s" % c[:18] for c in cfgs))
    for a in cfgs:
        row = []
        for b in cfgs:
            if a == b:
                row.append("%18s" % "-")
                continue
            mx, md, mn, n = pair(by, a, b)
            row.append("%18s" % ("%.3f/%.3f(%d)" % (mx, md, n)))
        print("%-28s " % a + " ".join(row))
    worst = max(pair(by, a, b)[0] for a, b in itertools.combinations(cfgs, 2))
    worst_pt = max((prof_test(by, a, b, gal) or (0,))[0] for a in cfgs for b in cfgs if a != b)
    print("\nХУДШАЯ ПАРА: макс по всем парам записей = %.3f ; макс профиль/тест = %.3f" % (worst, worst_pt))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    merge = "--nomerge" not in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = {a.split("=")[0]: a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--") and "=" in a}
    by, n_old, miss26 = load(merge)
    print("записей: старых %d, новых %d, несвязанных проигрышей 26.10: %s ; исключено чисткой (cos к своему центроиду < %.1f): %s" % (n_old, sum(len(x) for x in by.values()) - n_old, miss26, JUNK, EXCLUDED or "нет"))
    if args[0] == "screen":
        cmd_screen(by)
    elif args[0] == "best":
        MINREC = int(opts.get("--min", "3"))
        pool = opts.get("--pool", "")
        cmd_best(by, int(args[1]), [c for c in pool.split(",") if c] or None)
    elif args[0] == "report":
        cmd_report(by, args[1].split(","), int(opts.get("--gal", "5")))
