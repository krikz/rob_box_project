"""Разбор лога DJ-сета: таймлайн, треки, каркасы, сэмплы.

python an_set.py set1.full.log [set2.full.log ...]
"""
import collections
import re
import sys

import v2log as V

TS = re.compile(r"\[(\d{10})\.(\d+)\]")
ALL_SAMPLES = collections.Counter()
ALL_SKELETONS = []


def ts(line):
    m = TS.search(line)
    return float(m.group(1) + "." + m.group(2)[:3]) if m else None


def main():
    for f in sys.argv[1:]:
        lines = open(f, encoding="utf-8", errors="replace").read().splitlines()
        t0 = None
        tracks = []
        v2_tracks = []
        print("=" * 20, f)
        last_ts = None
        for line in lines:
            t = ts(line)
            if t:
                last_ts = t
            if "STT:" in line and "диджей сет" in line and t0 is None:
                t0 = t
                print(f"  +0.0  STT {line.split('STT:')[1].strip()[:80]}")
                continue
            if t0 is None or "tools(61)" in line or re.match(r"^\[dialogue_node-\d+\]\s+\[\d+\]", line):
                continue
            rel = f"{(last_ts or t0) - t0:+6.1f}"
            v2 = V.parse_started(line)
            if v2:
                v2_tracks.append(v2)
                print(f"  {rel}   V2 TRACK {v2['track_id']} bpm={v2['bpm']} deck={v2['deck']}")
                continue
            sv = V.parse_set_started(line)
            if sv:
                print(f"  {rel}   V2 SET {sv[0]} source={sv[1]}")
                continue
            pl = V.parse_plan(line)
            if pl:
                print(f"  {rel}   V2 PLAN row={pl['row']} mode={pl['mode']} hooks={pl['hooks'][:120]}")
                continue
            m = re.search(r"Запрос выполнения: (\w+) с параметрами (.*)", line)
            if m:
                name, args = m.groups()
                if name in ("speak_text",):
                    print(f"  {rel} speak_text")
                    continue
                print(f"  {rel} {name} {args[:170]}")
                if name == "compose_music":
                    tracks.append({"t": rel, "args": args})
                continue
            m = re.search(r"LLM REQUEST START .*thinking=(\S+)", line)
            if m and "disabled" not in m.group(1):
                print(f"  {rel} LLM thinking={m.group(1)}")
            m = re.search(r"Композиция: (.*)", line)
            if m:
                comp = m.group(1)
                print(f"  {rel}   COMP {comp[:200]}")
                sk = re.search(r"template=(\S+), kick=(\S+), hats=(\S+), lead=(\S+), bass=(\S+), pad=(\S+)", comp)
                if sk:
                    ALL_SKELETONS.append((f, sk.groups()))
            for pat, tag in ((r"club выбор: (.*)", "CLUB"), (r"club хук: (.*)", "HOOK"),
                             (r"история учтена: (\d+)", "HIST")):
                m = re.search(pat, line)
                if m:
                    print(f"  {rel}   {tag} {m.group(1)[:160]}")
            for m in re.finditer(r"loop\('([^']+)'", line):
                ALL_SAMPLES[m.group(1)] += 1
                print(f"  {rel}   LOOP {m.group(1)}")
            m = re.search(r"(d\d) >> play\(['\"]([^'\"]+)['\"](.*?sample=(\d+))?", line)
            if m:
                ALL_SAMPLES[f"play:{m.group(2)[:20]}|s={m.group(4)}"] += 1
            if re.search(r"\[ERROR\]|Traceback|Bug [BC]\b|Timeout ожидания", line):
                msg = re.sub(r'^.*?\] ', '', line)[:200]
                print(f"  {rel}   !! {msg}")
            m = re.search(r"DJ трек #(\d+)", line)
            if m:
                print(f"  {rel}   DJ TRACK #{m.group(1)}")
        print(f"  треков compose_music: {len(tracks)}, треков v2 (started): {len(v2_tracks)}")
    print("=" * 20, "КАРКАСЫ (template, kick, hats, lead, bass, pad)")
    seen = collections.Counter(sk for _f, sk in ALL_SKELETONS)
    for (f, sk) in ALL_SKELETONS:
        print(f"  {f.split('/')[-1]:16} {sk}  {'ПОВТОР x' + str(seen[sk]) if seen[sk] > 1 else ''}")
    for i, name in enumerate(("template", "kick", "hats", "lead", "bass", "pad")):
        c = collections.Counter(sk[i] for _f, sk in ALL_SKELETONS)
        print(f"  {name:8}: {dict(c.most_common())}")
    print("=" * 20, "СЭМПЛЫ/ЛУПЫ")
    for k, v in ALL_SAMPLES.most_common():
        print(f"  {v:3} {k}")


if __name__ == "__main__":
    main()
