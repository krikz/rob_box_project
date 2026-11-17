"""Ходы из логов voice-assistant в архиве марафонов + привязка к шагам (#4512). Только чтение архива.

Выход: turns.jsonl (ход: текст STT, tier1, срез тулов, исполненные тулы, итог 📊 [turn], сигналы сбоя,
метка шага и его вердикт, если ход сопоставлен шагу). Текст остаётся в scratchpad, в репо не идёт.
"""
import difflib
import glob
import json
import os
import re
import sys

ROOT, OUT = (sys.argv + ["", ""])[1:3]
TS = re.compile(r"\[(\d{10}\.\d+)\]")
STT = re.compile(r"\[dialog_node\]: 🎤 STT: (.*)$")
TID = re.compile(r"\bturn(?:_id)?=([0-9a-f]{12})\b")
TOOLS = re.compile(r"^\[dialog_node-\d+\]\s+tools\((\d+)\): (.*)$")
TIER1 = re.compile(r"tier1 kind=(\S+)")
EXEC = re.compile(r"📨 \[execute\] событие (\S+) ([0-9a-f]{12})/\d+ (\S+)")
CMD = re.compile(r"🧾 \[command\] provider=(\S+) mode=(\S+) verdict=(\S+) actions=(\[.*?\])")
FINAL = re.compile(r"📊 \[turn\] turn_id=([0-9a-f]{12}) epoch=(\d+) agent=(\S+) stage=(\S+) status=(\S+) "
                   r"latency_ms=(\d+) llm_calls=(\d+) llm_provider=(\S*) tokens=(\d+)")
SIGNALS = {
    "stale": "stale_epoch",
    "cancel_interrupt": "ход отменён перебиванием",
    "violation": "VIOLATION",
    "late": "поздний итог",
    "tool_error": "завершился с ошибкой",
    "hang_4135": "#4135",
    "echo": "[echo]",
    "error_line": "[ERROR]",
    "invalid": "verdict=invalid",
}


def norm(s):
    return re.sub(r"[^а-яёa-z0-9 ]", " ", s.lower().replace("ё", "е")).split()


def parse_log(path):
    turns, order = {}, []
    pending = {"tools": [], "signals": set(), "cmds": []}
    last_stt, cur = None, None
    for line in open(path, encoding="utf-8", errors="replace"):
        m = TS.search(line)
        ts = float(m.group(1)) if m else None
        s = STT.search(line)
        if s:
            last_stt = (s.group(1).strip(), ts)
            pending = {"tools": [], "signals": set(), "cmds": []}
            cur = None
            continue
        t = TOOLS.match(line)
        f = FINAL.search(line)
        ids = TID.findall(line) if "dialog_node" in line else []
        e = EXEC.search(line)
        if e:
            ids = [e.group(2)]
        tid = ids[0] if ids else None
        if tid and tid not in turns:
            turns[tid] = {"turn_id": tid, "text": last_stt[0] if last_stt else None,
                          "stt_ts": last_stt[1] if last_stt else None, "tier1": None, "slices": [],
                          "executed": [], "cmds": [], "signals": set(), "final": None}
            turns[tid]["slices"] += pending["tools"]
            turns[tid]["signals"] |= pending["signals"]
            turns[tid]["cmds"] += pending["cmds"]
            pending = {"tools": [], "signals": set(), "cmds": []}
            order.append(tid)
        if tid:
            cur = tid
        target = turns[cur] if cur and turns[cur]["final"] is None else pending
        if t:
            target["tools" if target is pending else "slices"].append(t.group(2).split(", "))
        c = CMD.search(line)
        if c:
            (target["cmds"]).append({"provider": c.group(1), "mode": c.group(2), "verdict": c.group(3),
                                     "actions": c.group(4)})
        for k, pat in SIGNALS.items():
            if pat in line and ("dialog_node" in line or k in ("echo", "hang_4135")):
                target["signals"].add(k)
        if tid:
            rec = turns[tid]
            m1 = TIER1.search(line)
            if m1:
                rec["tier1"] = m1.group(1)
            if e:
                rec["executed"].append((e.group(1), e.group(3)))
            if f:
                rec["final"] = dict(zip(["turn_id", "epoch", "agent", "stage", "status", "latency_ms", "llm_calls",
                                         "llm_provider", "tokens"], f.groups()))
                rec["final_ts"] = ts
                cur = None
    return [turns[t] for t in order]


def steps_of(run_dir):
    out = {}
    for sc in glob.glob(os.path.join(run_dir, "e2e-voice-artifacts-*", "*", "scenario.json")):
        for s in json.load(open(sc, encoding="utf-8")).get("steps", []):
            acc = s.get("acceptance") or {}
            out[s["label"]] = {"text": s.get("text", ""), "expected": acc.get("expected_tool_calls") or [],
                               "must_not_call": acc.get("must_not_call") or []}
    for vl in glob.glob(os.path.join(run_dir, "e2e-voice-logs-*", "e2e_verdict.log")):
        for line in open(vl, encoding="utf-8", errors="replace"):
            if line.startswith("E2E_STEP "):
                p = line.split()
                if len(p) >= 3 and p[1] in out:
                    out[p[1]]["status"] = p[2]
                    out[p[1]]["fail_class"] = p[3] if len(p) > 3 else ""
    return out


def main():
    seen = set()
    n_runs = 0
    with open(OUT, "w", encoding="utf-8") as fo:
        for log in sorted(glob.glob(os.path.join(ROOT, "*", "e2e-voice-logs-*", "voice_e2e_*.log"))):
            run_dir = os.path.dirname(os.path.dirname(log))
            run = os.path.basename(run_dir)
            n_runs += 1
            steps = steps_of(run_dir)
            turns = parse_log(log)
            used = set()
            for label, st in steps.items():  # шаг -> лучший по тексту ход этого прогона
                want = norm(st["text"])
                best, score = None, 0.0
                for tr in turns:
                    if tr["turn_id"] in used or not tr["text"]:
                        continue
                    r = difflib.SequenceMatcher(None, want, norm(tr["text"])).ratio()
                    if r > score:
                        best, score = tr, r
                if best is not None and score >= 0.6:
                    used.add(best["turn_id"])
                    best.setdefault("steps", []).append({"run": run, "label": label, "score": round(score, 2), **st})
            for tr in turns:
                if tr["turn_id"] in seen and not tr.get("steps"):
                    continue
                seen.add(tr["turn_id"])
                tr["run"] = run
                tr["signals"] = sorted(tr["signals"])
                fo.write(json.dumps(tr, ensure_ascii=False) + "\n")
    print("runs", n_runs, "turns", len(seen))


if __name__ == "__main__":
    main()
