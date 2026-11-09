"""Прогон правил develop (Tier-1 и срез тулов) на текстах ходов архива (#4512). Без LLM, без сети.

Сравнивает: что код develop решил бы сейчас — с тем, что было в логе марафона (путь хода и исполненные тулы).
"""
import collections
import json
import sys

from pathlib import Path

_W = Path(__file__).resolve().parents[2] / "src"
sys.path[:0] = [str(_W / "rob_box_dialog"), str(_W / "rob_box_core"), str(_W / "rob_box_harness")]

from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn import tier1_of

rows = [json.loads(x) for x in open(sys.argv[1], encoding="utf-8")]
by = {}
for r in rows:
    if r["turn_id"] not in by or r.get("steps"):
        by[r["turn_id"]] = r
snap = Session("personality").snapshot
T1 = {"stop_music", "set_volume", "set_voice", "set_speed", "set_pitch"}
agree = collections.Counter()
llm_t1like_now = collections.Counter()
slice_now = collections.Counter()
for t in by.values():
    text = t["text"] or ""
    f = t["final"]
    after = text.split(" ", 1)[1] if text.startswith("робот ") else ""
    cmd, _ = tier1_of(text, after, snap)
    now = cmd.kind if cmd is not None and not cmd.disputed else None
    was = t["tier1"]
    agree[("было tier1" if was else "было LLM/иное", "сейчас tier1" if now else "сейчас LLM")] += 1
    ex = {x for x, _ in t["executed"]}
    if not was and f and int(f["llm_calls"]) and ex and ex <= T1 and all(c["verdict"] == "command" for c in t["cmds"]):
        llm_t1like_now[("tier1 сейчас ловит" if now else "tier1 и сейчас не ловит", tuple(sorted(ex)))] += 1
    for s in t.get("steps", []):
        exp = [e for e in s["expected"] if " " not in e and "=" not in e]
        if not exp or was:
            continue
        names = {e.name if hasattr(e, "name") else getattr(e, "tool", str(e)) for e in available_tools(snap, None,
                                                                                                     "personality",
                                                                                                     text)}
        for e in exp:
            slice_now["в срезе" if e in names else "НЕ в срезе"] += 1
print("путь хода: было → сейчас", sorted(agree.items()))
print("LLM-ходы с одним Tier-1-подобным тулом:", sorted(llm_t1like_now.items()))
print("ожидаемый тул в срезе develop (снимок без переноса групп):", dict(slice_now))
