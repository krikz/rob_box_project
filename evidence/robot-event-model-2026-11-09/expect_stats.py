"""Что ждут шаги сценариев марафона (expected_tool_calls / must_not_call), #4512. Только чтение."""
import collections
import glob
import json
import os
import sys

ROOT = sys.argv[1]
exp = collections.Counter()
mnc = collections.Counter()
steps_total = 0
steps_with_tool = 0
uniq_texts = set()
for sc in glob.glob(os.path.join(ROOT, "*", "e2e-voice-artifacts-*", "*", "scenario.json")):
    d = json.load(open(sc, encoding="utf-8"))
    for s in d.get("steps", []):
        steps_total += 1
        uniq_texts.add(s.get("text", ""))
        acc = s.get("acceptance") or {}
        e = acc.get("expected_tool_calls") or []
        if e:
            steps_with_tool += 1
        for x in e:
            exp[x] += 1
        for x in acc.get("must_not_call") or []:
            mnc[x] += 1
print("steps", steps_total, "with expected_tool_calls", steps_with_tool, "unique texts", len(uniq_texts))
print("expected:", exp.most_common(60))
print("must_not_call:", mnc.most_common(30))
