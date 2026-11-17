"""Выгрузка строк voice-assistant из Loki за 7 дней по фильтру (только чтение, #4512). Окна по 6 ч: иначе
Loki на katana отвечает 429 «too many outstanding requests». Пишет в файл scratchpad."""
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

L = "http://10.1.1.249:3100/loki/api/v1"
OUT, regex = sys.argv[1], sys.argv[2]
WIN = 6 * 3600 * 10**9
end_ns = int(time.time() * 1e9)
lines = {}
w0 = end_ns - 7 * 86400 * 10**9
while w0 < end_ns:
    w1 = min(w0 + WIN, end_ns)
    cursor = w0
    while cursor < w1:
        q = '{container="voice-assistant"} |~ ' + json.dumps(regex)
        url = f"{L}/query_range?" + urllib.parse.urlencode(dict(query=q, start=cursor, end=w1, limit=1000,
                                                                  direction="forward"))
        for attempt in range(6):
            try:
                d = json.load(urllib.request.urlopen(url, timeout=300))
                break
            except urllib.error.HTTPError as e:
                if e.code != 429:
                    raise
                time.sleep(5 * (attempt + 1))
        else:
            raise SystemExit("429 не проходит")
        batch = [(int(ts), ln) for s in d["data"]["result"] for ts, ln in s["values"]]
        for ts, ln in batch:
            lines[(ts, ln)] = 1
        if len(batch) < 1000:
            break
        cursor = max(ts for ts, _ in batch) + 1
        time.sleep(1)
    w0 = w1
    time.sleep(1)
with open(OUT, "w", encoding="utf-8") as fo:
    for ts, ln in sorted(lines):
        fo.write(f"{ts}\t{ln.rstrip()}\n")
print("lines", len(lines))
