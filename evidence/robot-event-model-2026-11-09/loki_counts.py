"""Счётчики строк в Loki на katana (только чтение, #4512)."""
import json
import sys
import time
import urllib.parse
import urllib.request

L = "http://10.1.1.249:3100/loki/api/v1"


def get(path, **params):
    url = f"{L}/{path}?{urllib.parse.urlencode(params)}"
    with urllib.request.urlopen(url, timeout=120) as r:
        return json.load(r)


def count(container, pat, rng="7d"):
    sel = f'{{container="{container}"}}'
    q = f'sum(count_over_time({sel} |= {json.dumps(pat)} [{rng}]))' if pat else f"sum(count_over_time({sel}[{rng}]))"
    d = get("query", query=q, time=int(time.time()))
    r = d.get("data", {}).get("result", [])
    return r[0]["value"][1] if r else "0"


if __name__ == "__main__":
    container = sys.argv[1]
    for pat in sys.argv[2:]:
        print(f"{pat or '<all>'}\t{count(container, pat)}")
