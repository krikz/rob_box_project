"""Строки по дням в Loki (только чтение, #4512)."""
import json
import sys
import time
import urllib.parse
import urllib.request

L = "http://10.1.1.249:3100/loki/api/v1"
container, pat = sys.argv[1], sys.argv[2]
end = int(time.time())
start = end - 30 * 86400
q = f'sum(count_over_time({{container="{container}"}} |= {json.dumps(pat)} [1d]))'
url = f"{L}/query_range?" + urllib.parse.urlencode(dict(query=q, start=start, end=end, step="86400"))
d = json.load(urllib.request.urlopen(url, timeout=180))
for s in d.get("data", {}).get("result", []):
    for t, v in s["values"]:
        print(time.strftime("%Y-%m-%d %H:%M", time.gmtime(float(t))), v)
print(d.get("status"), d.get("error", ""))
