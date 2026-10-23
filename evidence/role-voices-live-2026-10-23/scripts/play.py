"""play.py ids.txt gap_sec playlog.jsonl : проигрывает eq-wav на katana по одному.
ГАРД (#3931, инцидент 23.10): перед первой и каждой 6-й фразой читает e2e_mode у speaker_id_node;
не True -> выход, ничего не играет (боевую БД дикторов трогать нельзя)."""
import json, subprocess, sys, time

ids = [l.strip() for l in open(sys.argv[1], encoding="utf-8") if l.strip()]
gap = float(sys.argv[2])
log = sys.argv[3]
CHK = ("ssh -o ConnectTimeout=8 ros2@10.1.1.22 'docker exec voice-assistant bash -lc \"source /opt/ros/humble/setup.bash; "
       "source /ws/install/setup.bash; ros2 param get /speaker_id_node e2e_mode --no-daemon 2>&1 | grep -i boolean\"'")


def e2e_on():
    r = subprocess.run(CHK, shell=True, capture_output=True, text=True)
    return "True" in r.stdout


for i, pid in enumerate(ids):
    if i % 6 == 0 and not e2e_on():
        print("ABORT: e2e_mode не True - не играю", flush=True)
        sys.exit(3)
    t0 = time.time()
    r = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=8", "ros2@10.1.1.249",
         "pactl set-sink-volume @DEFAULT_SINK@ 100%%; paplay /home/ros2/vp/eq/%s.wav; echo rc=$?" % pid],
        capture_output=True, text=True)
    t1 = time.time()
    with open(log, "a", encoding="utf-8") as f:
        f.write(json.dumps({"id": pid, "t_start": t0, "t_end": t1, "out": (r.stdout + r.stderr).strip()[-80:]}) + "\n")
    print(i + 1, len(ids), pid, "%.1f..%.1f" % (t0, t1), r.stdout.strip(), flush=True)
    time.sleep(gap)
