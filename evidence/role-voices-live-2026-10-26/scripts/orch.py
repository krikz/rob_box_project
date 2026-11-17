"""orch.py : оркестратор замера голосов на ЖИВОМ канале (laptop -> Sound Pi, katana). Подкоманды:

  push                      скрипты на Sound Pi (~/vp26/scripts) и katana (~/vp26)
  synth SPEC.json           yandex/minimax синтез внутри voice-assistant + EQ харнесса + доставка на laptop и katana
  silero SPEC.json          Silero на katana + EQ, затем копия на laptop (для длительностей)
  e2e on|off|check [сек]    e2e_mode (одноразовый контейнер, не ros2 CLI внутри сервисных)
  capstart | capstop        запись /audio/speech_audio одноразовым контейнером (имя vp26cap)
  play IDS.txt GAP LOG.jsonl  проигрыш по одному на katana; гард e2e перед 1-й и каждой 6-й фразой
  emb                       эмбеддинги записей (resemblyzer, одноразовый контейнер) -> laptop
  pull                      emb26.json, ls cap, docker logs voice-assistant с метки -> laptop
Все сырые ответы пишутся в evidence/role-voices-live-2026-10-26/logs/. Ключи нигде не печатаются."""
import json, os, subprocess, sys, time, glob, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
EV = os.path.abspath(os.path.join(HERE, ".."))
WORK = os.environ.get("VP26_WORK", r"C:\Users\krikz\AppData\Local\Temp\claude\D--PROJECTS-rob-box-project\e17dc72b-faa8-4bc1-b2e3-b1a0a3ef4b6a\scratchpad\new26")
LOGS = os.path.join(EV, "logs")
os.makedirs(os.path.join(WORK, "eq"), exist_ok=True)
os.makedirs(LOGS, exist_ok=True)
SP = "ros2@10.1.1.22"
KT = "ros2@10.1.1.249"
PLINK = r"C:\Program Files\PuTTY\plink.exe"
PSCP = r"C:\Program Files\PuTTY\pscp.exe"


def now():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def logline(fname, text):
    with open(os.path.join(LOGS, fname), "a", encoding="utf-8") as f:
        f.write(text if text.endswith("\n") else text + "\n")


def run(args, input=None, timeout=600):
    r = subprocess.run(args, input=input, stdin=(subprocess.DEVNULL if input is None else None), capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout)
    return r.returncode, (r.stdout or "") + (r.stderr or "")


def sp(cmd, input=None, timeout=600):
    return run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", SP, cmd], input=input, timeout=timeout)


def kt(cmd, timeout=600):
    return run([PLINK, "-ssh", "-pw", "open", "-batch", KT, cmd], timeout=timeout)


def kt_put(files, dest):
    for i in range(0, len(files), 30):
        rc, out = run([PSCP, "-pw", "open", "-batch", "-q"] + files[i:i + 30] + ["%s:%s" % (KT, dest)])
        if rc != 0:
            print("PSCP FAIL", out[-300:])
            return False
    return True


def kt_get(remote, local):
    return run([PSCP, "-pw", "open", "-batch", "-q", "%s:%s" % (KT, remote), local])


def cmd_push():
    sp("mkdir -p ~/vp26/scripts ~/vp26/cap ~/vp26/syn")
    for f in ("ros1.sh", "cap26.py", "emb26.py", "eq26.sh"):
        rc, out = run(["scp", "-q", os.path.join(HERE, f), "%s:/home/ros2/vp26/scripts/%s" % (SP, f)])
        print("scp", f, rc, out.strip())
    sp("chmod +x ~/vp26/scripts/*.sh")
    kt("mkdir -p ~/vp26/eq")
    print(kt_put([os.path.join(HERE, "silero_batch26.py")], "/home/ros2/vp26/"))


def tally(out, name):
    for l in out.splitlines():
        if l.startswith("CALLS "):
            logline("synth_ledger.txt", "%s %s %s" % (now(), name, l))


def cmd_synth(spec):
    name = os.path.basename(spec)
    rc, out = run(["scp", "-q", spec, "%s:/home/ros2/vp26/spec_cur.json" % SP])
    assert rc == 0, out
    sp("docker exec voice-assistant mkdir -p /tmp/vp26/syn && docker cp /home/ros2/vp26/spec_cur.json voice-assistant:/tmp/vp26/spec_cur.json "
       "&& docker cp /home/ros2/vp26/scripts/eq26.sh voice-assistant:/tmp/vp26/eq26.sh")
    script = open(os.path.join(HERE, "synth26.py"), encoding="utf-8").read()
    rc, out = sp("docker exec -i voice-assistant python3 - /tmp/vp26/spec_cur.json /tmp/vp26/syn", input=script, timeout=3000)
    logline("synth_raw.txt", "=== %s %s\n%s" % (now(), name, out))
    print(out[-3000:])
    tally(out, name)
    rc, out = sp("docker exec voice-assistant bash /tmp/vp26/eq26.sh")
    print("eq:", out.strip()[-200:])
    rc, out = sp("rm -rf ~/vp26/eq_new && docker cp voice-assistant:/tmp/vp26/eq_new /home/ros2/vp26/eq_new && "
                 "docker cp voice-assistant:/tmp/vp26/syn/. /home/ros2/vp26/syn/ && ls ~/vp26/eq_new | wc -l")
    print("copied to host:", out.strip()[-100:])
    before = set(os.listdir(os.path.join(WORK, "eq")))
    rc, out = run(["scp", "-q", "%s:/home/ros2/vp26/eq_new/*.wav" % SP, os.path.join(WORK, "eq") + os.sep])
    print("scp to laptop", rc, out.strip()[-200:])
    new = [os.path.join(WORK, "eq", f) for f in sorted(set(os.listdir(os.path.join(WORK, "eq"))) - before)]
    print("new files:", len(new))
    kt("mkdir -p ~/vp26/eq")
    if new:
        print("to katana:", kt_put(new, "/home/ros2/vp26/eq/"))


def cmd_silero(spec):
    ok = kt_put([spec], "/home/ros2/vp26/spec_s.json")
    rc, out = kt("python3 ~/vp26/silero_batch26.py ~/vp26/spec_s.json 2>&1 | tail -5", timeout=3000)
    print(out[-800:])
    logline("synth_raw.txt", "=== %s silero %s\n%s" % (now(), os.path.basename(spec), out))
    for s in json.load(open(spec, encoding="utf-8")):
        loc = os.path.join(WORK, "eq", s["id"] + ".wav")
        if not os.path.exists(loc):
            kt_get("/home/ros2/vp26/eq/%s.wav" % s["id"], loc)


ROS = "source /opt/ros/humble/setup.bash; "


def ros(cmd, filt='', timeout=120):
    return sp("~/vp26/scripts/ros1.sh '%s%s' 2>&1%s" % (ROS, cmd, (' | ' + filt) if filt else ''), timeout=timeout)


def e2e_log_last():
    rc, out = sp("docker logs --since 6h voice-assistant 2>&1 | grep -E 'e2e_mode=(ON|OFF)' | tail -3")
    return out.strip()


def guard(extend=3600.0):
    """True только если param e2e_mode=True и ПОСЛЕДНЯЯ строка переключения в логе - ON с e2e-базой. Продлевает таймер."""
    rc, out = ros("ros2 param get /speaker_id_node e2e_mode --no-daemon", filt="grep -i boolean", timeout=60)
    val = "Boolean value is: True" in out
    last = ""
    ok_log = False
    rc, o2 = sp("docker logs --since 6h voice-assistant 2>&1 | grep -E 'e2e_mode=(ON|OFF)' | tail -1")
    last = o2.strip()
    ok_log = ("e2e_mode=ON" in last) and ("speakers.e2e.db" in last)
    if val and ok_log and extend:
        ros("ros2 param set /speaker_id_node e2e_release_after_s %s --no-daemon" % extend, filt="grep -i 'set parameter'", timeout=60)
    logline("e2e_guard.txt", "%s guard param_true=%s log_on=%s | %s" % (now(), val, ok_log, last[-230:]))
    return val and ok_log


def cmd_e2e(action, secs="3600"):
    secs = float(secs)
    if action == "check":
        print(guard(extend=0))
        return
    if action == "on":
        rc, out = ros("ros2 param get /speaker_id_node e2e_mode --no-daemon", filt="grep -i boolean", timeout=60)
        logline("e2e_mode_cmds.txt", "%s get -> %s" % (now(), out.strip()[-80:]))
        if "True" in out:
            rc, out = ros("ros2 param set /speaker_id_node e2e_mode false --no-daemon", filt="grep -i 'set parameter'", timeout=60)
            logline("e2e_mode_cmds.txt", "%s set false (был True, перезапуск режима для свежей строки ON) -> %s" % (now(), out.strip()[-80:]))
            time.sleep(3)
        rc, out = ros("ros2 param set /speaker_id_node e2e_mode true --no-daemon", filt="grep -i 'set parameter'", timeout=60)
        logline("e2e_mode_cmds.txt", "%s set true -> %s" % (now(), out.strip()[-80:]))
        time.sleep(3)
        rc, out = ros("ros2 param set /speaker_id_node e2e_release_after_s %s --no-daemon" % secs, filt="grep -i 'set parameter'", timeout=60)
        logline("e2e_mode_cmds.txt", "%s set e2e_release_after_s=%s -> %s" % (now(), secs, out.strip()[-80:]))
        rc, out = sp("docker logs --since 3m voice-assistant 2>&1 | grep -E 'e2e|E2E' | tail -12")
        logline("e2e_mode_cmds.txt", "%s логи:\n%s" % (now(), out))
        print(out)
        print("GUARD", guard(extend=0))
    elif action == "off":
        rc, out = ros("ros2 param set /speaker_id_node e2e_mode false --no-daemon", filt="grep -i 'set parameter'", timeout=60)
        logline("e2e_mode_cmds.txt", "%s set false -> %s" % (now(), out.strip()[-80:]))
        time.sleep(3)
        rc, out = sp("docker logs --since 3m voice-assistant 2>&1 | grep -E 'e2e|E2E' | tail -8")
        logline("e2e_mode_cmds.txt", "%s логи:\n%s" % (now(), out))
        print(out)


def cmd_capstart():
    sp("docker rm -f vp26cap >/dev/null 2>&1; cd ~/vp26 && nohup ~/vp26/scripts/ros1.sh '%spython3 /vp26/scripts/cap26.py' vp26cap > ~/vp26/cap.log 2>&1 &" % ROS)
    for _ in range(30):
        time.sleep(2)
        rc, out = sp("grep -c '^ready' ~/vp26/cap.log; docker ps --filter name=vp26cap --format '{{.Names}} {{.Status}}'")
        if out.startswith("1"):
            print("cap ready:", out.strip())
            return
    print("cap NOT ready", sp("tail -5 ~/vp26/cap.log")[1])


def cmd_capstop():
    print(sp("docker stop vp26cap 2>&1; ls ~/vp26/cap | wc -l")[1])


def robot_wait(t_play_end, maxwait=45.0):
    """Ждёт, пока робот договорит ответ на только что сыгранную фразу (иначе его речь попадает в запись микрофона и в
    эмбеддинг). События из лога voice-assistant (только ASCII-якоря): '(respeaker):' - фраза принята STT;
    'accumulated reason' - принята без вейка (ответа не будет); 'TTS provider_state' - озвучка ответа началась;
    'TTS finished event' - озвучка закончилась."""
    t_stop = time.time() + maxwait
    time.sleep(5.0)
    acc_seen_at = None
    while time.time() < t_stop:
        since = int(time.time() - t_play_end) + 12
        rc, out = sp("docker logs --since %ds voice-assistant 2>&1 | grep -aE 'respeaker\\):|accumulated reason|TTS provider_state|TTS finished event'" % since, timeout=60)
        pending = False
        for l in out.splitlines():
            if "(respeaker):" in l:
                pending = True
                if acc_seen_at is None:
                    acc_seen_at = time.time()
            elif "accumulated reason" in l:
                pending = False
            elif "TTS provider_state" in l:
                pending = True
            elif "TTS finished event" in l:
                pending = False
        if pending and acc_seen_at and time.time() - acc_seen_at > 20 and "TTS provider_state" not in out:
            return "no_tts_after_accept"
        if not pending:
            return "idle"
        time.sleep(2.0)
    return "timeout"


def cmd_play(ids_file, gap, logf):
    ids = [l.strip() for l in open(ids_file, encoding="utf-8") if l.strip()]
    gap = float(gap)
    if os.path.exists(logf):  # возобновление: уже сыгранные id из журнала пропускаем
        done = {json.loads(l)["id"] for l in open(logf, encoding="utf-8") if l.strip()}
        ids = [x for x in ids if x not in done]
    for i, pid in enumerate(ids):
        if i % 6 == 0 and not guard():
            print("ABORT: e2e гард не прошёл - не играю", flush=True)
            sys.exit(3)
        t0 = time.time()
        rc, out = kt("pactl set-sink-volume @DEFAULT_SINK@ 100%%; paplay /home/ros2/vp26/eq/%s.wav; echo rc=$?" % pid, timeout=120)
        t1 = time.time()
        st = robot_wait(t1)
        with open(logf, "a", encoding="utf-8") as f:
            f.write(json.dumps({"id": pid, "t_start": t0, "t_end": t1, "out": out.strip()[-80:], "robot": st}) + "\n")
        print(i + 1, len(ids), pid, "%.1f" % (t1 - t0), out.strip()[-30:], st, flush=True)
        time.sleep(gap)


def cmd_emb():
    rc, out = sp("~/vp26/scripts/ros1.sh 'python3 /vp26/scripts/emb26.py /vp26/cap /vp26/emb26.json' 2>&1 | tail -3", timeout=3000)
    print(out)
    cmd_pull(emb_only=True)


def cmd_pull(since=None, emb_only=False):
    run(["scp", "-q", "%s:/home/ros2/vp26/emb26.json" % SP, os.path.join(WORK, "emb26.json")])
    rc, out = sp("ls ~/vp26/cap")
    open(os.path.join(WORK, "caps.txt"), "w").write(out)
    if not emb_only:
        s = since or "12h"
        rc, out = sp("docker logs --since %s voice-assistant 2>&1" % s, timeout=300)
        open(os.path.join(WORK, "robot_log.txt"), "w", encoding="utf-8").write(out)
        print("robot log", len(out), "bytes")


if __name__ == "__main__":
    c = sys.argv[1]
    a = sys.argv[2:]
    {"push": cmd_push, "synth": cmd_synth, "silero": cmd_silero, "e2e": cmd_e2e, "capstart": cmd_capstart,
     "capstop": cmd_capstop, "play": cmd_play, "emb": cmd_emb, "pull": cmd_pull}[c](*a)
