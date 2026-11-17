"""prep.py: wav с Sound Pi (~/vp/syn) -> scratch/syn -> EQ харнесса -> katana ~/vp/eq"""
import os, subprocess, sys, glob

HERE = os.path.dirname(os.path.abspath(__file__))
syn = os.path.join(HERE, "syn")
eq = os.path.join(HERE, "eq")
os.makedirs(syn, exist_ok=True)
os.makedirs(eq, exist_ok=True)
subprocess.run(["scp", "-q", "ros2@10.1.1.22:/home/ros2/vp/syn/*.wav", syn + "/"], check=True)
new = []
for f in sorted(glob.glob(os.path.join(syn, "*.wav"))):
    o = os.path.join(eq, os.path.basename(f))
    if os.path.exists(o):
        continue
    subprocess.run(["ffmpeg", "-nostdin", "-y", "-loglevel", "error", "-i", f, "-af",
                    "highpass=f=100,volume=3.0,alimiter=limit=0.98,adelay=1500|all=1",
                    "-ac", "1", "-ar", "16000", o], check=True)
    new.append(o)
print("eq new:", len(new))
subprocess.run(["ssh", "ros2@10.1.1.249", "mkdir -p ~/vp/eq"], check=True)
if new:
    subprocess.run(["scp", "-q"] + new + ["ros2@10.1.1.249:/home/ros2/vp/eq/"], check=True)
print("pushed")
