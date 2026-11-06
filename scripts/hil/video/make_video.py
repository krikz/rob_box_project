#!/usr/bin/env python3
"""Сборка ролика из прогона: титул, основной отрезок (compose.py), итоговая карточка с честной сводкой, MP4 и GIF.

Только настоящие кадры прогона; карточки — текст из route.log. Запуск в контейнере rob_box_sim:hil:
  make_video.py RUN_DIR OUT_DIR T0 T1 [--gif-mb 2]
RUN_DIR — каталог прогона (frames/, maprec/, route.log), T0..T1 — sim time основного отрезка.
"""
import re
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

run, out = Path(sys.argv[1]), Path(sys.argv[2])
t0, t1 = float(sys.argv[3]), float(sys.argv[4])
gif_mb = float(sys.argv[sys.argv.index("--gif-mb") + 1]) if "--gif-mb" in sys.argv else 2.0
out.mkdir(parents=True, exist_ok=True)
here = Path(__file__).resolve().parent
W, H = 1280, 720
B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
R = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def sh(*a):
    subprocess.run([str(x) for x in a], check=True)


def card(path, title, lines, accent):
    img = Image.new("RGB", (W, H), (14, 16, 22))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, 14, H], fill=accent)
    d.text((70, 90), title, fill=(255, 255, 255), font=ImageFont.truetype(B, 54))
    y = 190
    for ln in lines:
        d.text((70, y), ln, fill=(205, 210, 220), font=ImageFont.truetype(R, 25))
        y += 40
    img.save(path)


# --- честная сводка по целям из route.log
rows = []
for line in (run / "route.log").read_text(encoding="utf-8").splitlines():
    m = re.search(
        r"GOAL_END (\w+) result=(\w+) .*dur_sim=([\d.]+) err=([\d.]+) em_episodes=(\d+) recoveries=(\d+)", line
    )
    if m:
        rows.append(m.groups())
summary = []
for n, r, d, e, s, rc in rows:
    if r != "STATUS_SUCCEEDED":
        verdict = f"НЕ дошёл ({r})"
    elif float(e) > 1.0:
        verdict = f"ПРОМАХ: по карте дошёл, на деле в {float(e):.1f} м от цели"
    else:
        verdict = f"дошёл, ошибка {float(e):.2f} м"
    summary.append(f"{n}: {verdict}; {float(d):.0f} с sim, стопов {s}, восст. {rc}")
card(
    out / "intro.png",
    "РОББОКС в Gazebo: картография с нуля",
    [
        "Стек настоящего робота (Main Pi: Nav2, rtabmap, защита) едет по данным симулятора, HIL.",
        "Мир: Depot, Open Robotics, Fuel, CC-BY 4.0. Карта строится rtabmap с пустой базы.",
        "Вид сверху: красное — занятые клетки карты, голубое — свободные, зелёная точка — истинная поза.",
        "Справа внизу: 3D-камера Gazebo. Все кадры — из прогона, ничего не дорисовано.",
    ],
    (60, 180, 255),
)
card(
    out / "outro.png",
    "Итог прогона: что дошло и что нет",
    summary[:6]
    + [
        "",
        "Маршрут по готовой карте не получился: ICP-одометрия ушла (ошибка позы до 12 м).",
        "В ролик вошла только картография (цели M1-M5), пока оценка позы держалась.",
    ],
    (255, 170, 60),
)

main = out / "main"
for f in main.glob("*.png"):
    f.unlink()
sh("python3", here / "compose.py", run, 5, 20, t0, t1, main)
sh(
    "ffmpeg",
    "-y",
    "-loglevel",
    "error",
    "-loop",
    "1",
    "-framerate",
    20,
    "-t",
    4,
    "-i",
    out / "intro.png",
    "-vf",
    "fade=in:0:10,fade=out:70:10,format=yuv420p",
    "-c:v",
    "libx264",
    "-crf",
    20,
    out / "a_intro.mp4",
)
sh(
    "ffmpeg",
    "-y",
    "-loglevel",
    "error",
    "-framerate",
    20,
    "-i",
    main / "f%05d.png",
    "-vf",
    "format=yuv420p",
    "-c:v",
    "libx264",
    "-preset",
    "slow",
    "-crf",
    22,
    out / "b_main.mp4",
)
sh(
    "ffmpeg",
    "-y",
    "-loglevel",
    "error",
    "-loop",
    "1",
    "-framerate",
    20,
    "-t",
    9,
    "-i",
    out / "outro.png",
    "-vf",
    "fade=in:0:10,fade=out:170:10,format=yuv420p",
    "-c:v",
    "libx264",
    "-crf",
    20,
    out / "c_outro.mp4",
)
(out / "list.txt").write_text("".join(f"file '{out}/{n}.mp4'\n" for n in ("a_intro", "b_main", "c_outro")))
sh(
    "ffmpeg",
    "-y",
    "-loglevel",
    "error",
    "-f",
    "concat",
    "-safe",
    0,
    "-i",
    out / "list.txt",
    "-c",
    "copy",
    "-movflags",
    "+faststart",
    out / "hil_depot.mp4",
)

# --- GIF: быстрее (x8), 10 кадров/с, ширину и палитру вниз, пока не влезет
gdir = out / "gif"
gdir.mkdir(exist_ok=True)
for f in gdir.glob("*.png"):
    f.unlink()
sh("python3", here / "compose.py", run, 8, 10, t0, t1, gdir)
for width, fps, colors in ((640, 10, 96), (560, 8, 80), (480, 8, 64), (480, 6, 48)):
    sh(
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-framerate",
        10,
        "-i",
        gdir / "f%05d.png",
        "-vf",
        f"fps={fps},scale={width}:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors={colors}:stats_mode=diff[p];"
        f"[s1][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle",
        out / "hil_depot.gif",
    )
    size = (out / "hil_depot.gif").stat().st_size
    print(f"gif {width}px {fps}fps {colors}c: {size / 1024:.0f} КБ")
    if size <= gif_mb * 1024 * 1024:
        break
for n in ("hil_depot.mp4", "hil_depot.gif"):
    print(n, (out / n).stat().st_size)
