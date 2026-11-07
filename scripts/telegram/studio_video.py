"""Ролики и анимации студии: план из готовых шаблонов, материал через гейт, монтаж на builder.

Решение Шифу 06.11 (#4513): «видео оно должно собирать само и любое, которое мы обсудим». Порядок (ADR-0148):

  1. **Что можно** — считает код: какие шаблоны осуществимы с материалом в каталоге (``feasible``).
  2. **Что выбрать** — сначала правило кода (тема поста совпала с известной парой «до/после» → ``side_by_side``;
     возможен один шаблон → он). Только если вариантов несколько, а правило молчит, выбирает модель по узкой
     схеме ``{"template", "runs", "title"}``; ``validate_plan`` отбивает всё, чего нет в каталоге.
  3. **Монтаж** — готовыми инструментами репо: ``scripts/hil/video/pose_anim.py`` (правда против оценки),
     ``compose.py`` (вид сверху + камера), ``infographic.py`` (анимированная карточка и таймлайн).
  4. **Результат** — ``editMessageMedia`` того же черновика: MP4 ≤ 50 МБ (кнопка/просьба «видео») или
     GIF ≤ 2 МБ («анимация»). Каждый шаг — строка индикатора; фразы строит код.

Шаблоны:
  * ``side_by_side`` — два прогона бок о бок в одном масштабе: где робот на самом деле, где думает, ошибка позы;
  * ``top_view`` — вид сверху с картой rtabmap и врезкой камеры (compose.py) по кадрам прогона;
  * ``animated_card`` — карточка «было → стало» с цифрами, которые код посчитал по позе прогонов;
  * ``timeline`` — таймлайн влитых PR сессии с поэлементным появлением.
"""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, Optional

import channel_lib as lib
import studio_material as material
import studio_media as smedia
import studio_render as render

REPO = Path(__file__).resolve().parents[2]
TEMPLATES = ("compare", "top_view", "animated_card", "timeline")
# Семейство «сравнение двух прогонов»: раскладка и ориентация — параметры,
# а не отдельные шаблоны. Так студия сама собирает и «бок о бок», и
# «до сверху, после снизу», и то же самое в вертикальном кадре (Шифу, 07.11).
LAYOUTS = ("side_by_side", "stacked")
ORIENTATIONS = ("landscape", "portrait")
TEMPLATE_HELP = {
    "compare": "два прогона: рядом (side_by_side) или стопкой (stacked — «до» сверху, «после» снизу); "
    "кадр горизонтальный (landscape) или вертикальный (portrait)",
    "top_view": "вид сверху на карту rtabmap с врезкой 3D-камеры (кадры прогона)",
    "animated_card": "карточка «было → стало» с цифрами ошибки позы, посчитанными по прогонам",
    "timeline": "таймлайн влитых PR сессии, строки появляются по очереди",
}
COMPARE_MARKERS = ("до/после", "до и после", "до-после", "бок о бок", "было/стало", "было и стало", "сравн")
TOP_MARKERS = ("сверху", "камер", "карт")
# Просьба Шифу (07.11): «видео вертикальным, чтобы до было сверху, а после снизу».
# «сверху» отдельно — это вид сверху (top_view); стопка — только когда есть и «снизу».
VERTICAL_MARKERS = ("вертикаль", "вертикальн", "столбик", "стопкой", "в столбик", "сверху и снизу")
STACK_MARKERS = ("стопкой", "столбиком", "в столбик", "сверху и снизу")
COMPARE_RE = re.compile(r"до\s+и\s+после|до[^\w]{0,3}после|после[^\w]{0,3}до|до\s+сверху|до\s+был[оа]")
TITLE_MAX = 90
MP4_SIZE, MP4_FPS, MP4_SECONDS, HOLD_SECONDS = (1280, 720), 20, 30, 3
GIF_SIZE, GIF_FPS, GIF_SECONDS = (640, 360), 10, 14
MP4_VERTICAL_SIZE, GIF_VERTICAL_SIZE = (720, 1280), (360, 640)
TOP_MAX_FRAMES = 240  # кадров вида сверху: каждый кадр — два scp (вид сверху и камера)


# --------------------------------------------------------------------------- план


def feasible(record: dict, facts: list, catalog: list) -> list:
    """Шаблоны, которые можно собрать из того, что есть. Решает код."""
    poses = [item for item in catalog if item.get("kind") == "poses"]
    out = []
    if len(poses) >= 2:
        out.append("compare")
    if any(item.get("kind") == "frames" for item in catalog):
        out.append("top_view")
    if poses:
        out.append("animated_card")
    if smedia.timeline_spec(record, facts) is not None:
        out.append("timeline")
    return out


def _context(record: dict, request: str) -> str:
    return " ".join(str(record.get(k) or "") for k in ("title", "show", "body_html")) + " " + (request or "")


def plan_by_code(record: dict, request: str, catalog: list, options: list) -> Optional[dict]:
    """Правило кода: тема поста + известный материал. None — правило не решает, нужен выбор модели."""
    text = _context(record, request)
    topic = material.topic_of(text)
    low = (request or "").lower()
    # Раскладка и ориентация — по словам Шифу. «Сверху» само по себе — это вид
    # сверху (top_view); стопка — только если есть и «снизу» (Шифу, 07.11).
    stacked = any(m in low for m in STACK_MARKERS) or ("сверху" in low and "снизу" in low)
    portrait = any(m in low for m in VERTICAL_MARKERS)
    layout = "stacked" if stacked else "side_by_side"
    orientation = "portrait" if portrait else "landscape"
    wants_compare = any(m in low for m in COMPARE_MARKERS) or bool(COMPARE_RE.search(low))
    if "compare" in options and (wants_compare or stacked or portrait):
        pair = material.pair_for(topic, catalog) if topic else []
        runs = pair or _default_runs("compare", catalog)
        if len(runs) >= 2:
            title = material.TOPICS[topic]["title"] if topic and topic in material.TOPICS else (record.get("title") or "РОББОКС")
            return {"template": "compare", "runs": runs, "layout": layout, "orientation": orientation, "title": title}
    if (
        "top_view" in options
        and any(m in low for m in TOP_MARKERS)
        and not wants_compare
        and not stacked
        and not portrait
    ):
        return {"template": "top_view", "runs": [material.TOP_RUN], "title": "РОББОКС в Gazebo: склад Depot"}
    pair = material.pair_for(topic, catalog) if topic else []
    if pair and "compare" in options:
        return {
            "template": "compare",
            "runs": pair,
            "layout": layout,
            "orientation": orientation,
            "title": material.TOPICS[topic]["title"],
        }
    if len(options) == 1:
        only = options[0]
        runs = _default_runs(only, catalog)
        return {
            "template": only,
            "runs": runs,
            "layout": layout,
            "orientation": orientation,
            "title": record.get("title") or "РОББОКС",
        }
    return None


def _default_runs(template: str, catalog: list) -> list:
    if template == "top_view":
        return [material.TOP_RUN]
    if template in ("compare", "animated_card"):
        return [item["id"] for item in catalog if item.get("kind") == "poses"][:2]
    return []


def normalize_plan(plan: dict) -> dict:
    """Единый вид плана: шаблон, прогоны, раскладка, ориентация, заголовок.

    Старое имя шаблона «side_by_side» — это «compare» с боковой раскладкой
    в горизонтальном кадре (планы из состояния и от модели).
    """
    out = dict(plan or {})
    template = str(out.get("template") or "")
    if template == "side_by_side":
        out["template"] = "compare"
    elif template == "compare":
        out["template"] = "compare"
    out.setdefault("layout", "side_by_side")
    out.setdefault("orientation", "landscape")
    if out.get("layout") not in LAYOUTS:
        out["layout"] = "side_by_side"
    if out.get("orientation") not in ORIENTATIONS:
        out["orientation"] = "landscape"
    return out


def validate_plan(plan: dict, options: list, catalog: list) -> list:
    """Проблемы плана модели: шаблон не из возможных, прогоны не из каталога, заголовок с мусором."""
    problems = []
    if not isinstance(plan, dict):
        return ["план не JSON-объект"]
    plan = normalize_plan(plan)
    template = plan.get("template")
    if template == "none":
        return []
    if template not in options:
        problems.append(f"шаблон {template!r} не из возможных {options}")
    kinds = {item["id"]: item.get("kind") for item in catalog}
    runs = plan.get("runs") or []
    if not isinstance(runs, list) or any(r not in kinds for r in runs):
        problems.append(f"прогоны {runs!r} не из каталога")
    elif template == "compare" and (
        len(runs) != 2 or len(set(runs)) != 2 or {kinds[r] for r in runs} != {"poses"}
    ):
        problems.append("compare: нужно ровно два разных прогона с позой")
    elif template == "animated_card" and not (1 <= len(runs) <= 2 and all(kinds[r] == "poses" for r in runs)):
        problems.append("animated_card: нужен один или два прогона с позой")
    elif template == "top_view" and [kinds.get(r) for r in runs] != ["frames"]:
        problems.append("top_view: нужен один прогон с кадрами камер")
    if template == "compare":
        if plan.get("layout") not in LAYOUTS:
            problems.append(f"раскладка {plan.get('layout')!r} не из {LAYOUTS}")
        if plan.get("orientation") not in ORIENTATIONS:
            problems.append(f"ориентация {plan.get('orientation')!r} не из {ORIENTATIONS}")
    title = plan.get("title")
    if not isinstance(title, str) or not title.strip() or len(title) > TITLE_MAX or "<" in title:
        problems.append(f"заголовок пустой, длиннее {TITLE_MAX} знаков или с разметкой")
    elif any(pattern.search(title) for pattern, _what in lib.FORBIDDEN):
        problems.append("в заголовке запрещённое (адрес, ключ)")
    return problems


def plan_prompt(record: dict, request: str, options: list, catalog: list, fmt: str) -> str:
    lines = [
        "Ты — студия канала РОББОКС. Выбери шаблон ролика к черновику. Только из списка, ничего не выдумывай.",
        f"Формат результата: {'видео MP4' if fmt == 'mp4' else 'анимация GIF'}.",
        "Возможные шаблоны:",
        *[f"- {name}: {TEMPLATE_HELP[name]}" for name in options],
        "- none: ни один шаблон не подходит к теме поста",
        "Раскладка (layout) для compare: side_by_side — рядом, stacked — стопкой, «до» сверху, «после» снизу.",
        "Ориентация (orientation) для compare: landscape — горизонтальный кадр, portrait — вертикальный.",
        "Прогоны (id — подпись — что внутри):",
        *[f"- {item['id']} — {item['label']} — {item['kind']}" for item in catalog],
        f"Черновик: {re.sub(r'<[^>]+>', '', record.get('body_html') or '')[:1200]}",
        f"Просьба Шифу: {request or 'не сказано'}",
        'Верни СТРОГО JSON: {"template": "...", "runs": ["id", ...], "layout": "...", "orientation": "...", '
        '"title": "заголовок ролика до 90 знаков"}. Для шаблонов кроме compare layout и orientation не нужны.',
    ]
    return "\n".join(lines)


def choose_plan(record: dict, request: str, facts: list, catalog: list, fmt: str, ask: Optional[Callable] = None):
    """(план или None, почему так). Код решает, модель выбирает только между возможными."""
    options = feasible(record, facts, catalog)
    if not options:
        return None, "материала нет: ни прогонов на песочнице, ни влитых PR для таймлайна"
    planned = plan_by_code(record, request, catalog, options)
    if planned is not None:
        return planned, f"выбрал код: {TEMPLATE_HELP[planned['template']]}"
    if ask is None:
        import studio_text

        ask = studio_text.call_minimax
    answer = ask(plan_prompt(record, request, options, catalog, fmt), timeout=120)
    if not isinstance(answer, dict) or answer.get("error"):
        return None, f"модель не выбрала шаблон: {(answer or {}).get('error') if isinstance(answer, dict) else answer}"
    problems = validate_plan(answer, options, catalog)
    if problems:
        return None, "план модели не прошёл проверку: " + "; ".join(problems)
    if answer.get("template") == "none":
        return None, "модель сказала: ни один шаблон не подходит к теме поста"
    plan = normalize_plan(answer)
    why = f"выбрала модель из {options}, план проверен кодом"
    return {k: plan[k] for k in ("template", "runs", "layout", "orientation", "title")}, why


# --------------------------------------------------------------------------- монтаж


class Context:
    """Всё, что нужно сборке: гейт, рабочий каталог, факты, черновик, индикатор."""

    def __init__(
        self, gate, work: Path, facts: list, record: dict, progress, popen=subprocess.Popen, run=subprocess.run
    ):
        self.gate, self.work, self.facts, self.record, self.progress = gate, Path(work), facts, record, progress
        self.popen, self.run = popen, run
        self.build_seconds = 0.0

    @property
    def cache(self) -> Path:
        return self.work / "material"


def _load_run(ctx: Context, run_id: str, label: str):
    import pose_anim

    ctx.progress.step(f"Беру прогон {run_id} с песочницы (гейт только на чтение, scp -O)", "upload_video")
    calls = ctx.gate.calls
    got, reason = material.fetch_pose_run(ctx.gate, run_id, ctx.cache)
    if got is None:
        return None, reason
    rows = pose_anim.load_poses_csv(got["poses"])
    if not rows:
        return None, f"в {run_id}/maprec/poses.csv нет строк позы"
    maps, note = material.load_maps(got["maps"])
    stats = pose_anim.summary(rows)
    worst = pose_anim.meters(stats.get("max_error"))
    ctx.progress.note(
        f"Прогон {run_id}: {len(rows)} строк позы, {len(maps)} снимков карты, худшая ошибка {worst} "
        f"(вызовов гейта {ctx.gate.calls - calls}){'; ' + note if note else ''}"
    )
    return pose_anim.Run(rows, label, maps), ""


def _labels(catalog: list) -> dict:
    return {item["id"]: item["label"] for item in catalog}


def build_compare(plan: dict, fmt: str, ctx: Context, catalog: list) -> tuple:
    """Сравнение двух прогонов: рядом или стопкой, горизонт или вертикаль.

    Раскладку и ориентацию выбирает код (по словам Шифу) или модель — студия
    собирает и «бок о бок», и «до сверху, после снизу» в вертикальном кадре
    без нового шаблона (Шифу, 07.11).
    """
    import pose_anim

    plan = normalize_plan(plan)
    runs = []
    for run_id in plan["runs"]:
        run, reason = _load_run(ctx, run_id, _labels(catalog).get(run_id, run_id))
        if run is None:
            return None, reason
        runs.append(run)
    portrait = plan["orientation"] == "portrait"
    if fmt == "mp4":
        size = MP4_VERTICAL_SIZE if portrait else MP4_SIZE
        fps, seconds = MP4_FPS, MP4_SECONDS
    else:
        size = GIF_VERTICAL_SIZE if portrait else GIF_SIZE
        fps, seconds = GIF_FPS, GIF_SECONDS
    hold = HOLD_SECONDS * fps
    frames = seconds * fps + hold
    stacked = plan["layout"] == "stacked"

    def make():
        if stacked:
            return pose_anim.stacked_frames(runs, plan["title"], frames, size=size, hold=hold, font_hint=size[0])
        return pose_anim.side_by_side_frames(runs, plan["title"], frames, size=size, hold=hold)

    stem = f"{ctx.record['id']}-{plan['layout']}-{plan['orientation']}"
    music = plan.get("music")
    if music and not Path(str(music)).exists():
        music = None
    if music:
        ctx.progress.note(f"Накладываю трек: {Path(str(music)).name}")
    return _encode(make, fmt, size, fps, ctx, stem, music=music)


def _encode(make: Callable, fmt: str, size: tuple, fps: int, ctx: Context, stem: str, music=None) -> tuple:
    out = ctx.work / f"{stem}.{fmt}"
    ctx.progress.step(f"Монтирую {'MP4' if fmt == 'mp4' else 'GIF'} на builder", "record_video")
    if fmt == "mp4":
        path, reason, note = render.fit_mp4(make, size, fps, out, popen=ctx.popen, music=music)
    else:
        path, reason, note = render.fit_gif(make, out, fps)
    if path is None:
        return None, reason
    ctx.progress.note(f"Смонтировал: {note}")
    return path, ""


def card_spec(plan: dict, runs: list) -> dict:
    """Карточка «было → стало» из цифр прогонов (код считает, модель только назвала заголовок)."""
    import pose_anim

    def pick(key):
        return [pose_anim.meters(r.stats.get(key)) for r in runs]

    source = "HIL, maprec/poses.csv прогонов " + ", ".join(r.label for r in runs)
    worst, final = pick("max_error"), pick("final_error")
    pairs = [
        {"label": "Худшая ошибка позы", "before": worst[0], "after": worst[-1], "source": source},
        {"label": "Ошибка в конце прогона", "before": final[0], "after": final[-1], "source": source},
        {
            "label": "Длительность прогона",
            "before": f"{runs[0].stats['duration']:.0f} с",
            "after": f"{runs[-1].stats['duration']:.0f} с",
            "source": source,
        },
    ]
    return {
        "template": "before_after",
        "title": plan["title"][:60],
        "subtitle": "ошибка позы: оценка rtabmap против правды Gazebo",
        "pairs": pairs,
    }


def build_card(plan: dict, fmt: str, ctx: Context, catalog: list) -> tuple:
    runs = []
    for run_id in plan["runs"]:
        run, reason = _load_run(ctx, run_id, _labels(catalog).get(run_id, run_id))
        if run is None:
            return None, reason
        runs.append(run)
    return _infographic(card_spec(plan, runs), fmt, ctx, f"{ctx.record['id']}-card")


def build_timeline(plan: dict, fmt: str, ctx: Context, catalog: list) -> tuple:
    spec = smedia.timeline_spec(ctx.record, ctx.facts)
    if spec is None:
        return None, "влитых PR в фактах меньше двух — таймлайн не из чего собрать"
    spec.pop("background", None)
    return _infographic(spec, fmt, ctx, f"{ctx.record['id']}-timeline")


def _infographic(spec: dict, fmt: str, ctx: Context, stem: str) -> tuple:
    ctx.progress.step("Рисую анимированную инфографику (infographic.py, GIF)", "upload_video")
    gif = ctx.work / f"{stem}.gif"
    try:
        import infographic

        infographic.render_gif(spec, gif, "wide")
    except (Exception, SystemExit) as exc:  # noqa: BLE001 — нет шрифта или Pillow: причина в индикатор
        return None, f"инфографика не нарисовалась: {exc}"
    size = gif.stat().st_size
    if fmt == "gif":
        if size > render.GIF_LIMIT:
            return None, f"GIF вышел {render.mb(size)} — больше лимита {render.mb(render.GIF_LIMIT)}"
        ctx.progress.note(f"Нарисовал GIF: {render.mb(size)}")
        return gif, ""
    out = ctx.work / f"{stem}.mp4"
    ctx.progress.step("Перегоняю GIF в MP4 (ffmpeg)", "record_video")
    reason = render.gif_to_mp4(gif, out, run=ctx.run)
    if reason:
        return None, reason
    return out, ""


# --------------------------------------------------------------------------- вид сверху (compose.py)


def frame_fit(route_text: str) -> Optional[tuple]:
    """Номер кадра по sim-времени: frame = a·t + b (МНК по GOAL_START/GOAL_END, как compose.py)."""
    pairs = [
        (float(m.group(1)), int(m.group(2)))
        for m in re.finditer(r"GOAL_(?:START|END) \S+ .*?sim=([\d.]+) frame=(-?\d+)", route_text)
    ]
    if len(pairs) < 2:
        return None
    n = len(pairs)
    mt = sum(p[0] for p in pairs) / n
    mf = sum(p[1] for p in pairs) / n
    var = sum((p[0] - mt) ** 2 for p in pairs)
    if var == 0:
        return None
    a = sum((p[0] - mt) * (p[1] - mf) for p in pairs) / var
    return a, mf - a * mt


def mapping_window(route_text: str) -> Optional[tuple]:
    """Отрезок картографии по route.log: от первого GOAL_START до последнего GOAL_END целей M*."""
    starts = [float(x) for x in re.findall(r"GOAL_START M\w* .*?sim=([\d.]+)", route_text)]
    ends = [float(x) for x in re.findall(r"GOAL_END M\w* .*?sim=([\d.]+)", route_text)]
    if not starts or not ends:
        return None
    return min(starts), max(ends)


def top_view_frames(route_text: str, fps: int = MP4_FPS, limit: int = TOP_MAX_FRAMES) -> Optional[dict]:
    """Какие кадры камер нужны compose.py: {"t0", "t1", "speed", "frames": [номер, …]}. None — нет данных."""
    fit, window = frame_fit(route_text), mapping_window(route_text)
    if fit is None or window is None:
        return None
    t0, t1 = window
    speed = max(5.0, (t1 - t0) * fps / limit)
    count = int((t1 - t0) / (speed / fps))
    frames = sorted({int(round(fit[0] * (t0 + k * speed / fps) + fit[1])) for k in range(count)})
    return {"t0": t0, "t1": t1, "speed": speed, "frames": frames}


def build_top_view(plan: dict, fmt: str, ctx: Context, catalog: list) -> tuple:
    base, local = material.HIL_OUT, ctx.cache / material.TOP_RUN
    ctx.progress.step("Беру лог маршрута и позу прогона с песочницы (scp -O)", "upload_video")
    for rel in ("route.log", "maprec/poses.csv"):
        reason = ctx.gate.fetch(f"{base}/{rel}", local / rel)
        if reason:
            return None, reason
    need = top_view_frames((local / "route.log").read_text(encoding="utf-8", errors="replace"))
    if need is None:
        return None, "в route.log нет целей с кадрами — не понять, какие кадры брать"
    got = _fetch_top_frames(ctx, base, local, need["frames"])
    if got:
        return None, got
    frames_dir = ctx.work / f"{ctx.record['id']}-top"
    ctx.progress.step("Монтирую вид сверху (compose.py)", "record_video")
    speed, t0, t1 = need["speed"], need["t0"], need["t1"]
    cmd = [sys.executable, str(REPO / "scripts/hil/video/compose.py"), str(local), str(speed), str(MP4_FPS)]
    done = ctx.run(cmd + [str(t0), str(t1), str(frames_dir)], capture_output=True, text=True, timeout=1800, check=False)
    if done.returncode != 0:
        return None, f"compose.py упал: {(done.stderr or '').strip()[-200:]}"
    out = ctx.work / f"{ctx.record['id']}-top.mp4"
    reason = render.images_to_mp4(frames_dir, MP4_FPS, out, run=ctx.run)
    if reason:
        return None, reason
    if fmt == "gif":
        return _mp4_frames_to_gif(frames_dir, ctx)
    return out, ""


def _fetch_top_frames(ctx: Context, base: str, local: Path, numbers: list) -> str:
    wanted = set(numbers)
    for cam in ("video_top", "video_chase"):
        names, reason = ctx.gate.ls(f"{base}/frames/{cam}")
        if reason:
            return reason
        picked = [n for n in names if _frame_no(n) in wanted]
        ctx.progress.note(f"Кадры {cam}: беру {len(picked)} из {len(names)}")
        for name in picked:
            reason = ctx.gate.fetch(f"{base}/frames/{cam}/{name}", local / "frames" / cam / name)
            if reason:
                return reason
    return ""


def _frame_no(name: str) -> Optional[int]:
    match = re.search(r"_(-?\d+)\.png$", name)
    return int(match.group(1)) if match else None


def _mp4_frames_to_gif(frames_dir: Path, ctx: Context) -> tuple:
    from PIL import Image

    paths = sorted(Path(frames_dir).glob("*.png"))

    def make():
        return (Image.open(p) for p in paths)

    return _encode(make, "gif", GIF_SIZE, GIF_FPS, ctx, f"{ctx.record['id']}-top")


BUILDERS = {
    "compare": build_compare,
    "side_by_side": build_compare,  # старое имя шаблона
    "top_view": build_top_view,
    "animated_card": build_card,
    "timeline": build_timeline,
}


def build(plan: dict, fmt: str, ctx: Context, catalog: list) -> tuple:
    """План → файл. (путь или None, причина)."""
    ctx.work.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    path, reason = BUILDERS[plan["template"]](plan, fmt, ctx, catalog)
    if path is None:
        return None, reason
    if fmt == "mp4":
        # Сам проверяю результат до показа (идея из claude-video-editor: «отрендерил → проверил → показал»).
        note, reason = render.check_mp4(path, run=ctx.run)
        if reason:
            return None, reason
        ctx.progress.note(f"Смонтировал и {note}")
    limit = render.MP4_LIMIT if fmt == "mp4" else render.GIF_LIMIT
    size = path.stat().st_size
    if size > limit:
        return None, f"файл {render.mb(size)} больше лимита Telegram для студии {render.mb(limit)}"
    ctx.build_seconds = round(time.monotonic() - started, 1)
    return path, ""


def describe(plan: dict) -> str:
    return json.dumps({k: plan.get(k) for k in ("template", "runs")}, ensure_ascii=False)


def fingerprint(plan: dict, fmt: str, catalog: list) -> str:
    """Отпечаток входов сборки: тот же план и тот же материал → тот же ролик.

    Нужен, чтобы не пересобирать ролик заново на каждое нажатие «Видео»:
    сборка занимает ~50 с, а материал и план между нажатиями не меняются
    (жалоба Шифу, 07.11). Смена шаблона, раскладки, ориентации, прогона,
    формата или материала меняет отпечаток — тогда ролик соберётся заново.
    """
    plan = normalize_plan(plan)
    runs = [str(r) for r in (plan.get("runs") or [])]
    kinds = {str(i.get("id")): str(i.get("kind") or "") for i in catalog or []}
    payload = json.dumps(
        {
            "template": str(plan.get("template") or ""),
            "layout": str(plan.get("layout") or ""),
            "orientation": str(plan.get("orientation") or ""),
            "runs": runs,
            "fmt": str(fmt),
            "material": [[run, kinds.get(run, "")] for run in runs],
            "music": str(plan.get("music") or ""),
        },
        ensure_ascii=False,
        sort_keys=True,
    )
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()
