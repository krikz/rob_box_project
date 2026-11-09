"""Медиа черновика студии: что показать к посту и как это отправить в Telegram.

Решение Шифу (06.11): черновик приходит сразу с графикой. Источников два, оба —
из репо или из фактов сессии, ничего не выдумывается:

  1. готовое медиа: факты вида ``media`` (новые файлы в develop) и ``media_ref`` поста;
  2. инфографика ``scripts/telegram/infographic.py`` (шаблон ``timeline``) из влитых PR
     сессии. У каждой строки источник — PR, он печатается в подвале картинки.
     Фон — уже лежащая в репо подложка ``channel_bg`` (docs/channel/media/bg/*.jpg) по теме
     поста: API image-01 студия не зовёт, генерация фона — только через робота (channel_bg).

Кнопка «Медиа» перебирает варианты по кругу (``next_option``).
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Optional

import channel_lib as lib

REPO = Path(__file__).resolve().parents[2]

# Метод Bot API и тип InputMedia по расширению файла (Bot API 10.3: sendPhoto,
# sendAnimation, sendVideo; InputMediaPhoto/Animation/Video для editMessageMedia).
KINDS = {}
for _ext in lib.PHOTO_EXT:
    KINDS[_ext] = ("sendPhoto", "photo", "upload_photo")
for _ext in lib.ANIMATION_EXT:
    KINDS[_ext] = ("sendAnimation", "animation", "upload_video")
for _ext in lib.VIDEO_EXT:
    KINDS[_ext] = ("sendVideo", "video", "upload_video")

# Подложка по теме поста: только готовые файлы из docs/channel/media/bg (кэш channel_bg).
BG_BY_TAG = {
    "#навигация": "bg-doorway",
    "#симуляция": "bg-doorway",
    "#светодиоды": "bg-led",
    "#тесты": "bg-night",
    "#честность": "bg-night",
    "#железо": "bg-nodes",
    "#raspberrypi": "bg-nodes",
    "#ros2": "bg-nodes",
}
BG_DEFAULT = "bg-code"

TIMELINE_MAX = 5  # шаблон timeline: 2–6 событий; на 1280×720 до подвала влезает 5
TEXT_MAX = 60  # знаков в строке события: дальше текст уходит за край панели
PR_CLAIM_RE = re.compile(r"^PR #(\d+) влит в develop:\s*(.+)$")
CONVENTIONAL_RE = re.compile(r"^[a-z]+(?:\([^)]*\))?!?:\s*")


def kind_of(path: Path) -> Optional[tuple]:
    """(метод отправки, тип InputMedia, sendChatAction) или None, если тип не поддержан."""
    return KINDS.get(Path(path).suffix.lower())


def resolve(ref: str, repo_root: Path = REPO) -> Optional[Path]:
    """Файл по ссылке поста: абсолютный путь, клон или worktree develop. None — файла нет."""
    if not ref:
        return None
    path = Path(ref)
    if path.is_absolute():
        return path if path.is_file() else None
    found = lib.resolve_media(repo_root, ref)
    return found if found.is_file() else None


ISSUE_REF_RE = re.compile(r"\s*\(#\d+\)\s*$")


def _short(text: str) -> str:
    text = ISSUE_REF_RE.sub("", " ".join(text.split()))
    if len(text) <= TEXT_MAX:
        return text
    cut = text[: TEXT_MAX - 1]
    if " " in cut:
        cut = cut.rsplit(" ", 1)[0]
    return cut.rstrip(" ,.;:—-") + "…"


def _focus_order(facts: list, focus_keys: list) -> list:
    """Сначала факты поста, потом PR той же карточки (#NNNN), потом остальные."""
    own = [f for f in facts if f.get("key") in set(focus_keys or [])]
    refs = {m for f in own for m in re.findall(r"\(#(\d+)\)", f.get("claim") or "")}
    related = [f for f in facts if f not in own and refs & set(re.findall(r"\(#(\d+)\)", f.get("claim") or ""))]
    return own + related + [f for f in facts if f not in own and f not in related]


def reader_title(claim: str, limit: int = 90) -> str:
    """Заголовок для читателя из факта: без «PR #N влит в develop:» и типа коммита.

    «PR #4507 влит в develop: fix(rtabmap): один источник параметров — …» →
    «Один источник параметров — …». Такой заголовок может попасть на ролик, а
    внутренняя кухня проекта в канал не идёт (Шифу, 07.11).
    """
    text = (claim or "").strip()
    match = PR_CLAIM_RE.match(text)
    if match:
        text = CONVENTIONAL_RE.sub("", match.group(2)).strip()
    text = text[:limit].rstrip(" .,:;")
    return text[:1].upper() + text[1:]


def timeline_events(facts: list, limit: int = TIMELINE_MAX, focus_keys: Optional[list] = None) -> list:
    """События таймлайна — из фактов «PR влит в develop». Дата и источник берутся из факта."""
    events = []
    for fact in _focus_order(facts, focus_keys or []):
        if fact.get("kind") != "merged_pr":
            continue
        match = PR_CLAIM_RE.match((fact.get("claim") or "").strip())
        merged = fact.get("merged_at") or ""
        if not match or len(merged) < 10:
            continue
        number, title = match.groups()
        text = CONVENTIONAL_RE.sub("", title)
        events.append({"date": f"{merged[8:10]}.{merged[5:7]}", "text": _short(text), "source": f"PR #{number}"})
        if len(events) >= limit:
            break
    return events


def timeline_spec(post: dict, facts: list) -> Optional[dict]:
    """Данные для infographic.py (timeline) или None, если событий меньше двух."""
    events = timeline_events(facts, focus_keys=post.get("fact_keys") or [])
    if len(events) < 2:
        return None
    tags = post.get("tags") or lib.extract_tags(post.get("body_html") or "")
    slug = next((BG_BY_TAG[t] for t in tags if t in BG_BY_TAG), BG_DEFAULT)
    return {
        "template": "timeline",
        "title": "Что влито в develop",
        "subtitle": "по влитым PR — источник каждой строки в подвале",
        "events": events,
        "background": {"slug": slug, "prompt": "cached studio background"},
    }


def _cached_background(cfg: dict):
    """Подложка только из кэша репо: студия не генерирует фон сама (ключ image-01 — у робота)."""
    import channel_bg

    def _no_remote(_request: dict) -> dict:
        return {"ok": False, "description": "фона нет в кэше, а генерировать студия не будет"}

    return channel_bg.load_background(cfg, remote=_no_remote)


def render_timeline(post: dict, facts: list, out_dir: Path) -> tuple:
    """Нарисовать инфографику к посту. Возвращает (путь или None, причина при неудаче)."""
    spec = timeline_spec(post, facts)
    if spec is None:
        return None, "влитых PR в фактах меньше двух — таймлайн не из чего рисовать"
    try:
        import infographic
    except ImportError as exc:  # нет Pillow — честно говорим, что именно сломалось
        return None, f"инфографика недоступна: {exc}"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"{post.get('id') or 'post'}-timeline.png"
    # make_channel_media.font() без шрифта делает SystemExit — это не повод ронять
    # работу студии: ловим и отдаём причину строкой в индикатор.
    try:
        try:
            image = infographic.render(spec, "wide", bg_loader=_cached_background)
        except Exception:  # noqa: BLE001 — фон не обязателен: без него рисуем на панели
            spec.pop("background", None)
            image = infographic.render(spec, "wide")
        image.save(out, optimize=True)
    except (Exception, SystemExit) as exc:  # noqa: BLE001
        return None, f"инфографика не нарисовалась: {exc}"
    return out, ""


PR_NUMBER_RE = re.compile(r"#(\d+)")


def fact_prs(fact: dict) -> set:
    """Все номера PR, которые называет факт: из текста, источника и поля ``pr``."""
    text = " ".join(str(fact.get(k) or "") for k in ("claim", "source"))
    found = set(PR_NUMBER_RE.findall(text))
    own = str(fact.get("pr") or "").strip().lstrip("#")
    if own:
        found.add(own)
    return found


def own_media(post: dict, facts: list) -> list:
    """Медиа-факты, которые принадлежат посту: его собственные или из того же PR.

    Единственное место этого правила. Разбор 09.11: пост про диалог (#4555) получил
    график дрейфа из другой правки (#4508): черновик сессии брал любую картинку
    сессии, а правило «из того же PR» жило только в ``studio_draft.pick_media``.
    """
    keys = set(post.get("fact_keys") or [])
    wanted = {key.split(":", 1)[1] for key in keys if key.startswith("pr:")}
    for fact in facts:
        if fact.get("key") in keys:
            wanted |= fact_prs(fact)
    wanted.discard("")
    return [
        fact
        for fact in facts
        if fact.get("kind") == "media" and fact.get("source") and (fact.get("key") in keys or wanted & fact_prs(fact))
    ]


def options(post: dict, facts: list, repo_root: Path = REPO) -> list:
    """Варианты медиа по порядку: готовое медиа поста, его медиа из фактов, затем инфографика.

    Медиа из фактов — только своё (``own_media``): чужая картинка к посту не подходит.
    Элемент — строка-ссылка на файл или ``"infographic:timeline"`` (рисуется по запросу).
    """
    found = []
    for ref in [post.get("media_ref") or ""] + [f["source"] for f in own_media(post, facts)]:
        if ref and ref not in found and resolve(ref, repo_root) is not None and kind_of(Path(ref)):
            found.append(ref)
    if timeline_spec(post, facts) is not None:
        found.append("infographic:timeline")
    return found


def next_option(post: dict, facts: list, repo_root: Path = REPO) -> Optional[str]:
    """Следующий вариант после текущего (по кругу). None — вариантов, кроме текущего, нет."""
    variants = options(post, facts, repo_root)
    current = post.get("media_choice") or post.get("media_ref") or ""
    others = [v for v in variants if v != current]
    if not others:
        return None
    if current in variants:
        index = variants.index(current)
        return variants[(index + 1) % len(variants)]
    return others[0]


def materialize(choice: str, post: dict, facts: list, out_dir: Path, repo_root: Path = REPO) -> tuple:
    """Вариант → файл на диске. (путь или None, причина)."""
    if choice == "infographic:timeline":
        return render_timeline(post, facts, out_dir)
    path = resolve(choice, repo_root)
    if path is None:
        return None, f"файла {choice} нет ни в клоне, ни в worktree develop"
    problems = lib.media_problems(path, choice)
    if problems:
        return None, "; ".join(problems)
    return path, ""
