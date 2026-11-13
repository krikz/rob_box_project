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


# Подвал картинки виден в канале: номеров PR там нет (решение Шифу 07.11). Номер PR остаётся в поле
# ``pr`` события — для сверки и журнала, infographic.py его не рисует.
TIMELINE_SOURCE = "журнал изменений РОББОКС"
TIMELINE_TITLE = "Что изменилось у РОББОКСа"
TIMELINE_SUBTITLE = "по датам: что поменялось в роботе"

# Подпись прогона робота. Она же уходит в подвал картинки («Источник: …»),
# поэтому обязана быть публичным текстом (channel_lib.POST_FORBIDDEN).
ROBOT_RUN_SOURCE = "журнал прогонов РОББОКС"

# Названия вариантов инфографики — для строки прогресса в индикаторе.
# Раньше там было всегда «таймлайн влитых PR», даже когда рисовалась карточка цифр.
INFOGRAPHIC_LABELS = {
    "infographic:numbers": "цифры темы",
    "infographic:timeline": "таймлайн влитых PR",
}


def variant_label(choice: str) -> str:
    """Человеческое название варианта медиа — для строки прогресса."""
    return INFOGRAPHIC_LABELS.get(choice, "инфографика")


def is_public(text: str) -> bool:
    """Текст можно показать в канале: без приватного и без внутренней кухни (channel_lib.POST_FORBIDDEN)."""
    return not any(pattern.search(text or "") for pattern, _what in lib.POST_FORBIDDEN)


def timeline_events(facts: list, limit: int = TIMELINE_MAX, focus_keys: Optional[list] = None) -> list:
    """События таймлайна — из фактов «PR влит в develop». Дата берётся из факта, номер PR — в ``pr``.

    Строка с кухней («один источник параметров — patch_rtabmap_launch.py») на картинку не идёт.
    """
    events = []
    for fact in _focus_order(facts, focus_keys or []):
        if fact.get("kind") != "merged_pr":
            continue
        match = PR_CLAIM_RE.match((fact.get("claim") or "").strip())
        merged = fact.get("merged_at") or ""
        if not match or len(merged) < 10:
            continue
        number, title = match.groups()
        text = _short(CONVENTIONAL_RE.sub("", title))
        if not is_public(text):
            continue
        text = text[:1].upper() + text[1:]
        events.append(
            {"date": f"{merged[8:10]}.{merged[5:7]}", "text": text, "source": TIMELINE_SOURCE, "pr": f"#{number}"}
        )
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
        "title": TIMELINE_TITLE,
        "subtitle": TIMELINE_SUBTITLE,
        "events": events,
        "background": {"slug": slug, "prompt": "cached studio background"},
    }


# Числа темы: какое поле факта на картинку и как оно называется для читателя.
# Порядок важен: первые четыре поля попадают на карточку (infographic.py: 2–4).
NUMBER_FIELDS = (
    ("goals", "цель маршрута", "цели маршрута", "целей маршрута"),
    ("sim_s", "секунда симуляции", "секунды симуляции", "секунд симуляции"),
    ("map_snapshots", "снимок карты", "снимка карты", "снимков карты"),
)
NUMBERS_MIN = 2  # infographic.py рисует шаблон numbers только с 2–4 числами

# Заголовки карточки цифр — по виду факта, читательским языком.
NUMBERS_TITLES = {
    "robot_run": ("Что показал робот на складе", "цифры прогона: маршрут, время, карта"),
}
NUMBERS_TITLE_DEFAULT = ("Цифры темы", "по данным факта")


def plural(number: int, one: str, few: str, many: str) -> str:
    """Русское склонение числительного: 1 цель, 2 цели, 5 целей."""
    value = abs(int(number))
    if value % 10 == 1 and value % 100 != 11:
        return one
    if 2 <= value % 10 <= 4 and not 12 <= value % 100 <= 14:
        return few
    return many


def chosen_fact(post: dict, facts: list) -> dict:
    """Факт, на котором построен пост: по ключам поста, а не «первый в списке»."""
    keys = set(post.get("fact_keys") or [])
    if not keys:
        return {}
    for fact in post.get("fact_dicts") or []:
        if fact.get("key") in keys:
            return fact
    for fact in facts or []:
        if fact.get("key") in keys:
            return fact
    return {}


def numbers_source(fact: dict) -> str:
    """Подпись источника под карточкой цифр: публичный текст (в подвале её видно)."""
    source = str(fact.get("source") or "")
    if source and is_public(source):
        return source
    return "журнал РОББОКС"


def numbers_spec(post: dict, facts: list) -> Optional[dict]:
    """Данные для infographic.py (numbers) из чисел ВЫБРАННОЙ темы.

    Разбор 13.11 (issue #4702): пост про голос получил картинку «последние 5
    влитых PR» — тема и картинка разные. Теперь у темы есть свои числа, и
    карточка строится именно из них.

    None — если у темы нет своих чисел: тогда таймлайн остаётся последним
    вариантом, а не первым.
    """
    fact = chosen_fact(post, facts)
    if not fact:
        return None
    numbers = fact.get("numbers") or {}
    source = numbers_source(fact)
    items = []
    for field, one, few, many in NUMBER_FIELDS:
        value = numbers.get(field)
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            continue
        items.append({"value": value, "label": plural(value, one, few, many), "source": source})
    if len(items) < NUMBERS_MIN:
        return None
    title, subtitle = NUMBERS_TITLES.get(fact.get("kind") or "", NUMBERS_TITLE_DEFAULT)
    tags = post.get("tags") or lib.extract_tags(post.get("body_html") or "")
    slug = next((BG_BY_TAG[t] for t in tags if t in BG_BY_TAG), BG_DEFAULT)
    return {
        "template": "numbers",
        "title": title,
        "subtitle": subtitle,
        "items": items,
        "background": {"slug": slug, "prompt": "cached studio background"},
    }


def visible_text(spec: dict) -> list:
    """Всё, что infographic.py напечатает на картинке: заголовки, строки событий, подвал."""
    events = spec.get("events") or []
    items = spec.get("items") or []
    return (
        [spec.get("title") or "", spec.get("subtitle") or ""]
        + [str(ev.get(k) or "") for ev in events for k in ("date", "text", "source")]
        + [str(it.get(k) or "") for it in items for k in ("value", "label", "source")]
    )


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
    hidden = [text for text in visible_text(spec) if not is_public(text)]
    if hidden:
        return None, f"на картинку попала бы внутренняя кухня: «{hidden[0][:60]}» — не рисую"
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


def render_numbers(post: dict, facts: list, out_dir: Path) -> tuple:
    """Нарисовать карточку цифр ВЫБРАННОЙ темы. (путь или None, причина при неудаче)."""
    spec = numbers_spec(post, facts)
    if spec is None:
        return None, "у выбранной темы нет своих чисел — карточку цифр рисовать не из чего"
    hidden = [text for text in visible_text(spec) if not is_public(text)]
    if hidden:
        return None, f"на картинку попала бы внутренняя кухня: «{hidden[0][:60]}» — не рисую"
    try:
        import infographic
    except ImportError as exc:  # нет Pillow — честно говорим, что именно сломалось
        return None, f"инфографика недоступна: {exc}"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"{post.get('id') or 'post'}-numbers.png"
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
    """Варианты медиа по порядку: своё медиа поста, карточка цифр темы, таймлайн — последним.

    Медиа из фактов — только своё (``own_media``): чужая картинка к посту не подходит.
    ``media_ref`` поста, который указывает на медиа-факт ДРУГОГО PR, — след старого выбора
    (у p20261109-810 в записи остался график дрейфа из #4508): он тоже отбрасывается.

    Порядок (issue #4702): карточка цифр темы идёт ПЕРЕД общим таймлайном влитых PR.
    Раньше таймлайн был единственным вариантом для поста без своего медиа — и пост
    про голос получал картинку про несвязанные PR.
    """
    own = [f["source"] for f in own_media(post, facts)]
    foreign = {f.get("source") for f in facts if f.get("kind") == "media"} - set(own)
    ref = post.get("media_ref") or ""
    found = []
    for ref in ([ref] if ref not in foreign else []) + own:
        if ref and ref not in found and resolve(ref, repo_root) is not None and kind_of(Path(ref)):
            found.append(ref)
    if numbers_spec(post, facts) is not None:
        found.append("infographic:numbers")
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
    if choice == "infographic:numbers":
        return render_numbers(post, facts, out_dir)
    path = resolve(choice, repo_root)
    if path is None:
        return None, f"файла {choice} нет ни в клоне, ни в worktree develop"
    problems = lib.media_problems(path, choice)
    if problems:
        return None, "; ".join(problems)
    return path, ""
