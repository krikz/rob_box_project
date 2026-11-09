"""Тяжёлая работа студии: собрать черновик, переделать текст, сменить медиа.

Всё здесь идёт ВНЕ опроса кнопок (поток JobRunner в channel_studio.watch) и ВНЕ замка
состояния: модель думает десятки секунд, рендер — секунды. Состояние пишется короткими
транзакциями до и после.

Каждая работа ведёт индикатор (studio_progress.Progress): одно сообщение, которое правится
на месте по реальным шагам, плюс sendChatAction. Ошибка — строка «Не вышло на шаге …: причина».
Фразы об успехе строит код из результата вызова (ADR-0148), модель их не пишет.
"""

from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path
from typing import Optional

import channel_lib as lib
import channel_studio as cs
import studio_media as smedia
import studio_progress as sp
import studio_rules
import studio_state as st

REPO = Path(__file__).resolve().parents[2]
REFINE_LABELS = {
    cs.ACTION_SHORTER: "короче",
    cs.ACTION_SIMPLER: "проще",
    cs.ACTION_ANGLE: "другая подача",
    cs.ACTION_MEDIA: "другое медиа",
    cs.ACTION_FREEFORM: "правка по твоему сообщению",
    cs.ACTION_ANIMATE: "анимация",
    cs.ACTION_VIDEO: "видео",
}
# Действия, которым не нужны факты сессии: ролик берёт материал с песочницы, а не из facts.json.
NO_FACTS_ACTIONS = (cs.ACTION_ANIMATE, cs.ACTION_VIDEO)
# «Пришли пост» — готовое, модель и факты не нужны.
FACTS_FREE_ACTIONS = (cs.ACTION_POST,)
GH_REPO = os.environ.get("STUDIO_GH_REPO", "krikz/rob_box_project")


def run_dir() -> Path:
    return Path(os.environ.get("STUDIO_TMP", REPO / "docker" / "studio" / "run"))


def load_run_facts(path: Optional[Path] = None) -> list:
    """Факты последней сессии — из файла, который собрала studio_facts."""
    path = path or Path(os.environ.get("STUDIO_FACTS", run_dir() / "facts.json"))
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    if isinstance(data, list):
        return data
    return data.get("visible_facts") or data.get("facts") or []


def _own_facts(record: dict, facts: list) -> list:
    keys = set(record.get("fact_keys") or [])
    return [f for f in facts if f.get("key") in keys]


def _check_line(checks: dict) -> str:
    """«Проверяю цифры: 5/5 ✓, длина 640/1024» — из счёта, который посчитал код."""
    total, good = checks.get("numbers", 0), checks.get("numbers_ok", 0)
    mark = "✓" if total == good else "✗"
    line = f"Проверяю цифры: {good}/{total} {mark}"
    line += ", имена ✓" if not checks.get("names_bad") else f", имён не из фактов: {checks['names_bad']}"
    if checks.get("length"):
        line += f", длина {checks['length']}/{lib.CAPTION_LIMIT}"
    return line


def _produce(record: dict, facts: list, instruction: str, shifu_rules: Optional[list] = None) -> dict:
    """Переписать текст поста моделью; проверка — кодом (studio_text.produce_post)."""
    import studio_text

    idea = {
        "title": record.get("title", ""),
        "show": record.get("show") or record.get("title", ""),
        "deadline": record.get("deadline", ""),
        "media_ref": record.get("media_ref", ""),
        "fact_keys": record.get("fact_keys") or [],
        "current_body": record.get("body_html") or "",
        "shifu_rules": shifu_rules or [],
    }
    has_media = record.get("message_kind") == "media" or bool(record.get("media_path"))
    return studio_text.produce_post(idea, facts, instruction=instruction, has_media=has_media)


def update_draft(bot, record: dict, chat_id: int, new_media: Optional[Path] = None) -> tuple:
    """Обновить сообщение черновика на месте. (ok, причина)."""
    post_id = str(record.get("id") or "")
    message_id = record.get("message_id")
    if not message_id:
        return False, "у черновика нет message_id — править нечего"
    caption = cs.draft_caption(record)
    keyboard = cs.post_keyboard(post_id)
    if new_media is not None:
        # editMessageMedia умеет и заменить текстовое сообщение медиа (Bot API 10.3).
        result = bot.edit_message_media(chat_id, message_id, new_media, caption=caption, keyboard=keyboard)
    elif record.get("message_kind") == "media":
        result = bot.edit_message_caption(chat_id, message_id, caption, keyboard=keyboard)
    else:
        result = bot.edit_message_text(chat_id, message_id, caption, keyboard=keyboard)
    if not result.get("ok"):
        result = _edit_as_it_really_is(bot, record, chat_id, caption, keyboard, result)
    if result.get("ok"):
        return True, ""
    description = result.get("description") or "Telegram не принял правку"
    if "not modified" in description:
        return False, "новая версия совпала со старой — Telegram не меняет сообщение"
    return False, description


def _edit_as_it_really_is(bot, record: dict, chat_id: int, caption: str, keyboard, failed: dict) -> dict:
    """Запись говорит одно, а сообщение в Telegram другое — правим по тому, что там на деле.

    Живая проверка 09.11: у i20261106-1-post в записи «media» (ролик пересадили вручную),
    а в Telegram — текстовое сообщение; editMessageCaption: «there is no caption in the
    message to edit». Есть медиа-файл — текстовое сообщение становится медиа с подписью,
    нет — правим текст. Обратный случай — правим подпись. Фактический вид пишем в
    ``record["message_kind"]`` (вызывающий сохраняет его).
    """
    description = (failed.get("description") or "").lower()
    message_id = record.get("message_id")
    if "no caption" in description:
        path = smedia.resolve(record.get("media_path") or "")
        if path is not None:
            result = bot.edit_message_media(chat_id, message_id, path, caption=caption, keyboard=keyboard)
            kind = "media"
        else:
            result = bot.edit_message_text(chat_id, message_id, caption, keyboard=keyboard)
            kind = "text"
    elif "no text" in description:
        result = bot.edit_message_caption(chat_id, message_id, caption, keyboard=keyboard)
        kind = "media"
    else:
        return failed
    if result.get("ok"):
        record["message_kind"] = kind
    return result


def save_fields(state_path: Path, post_id: str, fields: dict) -> None:
    """Дописать поля черновика под замком. active_at — отметка «черновик трогали» для studio_intent.target."""
    with st.state_transaction(state_path) as current:
        record = current.setdefault("posts", {}).setdefault(post_id, {"id": post_id})
        record.update(fields)
        record["active_at"] = time.time()


_save_fields = save_fields


def report(state_path: Path, post_id: str, action: str, progress, started: float, out, ok: bool) -> None:
    """Итог работы в журнал и в состояние: причина неудачи — в state.posts[id].last_error (#4513).

    Раньше журнал писал «не вышло за 13.5 с» без причины, а причина жила только в индикаторе у Шифу.
    """
    seconds = time.monotonic() - started
    failure = progress.failure or {}
    line = f"пост {post_id or '—'}: {action} → {'готово' if ok else 'не вышло'} за {seconds:.1f} с"
    if not ok:
        line += f" на шаге «{failure.get('step', '?')}»: {failure.get('reason', 'причина не записана')}"
    out(line + f" (индикатор message_id={progress.message_id}, правок {progress.edits})")
    if not post_id:
        return
    with st.state_transaction(state_path) as current:
        record = (current.get("posts") or {}).get(post_id)
        if record is None:
            return  # черновика нет — пустую запись не заводим (разбор 06.11)
        if ok:
            record.pop("last_error", None)
            record["active_at"] = time.time()
            return
        record["last_error"] = {
            "at": st.now_iso(),
            "action": action,
            "step": failure.get("step", ""),
            "reason": failure.get("reason", "причина не записана"),
        }


def request_material(post_id: str, what: str) -> str:
    """Материала нет: заводим issue «материал для канала: …». Возвращает ссылку или причину."""
    title = f"материал для канала: {what}"
    body = (
        f"Черновику студии {post_id} не хватает материала.\n\nЧто нужно: {what}\n\n"
        "Материал ждём в docs/channel/media/ — координатор кладёт его через PR, "
        "студия берёт свежие материалы из worktree на origin/develop."
    )
    try:
        done = subprocess.run(
            ["gh", "issue", "create", "--repo", GH_REPO, "--title", title, "--body", body],
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "issue не заведён: gh недоступен"
    if done.returncode == 0 and done.stdout.strip():
        return done.stdout.strip().splitlines()[-1]
    return f"issue не заведён: {(done.stderr or '').strip()[:120] or 'gh вернул ошибку'}"


# --------------------------------------------------------------------------- кнопки черновика


def refine_job(
    bot, state_path: Path, post_id: str, action: str, reply_to: Optional[int] = None, out=print, instruction: str = ""
) -> bool:
    """Кнопки «Короче/Проще/Подача/Медиа»: индикатор шагов, работа, правка черновика на месте."""
    chat_id = cs.SHIFU_CHAT_ID
    started = time.monotonic()
    progress = sp.Progress(bot, chat_id, f"Черновик {post_id}: {REFINE_LABELS.get(action, action)}", reply_to=reply_to)
    progress.step("Беру черновик и факты сессии")
    try:
        ok = _refine(bot, state_path, post_id, action, progress, chat_id, out, instruction)
    except Exception as exc:  # noqa: BLE001 — упавшая работа тоже обязана оставить причину
        progress.fail(f"{type(exc).__name__}: {exc}")
        ok = False
    if not ok and progress.failure is None:
        progress.fail("работа закончилась без результата")
    report(state_path, post_id, action, progress, started, out, ok)
    return ok


def _refine(
    bot, state_path: Path, post_id: str, action: str, progress, chat_id: int, out, instruction: str = ""
) -> bool:
    record = (st.load_state(state_path).get("posts") or {}).get(post_id) or {}
    if not record.get("body_html"):
        progress.fail("черновика нет в состоянии студии — пришлю новый со следующей сессией")
        out(f"пост {post_id}: записи нет — переделка невозможна")
        return False
    if record.get("status") in ("approved", "rejected"):
        progress.fail(f"черновик уже {'одобрен' if record['status'] == 'approved' else 'отклонён'} — не трогаю")
        return False
    facts = load_run_facts()
    if not facts and action not in NO_FACTS_ACTIONS + FACTS_FREE_ACTIONS:
        progress.fail("факты сессии не найдены (docker/studio/run/facts.json)")
        return False
    own = _own_facts(record, facts)
    progress.note(f"Беру черновик и факты сессии: своих {len(own)} из {len(facts)}")
    if action in NO_FACTS_ACTIONS:
        import studio_motion

        return studio_motion.refine_motion(bot, state_path, record, facts, action, progress, chat_id, instruction)
    if action == cs.ACTION_MEDIA:
        return _refine_media(bot, state_path, record, facts, progress, chat_id)
    if action == cs.ACTION_POST:
        return _refine_post(bot, state_path, record, progress, chat_id)
    return _refine_text(bot, state_path, record, own or facts, action, progress, chat_id, instruction)


def _remember_post_message(state_path: Path, record: dict, sent: dict) -> None:
    """Присланный пост — тоже сообщение черновика: reply на него и «публикуем» после него — про него.

    Разбор 09.11: Шифу посмотрел присланный пост про навигацию и написал «публикуем», а студия
    не знала, что это сообщение её черновика, и выбрала другой.
    """
    message_id = (sent.get("result") or {}).get("message_id")
    if message_id:
        save_fields(state_path, record["id"], {"post_message_id": message_id})


def _refine_post(bot, state_path: Path, record: dict, progress, chat_id: int) -> bool:
    """«Пришли пост» — прислать собранный черновик в чат: текст и его медиа.

    Шифу (07.11, 23:37): «Так теперь пришли мне собранный пост» → бот отвечал
    «я не могу сам отправить готовый файл» и предлагал собрать ролик заново.
    Действия «прислать готовое» в студии не было — теперь есть.
    """
    import channel_lib as lib

    body = (record.get("body_html") or record.get("body") or "").strip()
    if not body:
        progress.fail("у черновика нет текста — пришлю заново")
        return False
    media = record.get("media_path") or ""
    path = Path(media) if media else None
    if path is not None and not path.exists():
        path = None
    if path is not None:
        caption = body[: lib.CAPTION_LIMIT]
        progress.step(f"Присылаю пост с медиа: {path.name}", "upload_video")
        sent = bot.send_media(chat_id, path, caption=caption)
        if not sent.get("ok"):
            progress.fail(sent.get("description") or "Telegram не принял файл")
            return False
        _remember_post_message(state_path, record, sent)
        note = "" if len(body) <= lib.CAPTION_LIMIT else " (текст обрезан: не влезает в подпись)"
        progress.finish(f"Готово: прислал пост с медиа {path.name}{note}")
        return True
    progress.step("Присылаю пост текстом", "typing")
    sent = bot.send_message(chat_id, body[: lib.MESSAGE_LIMIT], parse_mode="HTML")
    if not sent.get("ok"):
        progress.fail(sent.get("description") or "Telegram не принял текст")
        return False
    _remember_post_message(state_path, record, sent)
    progress.finish("Готово: прислал пост текстом")
    return True


def _refine_media(bot, state_path: Path, record: dict, facts: list, progress, chat_id: int) -> bool:
    post_id = record["id"]
    progress.step("Подбираю медиа", "upload_photo")
    variants = smedia.options(record, facts)
    choice = smedia.next_option(record, facts)
    if choice is None:
        if not variants:
            link = request_material(post_id, "картинка или видео к посту")
            progress.fail(f"материала нет ни в фактах, ни для инфографики; {link}")
        else:
            progress.fail("другого материала нет — в черновике уже единственный вариант")
        return False
    progress.note(f"Подбираю медиа: вариант {variants.index(choice) + 1} из {len(variants)}")
    if choice.startswith("infographic:"):
        progress.step("Рисую инфографику: таймлайн влитых PR", "upload_photo")
    path, reason = smedia.materialize(choice, record, facts, run_dir() / "media")
    if path is None:
        progress.fail(reason)
        return False
    progress.step(f"Меняю медиа в черновике: {path.name}", "upload_photo")
    updated = dict(record, media_path=str(path), media_choice=choice)
    if not choice.startswith("infographic:"):
        updated["media_ref"] = choice
    ok, reason = update_draft(bot, updated, chat_id, new_media=path)
    if not ok:
        progress.fail(reason)
        return False
    _save_fields(
        state_path,
        post_id,
        {k: updated.get(k) for k in ("media_path", "media_choice", "media_ref") if updated.get(k) is not None}
        | {"message_kind": "media"},
    )
    progress.finish("Готово: медиа в черновике заменено")
    return True


def _refine_text(
    bot, state_path: Path, record: dict, facts: list, action: str, progress, chat_id: int, instruction: str = ""
) -> bool:
    import studio_text

    progress.step(f"Пишу текст ({studio_text.DEFAULT_MODEL})")
    if not instruction:
        instruction = cs.REFINE_INSTRUCTIONS[action]
    else:
        # Свободная правка Шифу: его слова — задание, факты и правила канала остаются в силе.
        instruction = f"Правка от Шифу: «{instruction[:500]}». Выполни её; числа и имена — только из фактов."
    produced = _produce(record, facts, instruction, studio_rules.active(st.load_state(state_path)))
    if produced.get("error"):
        progress.fail(str(produced["error"])[:200])
        return False
    progress.step("Проверяю цифры и имена по фактам")
    progress.note(_check_line(produced.get("checks") or {}))
    if produced.get("problems"):
        progress.fail("; ".join(produced["problems"][:3]))
        return False
    progress.set_why(produced.get("why") or "")
    if produced.get("why_problems"):
        progress.note(f"Пояснение модели не показываю: {produced['why_problems'][0]}")
    progress.step("Правлю черновик на месте")
    updated = dict(record, body_html=produced.get("body_html") or "", tags=produced.get("tags") or record.get("tags"))
    ok, reason = update_draft(bot, updated, chat_id)
    if not ok:
        progress.fail(reason)
        return False
    fields = {"body_html": updated["body_html"], "tags": updated["tags"], "message_kind": updated.get("message_kind")}
    _save_fields(state_path, record["id"], {k: v for k, v in fields.items() if v is not None})
    progress.finish("Готово: черновик поправлен на месте")
    return True


# --------------------------------------------------------------------------- черновик сессии


def draft_job(
    bot, state_path: Path, facts_path: Path, chat_id: Optional[int] = None, note: str = "", out=print
) -> bool:
    """Собрать готовый пост из фактов и прислать Шифу — сразу с графикой и кнопками.

    Индикатор показывает настоящие шаги: факты → текст (MiniMax) → проверка → медиа → отправка.
    """
    import studio_draft

    chat_id = chat_id or cs.SHIFU_CHAT_ID
    title = "Студия: новый черновик" + (f" ({note})" if note else "")
    progress = sp.Progress(bot, chat_id, title)
    progress.step("Читаю факты сессии")
    facts = load_run_facts(facts_path)
    if not facts:
        progress.fail(f"фактов нет в {facts_path}")
        return False
    topic = studio_draft.pick_fact(facts)
    progress.note(f"Читаю факты сессии: {len(facts)}, тема — {topic.get('source', '?').split(' (')[0]}")
    progress.step(f"Пишу текст ({studio_draft.st.DEFAULT_MODEL})")
    post = studio_draft.build_post(facts, shifu_rules=studio_rules.active(st.load_state(state_path)))
    if post.get("error"):
        problems = "; ".join((post.get("problems") or [])[:3])
        progress.fail(f"{post['error']}{': ' + problems if problems else ''} (попыток: {post.get('attempts', 1)})")
        return False
    if post.get("attempts", 1) > 1:
        progress.note("Пишу текст: со второй попытки — первая не прошла проверку")
    progress.step("Проверяю цифры и имена по фактам")
    progress.note(_check_line(post.get("checks") or {}))
    progress.set_why(post.get("why") or "")
    post["id"] = studio_draft.post_id_for(post)
    post["status"] = "proposed"
    post["note"] = note
    existing = (st.load_state(state_path).get("posts") or {}).get(post["id"]) or {}
    if existing.get("queue_file") or existing.get("status") in ("approved", "rejected"):
        progress.fail(f"черновик {post['id']} уже {existing.get('status')} — второй не шлю")
        return False
    _attach_media(post, facts, progress)
    cs.register_post(state_path, post)
    progress.step("Отправляю черновик", "upload_photo" if post.get("media_path") else "typing")
    sent = cs.send_post(bot, post, chat_id, out=out)
    if not sent:
        progress.fail("Telegram не принял черновик")
        return False
    cs.register_post(state_path, post)
    progress.finish(f"Готово: черновик {post['id']} ниже, кнопки под ним")
    out(f"индикатор message_id={progress.message_id}, черновик message_id={post.get('message_id')}")
    return True


def _attach_media(post: dict, facts: list, progress) -> None:
    """Первый вариант медиа: готовый файл или инфографика. Нет — честная строка и пост без медиа."""
    progress.step("Подбираю медиа", "upload_photo")
    variants = smedia.options(post, facts)
    if not variants:
        progress.note("Подбираю медиа: материала нет — черновик уйдёт без картинки")
        return
    choice = variants[0]
    if choice.startswith("infographic:"):
        progress.step("Рисую инфографику: таймлайн влитых PR", "upload_photo")
    path, reason = smedia.materialize(choice, post, facts, run_dir() / "media")
    if path is None:
        progress.note(f"Медиа не вышло: {reason} — черновик уйдёт без картинки")
        return
    post["media_path"] = str(path)
    post["media_choice"] = choice
    if choice.startswith("infographic:"):
        progress.note(f"Рисую инфографику: таймлайн влитых PR → {path.name}")
    else:
        post["media_ref"] = choice
        progress.note(f"Подбираю медиа: {choice}")
