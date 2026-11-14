"""Кнопки «Анимация» и «Видео» черновика: план → материал → монтаж → editMessageMedia того же сообщения.

Работа идёт в потоке JobRunner (вне опроса и вне замка состояния), шаги — в индикаторе studio_progress.
«Анимация» даёт GIF ≤ 2 МБ, «Видео» — MP4 ≤ 50 МБ (решение Шифу, #4513). Что собрать, решает
studio_video (код, модель — только выбор между возможными шаблонами).
"""

from __future__ import annotations

from pathlib import Path

import channel_studio as cs
import studio_jobs as jobs
import studio_material as material
import studio_render as render
import studio_state as st
import studio_video as video

FORMATS = {cs.ACTION_ANIMATE: "gif", cs.ACTION_VIDEO: "mp4"}

# Просьба прислать тот же файл ещё раз (Шифу: «так много видосов»).
RESEND_MARKERS = ("пришли", "прислать", "покажи снова", "покажи ещё раз", "отправь ещё раз")
# Слова, по которым видно, что Шифу просит ДРУГОЙ материал, а не повтор того же.
# Разбор 14.11 (issue #4711): «Это не самый длинный заезд, ещё раз говорю найти
# самый длинный» — студия прочитала «ещё раз» как «пришли тот же ролик» и прислала
# старое видео. Повтор поиска — не повтор отправки.
NEW_MATERIAL_MARKERS = (
    "найди",
    "найти",
    "возьми",
    "возьм",
    "длинн",
    "покрыти",
    "максимальн",
    "другой",
    "другое",
    "новый",
    "новое",
    "заново",
    "пересобер",
    "исправ",
    "почини",
    "замени",
)


def wants_resend(request: str) -> bool:
    """Просит ли Шифу прислать тот же файл ещё раз.

    Чистая просьба «пришли ещё раз» — да. Просьба с указанием, что нужен ДРУГОЙ
    материал («найди самый длинный заезд») — нет: тогда ролик надо собрать заново.
    """
    low = (request or "").lower()
    if not any(marker in low for marker in NEW_MATERIAL_MARKERS):
        return any(marker in low for marker in RESEND_MARKERS) or "ещё раз" in low or "еще раз" in low
    return False


def refine_motion(
    bot, state_path: Path, record: dict, facts: list, action: str, progress, chat_id: int, request: str = "", gate=None
) -> bool:
    post_id = record["id"]
    fmt = FORMATS[action]
    progress.step("Смотрю, какой материал есть на песочнице (гейт только на чтение)", "typing")
    gate = gate or material.Gate()
    catalog, reason = material.runs_catalog(gate)
    if reason:
        progress.note(f"Материал песочницы недоступен: {reason}")
    else:
        # Материал, который уже уходил в канал, повторно не предлагаем (issue #4711):
        # 09.11 пост про PR #4555 получил ролик, байт-идентичный опубликованному
        # 06.11. Шифу звал «найди самый длинный заезд» — а приходило старое видео.
        used = material.published_runs(st.load_state(state_path))
        if used:
            fresh = material.fresh_catalog(catalog, used)
            if len(fresh) != len(catalog):
                progress.note(f"Убираю прогоны, чьё видео уже в канале: {', '.join(sorted(used))}")
            catalog = fresh
        progress.note(f"Материал песочницы: прогонов {len(catalog)} ({', '.join(i['id'] for i in catalog)})")
    progress.step("Выбираю шаблон ролика")
    plan, why = video.choose_plan(record, request, facts, catalog, fmt)
    if plan is None:
        progress.fail(why)
        return False
    progress.note(f"Шаблон {plan['template']} {plan.get('runs') or ''}: {why}")
    kind = "видео" if fmt == "mp4" else "анимация"
    plan = video.normalize_plan(plan)
    # Трек: Шифу прислал MP3 в личку — студия помнит его и кладёт под ролик
    # («нуб музычку надо наложить на водосик», 07.11). Без трека в ролике звук
    # не появится: монтаж без аудиодорожки.
    request_low = (request or "").lower()
    music_info = st.load_state(state_path).get("music") or {}
    music_path = music_info.get("path") or ""
    wants_music = any(m in request_low for m in ("музык", "трек", "звук", "аудио"))
    no_music = any(m in request_low for m in ("без музыки", "без звука", "без трека", "без аудио"))
    if music_path and not Path(music_path).exists():
        music_path = ""
    if wants_music and not music_path:
        progress.fail("трека нет — пришли MP3 в личку, наложу на ролик")
        return False
    if music_path and not no_music:
        plan["music"] = music_path
        progress.note(f"Трек: {Path(music_path).name}")
    choice = f"{plan['template']}:{plan['layout']}/{plan['orientation']}:{','.join(plan.get('runs') or [])}"
    # Тот же план с тем же материалом — тот же ролик. Сборка занимает ~50 с,
    # а между нажатиями «Видео» ничего не меняется (жалоба Шифу, 07.11).
    # Слово «пересобери» в просьбе — явная команда пересобрать заново.
    fingerprint = video.fingerprint(plan, fmt, catalog)
    last = record.get("last_motion") or {}
    cached = last.get("path")
    force = "пересобер" in (request or "").lower()
    path, build_seconds, reused = None, 0.0, False
    if not force and last.get("fingerprint") == fingerprint and cached and Path(cached).exists():
        path = Path(cached)
        reused = True
        progress.note("Ролик уже собран с тем же материалом — переиспользую, заново не монтирую")
    if path is None:
        ctx = video.Context(gate, jobs.run_dir() / "media" / "video", facts, record, progress)
        path, reason = video.build(plan, fmt, ctx, catalog)
        if path is None:
            progress.fail(reason)
            return False
        build_seconds = ctx.build_seconds
    size = path.stat().st_size
    # Тот же файл уже присылали — второй раз не шлём (Шифу: «так много видосов»).
    # Но «ещё раз» в просьбе про поиск нового материала — это повтор ПОИСКА, а не
    # повтор отправки (issue #4711).
    again = wants_resend(request)
    already = last.get("delivered") == fingerprint and last.get("path") == str(path)
    video_message_id = last.get("message_id") if already else None
    # Под какой просьбой ролик уходил в чат в прошлый раз: новая просьба — Шифу
    # ждёт ролик в чате (нажал «Видео» после вопроса), та же самая просьба — второй
    # раз молчим, чтобы не сыпать одинаковыми роликами (разбор 14.11, issue #4713).
    last_request = str(last.get("request") or "")
    now_request = (request or "").strip()
    same_intent = last_request == now_request
    should_send = (not already) or again or (not same_intent)
    sent_to_chat = False
    if should_send:
        progress.step(f"Присылаю {kind} в чат: {path.name}, {render.mb(size)}", "upload_video")
        # Видео — отдельным сообщением, а не правкой старой картинки: Telegram
        # кэширует превью сообщения, и после editMessageMedia клиент продолжает
        # показывать старую картинку («бот не прислал видео», Шифу, 07.11).
        sent = bot.send_media(chat_id, path, caption=f"Черновик {post_id}: {kind} · {plan['template']}")
        if not sent.get("ok"):
            progress.fail(sent.get("description") or "Telegram не принял файл")
            return False
        # reply на ролик — это reply на черновик (studio_intent.message_ids).
        video_message_id = (sent.get("result") or {}).get("message_id")
        sent_to_chat = True
    else:
        progress.note("Этот ролик уже присылан под эту же просьбу — второй раз не отправляю")
    # Черновик приводим к тому же виду: правка не обязательна — видео уже доставлено.
    updated = dict(record, media_path=str(path), media_choice=choice)
    ok, why = jobs.update_draft(bot, updated, chat_id, new_media=path)
    if not ok:
        progress.note(f"Черновик оставил как есть: {why}")
    jobs.save_fields(
        state_path,
        post_id,
        {
            "media_path": str(path),
            "media_choice": choice,
            "message_kind": "media",
            "last_motion": {
                "template": plan["template"],
                "layout": plan["layout"],
                "orientation": plan["orientation"],
                "runs": plan.get("runs") or [],
                "format": fmt,
                "bytes": size,
                "build_seconds": build_seconds,
                "path": str(path),
                "fingerprint": fingerprint,
                "delivered": fingerprint if sent_to_chat else last.get("delivered"),
                "message_id": video_message_id,
                # Под какой просьбой ролик ушёл в чат: по этому тексту студия
                # понимает, что ролик по этой просьбе уже присылан (issue #4713).
                "request": now_request,
            },
        },
    )
    if not sent_to_chat:
        # Индикатор не должен врать: в чат ничего не ушло — так и говорим.
        progress.finish(f"Ролик по этой просьбе уже присылан, нового не слал: {path.name}, {render.mb(size)}")
        return True
    if reused:
        progress.finish(f"Готово: {kind} в чате, {render.mb(size)}, готовый ролик — заново не монтировал")
    else:
        progress.finish(f"Готово: {kind} в чате, {render.mb(size)}, собрано за {build_seconds} с")
    return True
