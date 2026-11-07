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
import studio_video as video

FORMATS = {cs.ACTION_ANIMATE: "gif", cs.ACTION_VIDEO: "mp4"}


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
        progress.note(f"Материал песочницы: прогонов {len(catalog)} ({', '.join(i['id'] for i in catalog)})")
    progress.step("Выбираю шаблон ролика")
    plan, why = video.choose_plan(record, request, facts, catalog, fmt)
    if plan is None:
        progress.fail(why)
        return False
    progress.note(f"Шаблон {plan['template']} {plan.get('runs') or ''}: {why}")
    ctx = video.Context(gate, jobs.run_dir() / "media" / "video", facts, record, progress)
    path, reason = video.build(plan, fmt, ctx, catalog)
    if path is None:
        progress.fail(reason)
        return False
    size = path.stat().st_size
    progress.step(f"Меняю медиа в черновике: {path.name}, {render.mb(size)}", "upload_video")
    updated = dict(record, media_path=str(path), media_choice=f"{plan['template']}:{','.join(plan.get('runs') or [])}")
    ok, reason = jobs.update_draft(bot, updated, chat_id, new_media=path)
    if not ok:
        progress.fail(reason)
        return False
    jobs.save_fields(
        state_path,
        post_id,
        {
            "media_path": str(path),
            "media_choice": updated["media_choice"],
            "message_kind": "media",
            "last_motion": {
                "template": plan["template"],
                "runs": plan.get("runs") or [],
                "format": fmt,
                "bytes": size,
                "build_seconds": ctx.build_seconds,
            },
        },
    )
    kind = "видео" if fmt == "mp4" else "анимация"
    progress.finish(f"Готово: {kind} в черновике, {render.mb(size)}, собрано за {ctx.build_seconds:.0f} с")
    return True
