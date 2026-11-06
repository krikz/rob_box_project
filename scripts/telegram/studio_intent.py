"""Что Шифу просит сделать и с каким черновиком. Решает код, не модель (ADR-0148).

Разбор 06.11 (#4513): «Может сделать из него анимацию?» без reply ушло в разговор, модель ответила
«да, можно» — и ничего не произошло. Теперь просьба о действии над черновиком — работа, а не беседа:

  * ``classify`` — какое действие: видео, анимация, медиа, короче, проще, подача, правка текста;
    пустая строка — это вопрос или разговор, его отвечает модель-мост;
  * ``target`` — какой черновик: reply на него; id черновика в тексте; единственный на обсуждении;
    единственный тронутый за ``ACTIVE_WINDOW``. Иначе — список кандидатов, и студия переспрашивает кнопками.

Слова — подстроки в нижнем регистре, без регексов: список короткий и читается глазами.
"""

from __future__ import annotations

import time
from typing import Optional

ACTION_VIDEO = "video"
ACTION_ANIMATE = "animate"
ACTION_MEDIA = "media"
ACTION_SHORTER = "shorter"
ACTION_SIMPLER = "simpler"
ACTION_ANGLE = "angle"
ACTION_EDIT = "edit"

# Порядок важен: «ролик до/после» — видео, даже если там есть «картинка»; «анимацию» — раньше «медиа».
ACTION_WORDS = (
    (ACTION_VIDEO, ("видео", "ролик", "до/после", "до и после", "смонтируй", "монтаж", "клип")),
    (ACTION_ANIMATE, ("анимац", "анимир", "гифк", "гиф ", "gif", "оживи")),
    (ACTION_MEDIA, ("картинк", "медиа", "другое фото", "другую фот", "инфографик", "изображени", "графику")),
    (ACTION_SHORTER, ("короче", "сократи", "покороче")),
    (ACTION_SIMPLER, ("проще", "попроще", "упрости")),
    (ACTION_ANGLE, ("подачу", "подача", "другой угол", "другим углом")),
    (ACTION_EDIT, ("перепиши", "поправь", "исправь", "переделай", "убери", "добавь", "замени", "допиши", "вычеркни")),
)
LABELS = {
    ACTION_VIDEO: "видео",
    ACTION_ANIMATE: "анимация",
    ACTION_MEDIA: "другое медиа",
    ACTION_SHORTER: "короче",
    ACTION_SIMPLER: "проще",
    ACTION_ANGLE: "другая подача",
    ACTION_EDIT: "правка текста",
}
ACTIVE_WINDOW = 6 * 3600  # с: черновик, тронутый за это время, считается «тем, о котором речь»
PICK_MAX = 4  # кнопок «к какому черновику?»


def classify(text: str) -> str:
    """Действие по словам Шифу. Пустая строка — не просьба о действии (вопрос, разговор)."""
    low = " ".join((text or "").lower().split()) + " "
    # «Короче, что там с дрейфом?» — слово-связка, а не просьба сократить пост.
    for filler in ("короче, ",):
        if low.startswith(filler):
            low = low[len(filler) :]  # noqa: E203
            break
    for action, words in ACTION_WORDS:
        if any(word in low for word in words):
            return action
    return ""


def drafts(state: dict) -> list:
    """Черновики на обсуждении: [(id, запись)], свежие первыми."""
    found = [
        (post_id, record)
        for post_id, record in (state.get("posts") or {}).items()
        if record.get("status") == "proposed" and record.get("message_id") and record.get("body_html")
    ]
    return sorted(found, key=lambda item: (activity(item[1]), item[1].get("message_id") or 0), reverse=True)


def activity(record: dict) -> float:
    """Когда черновик последний раз трогали (отправка, правка): эпоха, с. Нет отметки — 0."""
    value = record.get("active_at")
    return float(value) if isinstance(value, (int, float)) else 0.0


def target(state: dict, text: str = "", reply_to: Optional[int] = None, now: Optional[float] = None) -> tuple:
    """(id черновика, кандидаты). Черновик найден — кандидатов нет; не найден — переспросить кнопками.

    Пустой id и пустые кандидаты — черновиков на обсуждении нет вовсе.
    """
    found = drafts(state)
    if reply_to:
        for post_id, record in found:
            if record.get("message_id") == reply_to:
                return post_id, []
    named = [post_id for post_id, _record in found if post_id in (text or "")]
    if len(named) == 1:
        return named[0], []
    if not found:
        return "", []
    if len(found) == 1:
        return found[0][0], []
    now = time.time() if now is None else now
    recent = [post_id for post_id, record in found if now - activity(record) <= ACTIVE_WINDOW]
    if len(recent) == 1:
        return recent[0], []
    return "", [post_id for post_id, _record in found[:PICK_MAX]]


def pick_keyboard(state: dict, candidates: list, action: str) -> dict:
    """Кнопки «к какому черновику?»: по одной на кандидата, действие то же."""
    posts = state.get("posts") or {}
    rows = []
    for post_id in candidates:
        title = (posts.get(post_id) or {}).get("title") or ""
        label = f"{post_id}: {title}"[:60]
        rows.append([{"text": label, "callback_data": f"studio:post:{post_id}:{action}"}])
    return {"inline_keyboard": rows}
