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
ACTION_POST = "post"
ACTION_PUBLISH = "publish"

# Порядок важен: «ролик до/после» — видео, даже если там есть «картинка»; «анимацию» — раньше «медиа».
ACTION_WORDS = (
    # Разбор 09.11: «Отлично! публикуем!» ушло в разговор, модель ответила «Публикуем» и дала
    # кнопку «пришли пост» к чужому черновику. Публикация словами — первая: «публикуем видео»
    # — это публикация, а не новый ролик. Решение всё равно кнопкой (studio_messages).
    (ACTION_PUBLISH, ("публику", "опубликуй", "опубликуем", "опубликовать", "выпускай")),
    (
        ACTION_VIDEO,
        (
            "видео",
            "видос",
            "видяш",
            "ролик",
            "до/после",
            "до и после",
            "до после",
            "до-после",
            "смонтируй",
            "монтаж",
            "клип",
        ),
    ),
    (ACTION_ANIMATE, ("анимац", "анимир", "гифк", "гиф ", "gif", "оживи")),
    (
        ACTION_POST,
        (
            "пришли пост",
            "пришли мне пост",
            "пришли черновик",
            "покажи пост",
            "покажи черновик",
            "собранный пост",
            "готовый пост",
            "перешли пост",
            "скинь пост",
            "пост целиком",
        ),
    ),
    (ACTION_MEDIA, ("картинк", "медиа", "другое фото", "другую фот", "инфографик", "изображени", "графику")),
    (ACTION_SHORTER, ("короче", "сократи", "покороче")),
    (ACTION_SIMPLER, ("проще", "попроще", "упрости")),
    (ACTION_ANGLE, ("подачу", "подача", "другой угол", "другим углом")),
    (ACTION_EDIT, ("перепиши", "поправь", "исправь", "переделай", "убери", "добавь", "замени", "допиши", "вычеркни")),
)
LABELS = {
    ACTION_PUBLISH: "опубликовать",
    ACTION_POST: "прислать пост",
    ACTION_VIDEO: "видео",
    ACTION_ANIMATE: "анимация",
    ACTION_MEDIA: "другое медиа",
    ACTION_SHORTER: "короче",
    ACTION_SIMPLER: "проще",
    ACTION_ANGLE: "другая подача",
    ACTION_EDIT: "правка текста",
}
ACTIVE_WINDOW = 6 * 3600  # с: черновик, тронутый за это время, считается «тем, о котором речь»
# с: черновик, который студия показала только что, — «тот, о котором речь», если другие давно не трогали
LAST_SHOWN_WINDOW = 30 * 60
# «Не публикуй», «рано публиковать» — это не просьба опубликовать.
PUBLISH_NEGATIONS = ("не публ", "не опубл", "рано публ", "погоди публ", "подожди публ")
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
            if action == ACTION_PUBLISH and any(neg in low for neg in PUBLISH_NEGATIONS):
                return ""
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


def message_ids(record: dict) -> set:
    """Сообщения, reply на которые — это reply на черновик: сам черновик и его ролик.

    Разбор 08.11: ролик приходит отдельным сообщением, и ответ на него («оформи текст»)
    не считался ответом на черновик — уходил в разговор, где модель только обещала.
    """
    ids = {record.get("message_id"), (record.get("last_motion") or {}).get("message_id"), record.get("post_message_id")}
    ids.discard(None)
    return ids


def target(state: dict, text: str = "", reply_to: Optional[int] = None, now: Optional[float] = None) -> tuple:
    """(id черновика, кандидаты). Черновик найден — кандидатов нет; не найден — переспросить кнопками.

    Пустой id и пустые кандидаты — черновиков на обсуждении нет вовсе.
    """
    found = drafts(state)
    if reply_to:
        for post_id, record in found:
            if reply_to in message_ids(record):
                return post_id, []
    named = [post_id for post_id, _record in found if post_id in (text or "")]
    if len(named) == 1:
        return named[0], []
    if not found:
        return "", []
    if len(found) == 1:
        return found[0][0], []
    now = time.time() if now is None else now
    # found отсортирован по свежести. Только что показанный черновик при давно не тронутых
    # остальных — тот, о котором речь (09.11: Шифу смотрел пост про навигацию и написал «публикуем»).
    fresh = [post_id for post_id, record in found if now - activity(record) <= LAST_SHOWN_WINDOW]
    if len(fresh) == 1:
        return fresh[0], []
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
