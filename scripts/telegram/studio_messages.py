"""Сообщения Шифу в личке студии: каждое — в работу или честный ответ, молча ничего (#4513).

Маршрут решает код (ADR-0148), по порядку:
  1. не из чата Шифу — не наше: только сдвигаем смещение;
  2. файл (фото, видео, GIF, документ; подпись — в ``caption``) → studio_inbox.attach_job в черновик;
  3. стикер, голосовое и прочее, что студия не принимает → «такое пока не умею», с перечнем, что умею;
  4. текст (или подпись) с просьбой о действии (studio_intent.classify) → работа над черновиком
     (studio_intent.target); черновиков несколько и неясно, над каким, → кнопки «к какому черновику?»;
  5. правило к постам («в постах не пиши про …», studio_rules) → код сохраняет его в состояние,
     сам отвечает, что записал, и переписывает по нему черновик на обсуждении;
  6. reply на черновик (или на его ролик) без слова-действия → правка текста по его словам;
  7. остальное — вопрос: отвечает модель-мост (channel_studio.reply_to_shifu).
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import channel_studio as cs
import studio_inbox as inbox
import studio_intent as intent
import studio_rules as rules

CAN_DO = "текст, фото, видео, GIF и документы (до 20 МБ)"


def route(bot, state_path: Path, state: Optional[dict], update_id: int, message: Optional[dict], runner, out) -> int:
    """Разобрать одно сообщение. Возвращает 1, если оно от Шифу (обработано), иначе 0."""
    decision = cs._with_state(state_path, state, lambda cur: decide(cur, update_id, message or {}))
    kind = decision["kind"]
    if kind == "skip":
        return 0
    out(f"сообщение от Шифу ({kind}): {decision.get('note', '')[:60]}")
    if kind == "unsupported":
        runner.submit("reply", cs._say, bot, decision["answer"])
    elif kind == "ask":
        runner.submit("reply", _ask, bot, decision)
    elif kind == "nodraft":
        runner.submit("reply", cs._say, bot, decision["answer"])
    elif kind == "file":
        key = f"post:{decision['post_id']}" if decision["post_id"] else f"file:{decision['message_id']}"
        args = (
            bot,
            state_path,
            decision["post_id"],
            decision["media"],
            decision["caption"],
            decision["message_id"],
            out,
        )
        _submit(runner, bot, key, (inbox.attach_job,) + args)
    elif kind == "action":
        _action(bot, state_path, decision, runner, out)
    elif kind == "confirm":
        runner.submit("reply", _confirm, bot, decision)
    elif kind == "rule":
        runner.submit("reply", cs._say, bot, decision["answer"])
        if decision.get("post_id"):
            _action(bot, state_path, decision, runner, out)
    else:
        runner.submit("reply", cs.reply_to_shifu, bot, state_path, decision["note"], out)
    return 1


def decide(state: dict, update_id: int, message: dict) -> dict:
    """Решение по сообщению под замком состояния (быстро, без сети)."""
    cs._advance_offset(state, update_id)
    if (message.get("chat") or {}).get("id") != cs.SHIFU_CHAT_ID:
        return {"kind": "skip"}
    note = cs.capture_message(state, message)
    media = inbox.media_of(message)
    reply_to = (message.get("reply_to_message") or {}).get("message_id")
    base = {"note": note, "message_id": message.get("message_id")}
    if media is not None:
        return _decide_file(state, base, media, reply_to)
    if not note:
        what = inbox.unsupported_kind(message) or "такое сообщение"
        answer = f"Получил {what}, но такое пока не умею принимать. Умею: {CAN_DO}."
        return dict(base, kind="unsupported", note=what, answer=answer)
    if rules.is_rule(note):
        return _decide_rule(state, base, reply_to)
    action = intent.classify(note)
    if not action:
        post_id, _candidates = intent.target(state, note, reply_to)
        replied = reply_to and post_id and reply_to in intent.message_ids((state.get("posts") or {}).get(post_id, {}))
        if not replied:
            return dict(base, kind="talk")
        action = intent.ACTION_EDIT
    return _decide_action(state, base, action, reply_to)


def _decide_rule(state: dict, base: dict, reply_to: Optional[int]) -> dict:
    """Правило к постам: сохраняет код; черновик на обсуждении переписывается по нему."""
    rule, is_new = rules.remember(state, base["note"])
    post_id, candidates = intent.target(state, base["note"], reply_to)
    if candidates:
        post_id = ""
    answer = rules.reply_text(rule, is_new, post_id, len(rules.active(state)), several=bool(candidates))
    return dict(base, kind="rule", action=cs.ACTION_FREEFORM, post_id=post_id, answer=answer)


def _decide_file(state: dict, base: dict, media: dict, reply_to: Optional[int]) -> dict:
    post_id, candidates = intent.target(state, base["note"], reply_to)
    if candidates:
        state["pending_request"] = {
            "action": cs.ACTION_FILE,
            "media": media,
            "text": base["note"],
            "message_id": base["message_id"],
            "at": time.time(),
        }
        keyboard = _pick(state, candidates, cs.ACTION_FILE)
        return dict(base, kind="ask", action=cs.ACTION_FILE, candidates=candidates, keyboard=keyboard)
    note = base["note"] or media.get("name") or media["kind"]
    return dict(base, kind="file", post_id=post_id, media=media, caption=base["note"], note=note)


def _decide_action(state: dict, base: dict, action: str, reply_to: Optional[int]) -> dict:
    post_id, candidates = intent.target(state, base["note"], reply_to)
    label = intent.LABELS.get(action, action)
    if candidates:
        state["pending_request"] = {"action": action, "text": base["note"], "at": time.time()}
        return dict(base, kind="ask", action=action, candidates=candidates, keyboard=_pick(state, candidates, action))
    if not post_id:
        answer = f"Понял: {label}. Но черновика на обсуждении нет — делать не над чем. Пришлю, как соберу новый."
        return dict(base, kind="nodraft", answer=answer)
    if action == intent.ACTION_PUBLISH:
        return dict(base, kind="confirm", action=action, post_id=post_id, title=_title(state, post_id))
    return dict(base, kind="action", action=action, post_id=post_id)


def _title(state: dict, post_id: str) -> str:
    record = (state.get("posts") or {}).get(post_id) or {}
    return (record.get("title") or post_id)[:80]


def _confirm(bot, decision: dict) -> None:
    """«Публикуем» словами: код называет черновик и даёт кнопку, публикует кнопка (как у черновика).

    Разбор 09.11: на «Отлично! публикуем!» модель ответила «Публикуем» — ничего не опубликовав —
    и дала кнопку к другому черновику. Теперь ответ строит код и черновик назван прямо.
    """
    post_id = decision["post_id"]
    text = (
        f"Публикую черновик {post_id} «{decision['title']}»? Кнопка ниже ставит его в очередь канала "
        "на ближайший слот (вт/чт/сб 11:00). Если не тот — ответь на нужный черновик словом «публикуем»."
    )
    keyboard = {
        "inline_keyboard": [
            [{"text": f"Опубликовать {post_id}", "callback_data": f"studio:post:{post_id}:{cs.ACTION_PUBLISH}"}]
        ]
    }
    bot.send_message(cs.SHIFU_CHAT_ID, text, keyboard=keyboard, parse_mode=None)


def _pick(state: dict, candidates: list, action: str) -> dict:
    return intent.pick_keyboard(state, candidates, action)


def _ask(bot, decision: dict) -> None:
    label = intent.LABELS.get(
        decision["action"], "файл" if decision["action"] == cs.ACTION_FILE else decision["action"]
    )
    text = f"К какому черновику: {label}? Черновиков на обсуждении несколько — выбери кнопкой."
    bot.send_message(cs.SHIFU_CHAT_ID, text, keyboard=decision["keyboard"], parse_mode=None)


def _submit(runner, bot, key: str, args: tuple) -> None:
    if not runner.claim(key):
        bot.send_message(cs.SHIFU_CHAT_ID, "Уже работаю над этим черновиком — пришли, когда закончу.", parse_mode=None)
        return
    runner.submit(key, *args)


def _action(bot, state_path: Path, decision: dict, runner, out) -> None:
    import studio_jobs as jobs

    post_id, action = decision["post_id"], decision["action"]
    instruction = decision["note"] if action in (cs.ACTION_FREEFORM, cs.ACTION_VIDEO, cs.ACTION_ANIMATE) else ""
    args = (jobs.refine_job, bot, state_path, post_id, action, decision["message_id"], out, instruction)
    _submit(runner, bot, f"post:{post_id}", args)
