"""Студия канала @robbox_project: кнопки, long-poll апдейтов, состояние.

Студия живёт на отдельной машине и ведёт канал своим ботом (@robboxcontentbot, токен TG_STUDIO_BOT_TOKEN).
Наружу она общается с Шифу в его личке (TG_SHIFU_CHAT_ID): присылает идеи и черновики с кнопками
и ждёт решения. Решения — callback_query от Telegram, их разбирает long-poll getUpdates.

Правила ведения канала — docs/process/telegram-channel.md, бриф студии — docs/process/channel-studio-brief.md.

Токен бота не печатается: в сообщениях об ошибках URL вырезается, как в channel_exec.py.

Команды:
  channel_studio.py poll            — разобрать накопленные апдейты (один проход getUpdates)
  channel_studio.py watch           — долгий long-poll в цикле (для контейнера студии)
  channel_studio.py state           — показать состояние (идеи, offset)
  channel_studio.py ideas <файл>    — прислать Шифу список идей с кнопками (файл — JSON, как в state.json)
  channel_studio.py post <файл>     — прислать Шифу черновик поста с кнопками «Опубликовать/Отклонить»
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional

import channel_lib as lib
import studio_state as st

REPO = Path(__file__).resolve().parents[2]
# Состояние студии — локальное (смещение getUpdates, идеи, статусы), в git не попадает:
# путь по умолчанию внутри docker/studio/, который закрыт .gitignore.
STATE_PATH = Path(os.environ.get("STUDIO_STATE", REPO / "docker" / "studio" / "state.json"))

# Действия кнопок. Ключ попадает в callback_data (максимум 64 байта у Telegram).
ACTION_TAKE = "take"  # «Берём»
ACTION_DISCUSS = "discuss"  # «Обсудить»
ACTION_SKIP = "skip"  # «Мимо»
ACTION_PUBLISH = "publish"  # «Опубликовать»
ACTION_REJECT = "reject"  # «Отклонить»
ACTION_SHORTER = "shorter"  # «Короче»
ACTION_SIMPLER = "simpler"  # «Проще»
ACTION_ANGLE = "angle"  # «Подача»
ACTION_MEDIA = "media"  # «Медиа»

# Чем переделать пост по кнопке Шифу (решение Шифу, 06.11).
REFINE_INSTRUCTIONS = {
    ACTION_SHORTER: "Сделай текст заметно короче: убери вводные и детали, остав суть.",
    ACTION_SIMPLER: "Упрости язык: короткие фразы, без терминов, как будто объясняешь другу.",
    ACTION_ANGLE: "Смени угол подачи: начни с другого факта и покажи другую сторону.",
}

IDEA_ACTIONS = (ACTION_TAKE, ACTION_DISCUSS, ACTION_SKIP)
POST_ACTIONS = (ACTION_PUBLISH, ACTION_REJECT, ACTION_SHORTER, ACTION_SIMPLER, ACTION_ANGLE, ACTION_MEDIA)

# Личка Шифу: сюда студия шлёт идеи и черновики, отсюда приходят кнопки и ответы.
SHIFU_CHAT_ID = int(os.environ.get("TG_SHIFU_CHAT_ID", "495039871"))

# callback_data: studio:<что>:<id>:<действие>
CALLBACK_RE = re.compile(r"^studio:(idea|post):([A-Za-z0-9_-]{1,32}):([a-z]{2,10})$")


# --------------------------------------------------------------------------- состояние


def find_idea(state: dict, idea_id: str) -> Optional[dict]:
    for idea in state.get("ideas", []):
        if idea.get("id") == idea_id:
            return idea
    return None


# --------------------------------------------------------------------------- Bot API


class StudioBot:
    """Тонкая обёртка над Bot API бота студии. Токен из окружения, наружу не печатается."""

    def __init__(self, token: Optional[str] = None, timeout: int = 60, session=None):
        self.token = token or os.environ.get("TG_STUDIO_BOT_TOKEN", "")
        if not self.token:
            raise RuntimeError("TG_STUDIO_BOT_TOKEN не задан: студии нужен свой бот, токен робота не подходит")
        self.timeout = timeout
        self._session = session

    def _post(self, method: str, params: dict) -> dict:
        url = f"https://api.telegram.org/bot{self.token}/{method}"
        session = self._session
        if session is None:
            # requests импортируем только когда реально идём в сеть: в оффлайн-тестах
            # модуля нет, и его отсутствие не должно ронять тест.
            try:
                import requests  # локальный импорт: сеть нужна только вживую
            except ImportError:
                return {"ok": False, "description": "ERR нет модуля requests"}
            session = requests.Session()
        last = {"ok": False, "description": "ERR no attempt"}
        # 429 — Telegram просит подождать, 409 — конфликт long-poll (например два
        # получателя апдейтов). Оба проходят сами, если подождать и повторить.
        for attempt in range(3):
            try:
                r = session.post(url, json=params, timeout=self.timeout)
                payload = r.json()
            except Exception as e:  # не печатать URL: в нём токен
                last = {"ok": False, "description": f"ERR {type(e).__name__}"}
                break
            last = {
                "ok": bool(payload.get("ok")),
                "result": payload.get("result"),
                "description": payload.get("description", ""),
                "error_code": payload.get("error_code"),
            }
            if last["ok"]:
                return last
            # 429 — Telegram просит подождать, 409 — конфликт long-poll (два получателя).
            # Код ошибки смотрим в error_code: в описании Telegram пишет «Conflict:
            # terminated by other getUpdates request», и цифры «409» там нет — по описанию
            # ретрай не срабатывал, и опрос молча возвращал «новых апдейтов нет».
            code = last.get("error_code")
            if code not in (429, 409):
                return last
            if attempt == 2:
                break
            wait = 5 * (attempt + 1)
            time.sleep(wait)
        return last

    def get_updates(self, offset: int, timeout: int = 30) -> list:
        """Апдейты с offset. Пустой offset=0 Telegram не примет — сначала сдвигаем на -1.

        Забираем и callback_query, и message: без message в allowed_updates сообщения
        Шифу (фаза обсуждения) не приходили бы вовсе, а capture_message был бы мёртвым кодом.
        """
        result = self._post(
            "getUpdates",
            {"offset": offset, "timeout": timeout, "allowed_updates": ["callback_query", "message"]},
        )
        if not result.get("ok"):
            return []
        items = result.get("result") or []
        return items if isinstance(items, list) else []

    def answer_callback(self, callback_id: str, text: str = "") -> dict:
        params = {"callback_query_id": callback_id}
        if text:
            params["text"] = text[:200]
        return self._post("answerCallbackQuery", params)

    def send_message(self, chat_id: int, text: str, keyboard: Optional[dict] = None, parse_mode: str = "HTML") -> dict:
        params = {
            "chat_id": chat_id,
            "text": text,
            "parse_mode": parse_mode,
            # Bot API 10.3: link_preview_options вместо устаревшего disable_web_page_preview
            "link_preview_options": {"is_disabled": True},
        }
        if keyboard:
            params["reply_markup"] = keyboard
        return self._post("sendMessage", params)

    def edit_message_text(
        self,
        chat_id: int,
        message_id: int,
        text: str,
        keyboard: Optional[dict] = None,
        parse_mode: Optional[str] = "HTML",
    ) -> dict:
        params = {"chat_id": chat_id, "message_id": message_id, "text": text}
        if parse_mode:
            params["parse_mode"] = parse_mode
        if keyboard:
            params["reply_markup"] = keyboard
        return self._post("editMessageText", params)


def keyboard(rows: list) -> dict:
    """Клавиатура из рядов кнопок: [[{"text": ..., "callback_data": ...}], ...]."""
    return {"inline_keyboard": rows}


# --------------------------------------------------------------------------- кнопки


def idea_keyboard(idea_id: str) -> dict:
    return keyboard(
        [
            [
                {"text": "Берём", "callback_data": f"studio:idea:{idea_id}:{ACTION_TAKE}"},
                {"text": "Обсудить", "callback_data": f"studio:idea:{idea_id}:{ACTION_DISCUSS}"},
                {"text": "Мимо", "callback_data": f"studio:idea:{idea_id}:{ACTION_SKIP}"},
            ]
        ]
    )


def post_keyboard(post_id: str) -> dict:
    """Кнопки черновика: итерация текста, медиа и решение.

    Решение Шифу (06.11): на обсуждение приходит готовый пост, дальше
    он крутится кнопками — «короче», «проще», «подача», «медиа» — и
    правится на месте, без новых сообщений.
    """
    return keyboard(
        [
            [
                {"text": "Короче", "callback_data": f"studio:post:{post_id}:{ACTION_SHORTER}"},
                {"text": "Проще", "callback_data": f"studio:post:{post_id}:{ACTION_SIMPLER}"},
            ],
            [
                {"text": "Подача", "callback_data": f"studio:post:{post_id}:{ACTION_ANGLE}"},
                {"text": "Медиа", "callback_data": f"studio:post:{post_id}:{ACTION_MEDIA}"},
            ],
            [
                {"text": "Опубликовать", "callback_data": f"studio:post:{post_id}:{ACTION_PUBLISH}"},
                {"text": "Отклонить", "callback_data": f"studio:post:{post_id}:{ACTION_REJECT}"},
            ],
        ]
    )


def _now() -> str:
    """Метка времени для журнала покрытия."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def apply_idea_action(state: dict, idea_id: str, action: str) -> str:
    """Применить решение по идее к состоянию. Возвращает текст ответа Шифу."""
    idea = find_idea(state, idea_id)
    if idea is None:
        # Идеи нет в состоянии: список обновился, пока Шифу смотрел на старое сообщение.
        # Служебный текст в чате выглядит как поломка — говорим по-человечески.
        return (
            "Этой идеи уже нет в списке — список обновился, пока ты смотрел на сообщение. "
            "Напиши одной строкой, о какой речь, и я подниму её заново."
        )
    if action == ACTION_TAKE:
        # Отклонённый пост не воскрешается: «Отклонить» → «Берём» → «Опубликовать»
        # не должен вернуть его в очередь.
        rejected = state.get("posts", {}).get(f"{idea_id}-post", {}).get("status") == "rejected"
        if rejected:
            return f"Пост идеи {idea_id} был отклонён — вернуть его в публикацию нельзя."
        idea["status"] = "agreed"
        idea["agreed_at"] = _now()
        # Что именно согласовано: без этой записи через месяц не вспомнить,
        # о чём договорились, когда идея бралась в работу.
        idea["agreement"] = (
            f"взято кнопкой «Берём» (Шифу подтвердил идею, текст ещё не согласован): "
            f"согласовано: {idea.get('title', '')}; "
            f"показать: {idea.get('show') or 'не сказано'}; "
            f"срок: {idea.get('deadline') or 'не назван'}"
        )
        # Идея взята в работу, но событие ещё не показано: факт остаётся в ожидании
        # и закроется только после публикации поста. Иначе неудавшийся пост съел бы
        # событие навсегда — это и есть «пропуск».
        return f"Идея {idea_id} взята в работу: «{idea.get('title', '')}». Собираю материал и пришлю черновик."
    if action == ACTION_DISCUSS:
        idea["status"] = "discuss"
        # Кнопка «Обсудить» должна дать живой ответ, а не только сменить статус:
        # иначе Шифу жмёт кнопку и видит пустоту, пока сам не напишет вопрос.
        return (
            f"Обсуждаем идею {idea_id}: «{idea.get('title', '')}». Сейчас разберу её по фактам "
            f"и спрошу, что показать — пишите, что поправить в подаче."
        )
    if action == ACTION_SKIP:
        idea["status"] = "rejected"
        # Отклонённое событие больше не предлагаем.
        st.mark_rejected(state, set(idea.get("fact_keys") or []), _now())
        return f"Идея {idea_id} отложена."
    return f"Неизвестное действие {action}."


def next_slot(now: Optional[datetime] = None) -> datetime:
    """Ближайший слот публикации: вторник, четверг, суббота в 11:00 GMT+1 (10:00 UTC).

    Порядок слотов задан в docs/process/telegram-channel.md и в front matter очереди
    (not_before: 10:00Z), поэтому считаем в UTC, а не в поясе машины.
    """
    now = now or datetime.now(timezone.utc)
    for days_ahead in range(8):
        day = now + timedelta(days=days_ahead)
        if day.weekday() not in (1, 3, 5):  # вт, чт, сб
            continue
        slot = day.replace(hour=10, minute=0, second=0, microsecond=0)
        if slot > now:
            return slot
    return now + timedelta(days=7)


def _queue_number(queue_dir: Path, post_id: str = "") -> int:
    """Следующий свободный номер поста в очереди (001, 002, …).

    Номер берётся с запасом относительно уже существующих файлов и не должен
    совпадать с номерами в develop: каталог очереди общий для машины студии и
    репозитория, и одинаковые имена означали бы два разных поста с одним номером.
    """
    used = set()
    for post in queue_dir.glob("*.md"):
        # Ведущие цифры имени, а не первые три символа: "1000-post.md" даёт
        # int("100") = 100, и следующий номер снова становится тысячей.
        match = re.match(r"(\d+)", post.name)
        if match:
            used.add(int(match.group(1)))
    number = max(used) + 1 if used else 1
    # Минимум 900: номера develop живут в сотнях, наши — в тысячах, чтобы
    # никогда не пересечься.
    number = max(number, 900)
    return number


def queue_post(post: dict, queue_dir: Path, now: Optional[datetime] = None) -> Optional[Path]:
    """Положить одобренный пост в очередь канала — как файл в docs/channel/queue.

    Публикатор (studio_publish.sh → channel_post.py --transport direct) берёт посты
    именно оттуда: front matter со status: approved и not_before. Без этого кнопка
    «Опубликовать» обещала бы действие, которого нет.
    """
    post_id = str(post.get("id") or "")
    body = post.get("body_html") or post.get("body") or ""
    if not body.strip():
        return None
    tags = post.get("tags") or []
    if not tags:
        tags = lib.extract_tags(body)
    if not tags:
        return None
    queue_dir.mkdir(parents=True, exist_ok=True)
    number = _queue_number(queue_dir, post_id)
    slug = re.sub(r"[^a-z0-9]+", "-", (post.get("title") or post_id).lower()).strip("-")[:40] or "post"
    path = queue_dir / f"{number:03d}-{slug}.md"
    slot = next_slot(now)
    media = post.get("media_ref") or ""
    head = [
        "---",
        "status: approved",
        f"media: {media}" if media else "media:",
        f"not_before: {slot.strftime('%Y-%m-%dT%H:%M:%SZ')}",
        "message_id:",
        f"hashtags: {' '.join(tags)}",
        "posted_at:",
        "---",
    ]
    path.write_text("\n".join(head) + "\n" + body.strip() + "\n", encoding="utf-8")
    return path


def load_run_facts() -> list:
    """Факты последней сессии — из файла, который собрала studio_facts."""
    path = Path(os.environ.get("STUDIO_FACTS", REPO / "docker" / "studio" / "run" / "facts.json"))
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    return data.get("visible_facts") or data.get("facts") or []


def request_material(bot: StudioBot, post: dict, what: str, out=print) -> str:
    """Нужного материала нет: говорим об этом Шифу и заводим issue.

    Бриф требует именно issue: координатор кладёт файл в
    docs/channel/media/ через PR, а студия сама нигде не ищет.
    """
    title = f"материал для канала: {what}"
    body = (
        f"Посту студии не хватает материала.\n\n"
        f"Что нужно: {what}\n"
        f"Пост: {post.get('id', '')}\n"
        f"Идея: {post.get('title', '')}\n\n"
        f"Материал ждём в docs/channel/media/ — координатор кладёт его через PR, "
        f"студия берёт свежие материалы из worktree на origin/develop."
    )
    created = ""
    try:
        done = subprocess.run(
            [
                "gh",
                "issue",
                "create",
                "--repo",
                "GH_REPO",
                "--title",
                title,
                "--body",
                body,
            ],
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )
        if done.returncode == 0:
            created = (done.stdout or "").strip().splitlines()[-1]
        else:
            out(f"issue не создан: {(done.stderr or '').strip()[:120]}")
    except (OSError, subprocess.TimeoutExpired):
        out("issue не создан: gh недоступен")
    notice = f"Нужен файл: {what}."
    if created:
        notice += f" Завёл issue: {created}"
    else:
        notice += " Issue создать не вышло — скажи координатору вручную."
    bot.send_message(SHIFU_CHAT_ID, notice)
    return notice


def refine_post(bot: StudioBot, state_path: Path, post_id: str, action: str, out=print) -> str:
    """Переделать пост по кнопке Шифу и обновить сообщение на месте.

    Текст правится по кнопке «короче»/«проще»/«подача», медиа — по кнопке
    «медиа». Всё это медленная работа (модель зовётся по HTTP), поэтому
    выполняется ВНЕ блокировки состояния.
    """
    state = st.load_state(state_path)
    record = (state.get("posts") or {}).get(post_id)
    if not record:
        return f"Пост {post_id} не найден — возможно, список обновился."

    facts = load_run_facts()
    if not facts:
        return f"Пост {post_id}: факты сессии недоступны, переделать не из чего."

    if action == ACTION_MEDIA:
        media = [f for f in facts if f.get("kind") == "media" and f.get("source")]
        if not media:
            return request_material(bot, record, "картинка или видео к посту", out=out)
        used = {record.get("media_ref")}
        options = [f for f in media if f.get("source") not in used] or media
        chosen = options[0]
        record["media_ref"] = chosen.get("source")
        _save_post(state_path, record)
        message = _edit_post(bot, record, out=out)
        return f"Медиа: {chosen.get('source')}. {message}"

    instruction = REFINE_INSTRUCTIONS.get(action)
    if not instruction:
        return f"Неизвестное действие {action}."

    own = [f for f in facts if f.get("key") in set(record.get("fact_keys") or [])] or facts
    idea = {
        "title": record.get("title", ""),
        "show": record.get("show") or record.get("title", ""),
        "deadline": record.get("deadline", ""),
        "media_ref": record.get("media_ref", ""),
        "fact_keys": record.get("fact_keys") or [],
    }
    produced = _produce(idea, own, instruction=instruction)
    if produced.get("error"):
        out(f"пост {post_id}: переделка не вышла: {produced.get('error')}")
        return f"Пост {post_id}: переделать не вышло ({produced.get('error')})."
    problems = produced.get("problems") or []
    if problems:
        out(f"пост {post_id}: переделка не прошла проверку: {problems}")
        return f"Пост {post_id}: новая версия не прошла проверку ({'; '.join(problems[:2])})."
    record["body_html"] = produced.get("body_html") or record.get("body_html", "")
    if produced.get("tags"):
        record["tags"] = produced["tags"]
    _save_post(state_path, record)
    message = _edit_post(bot, record, out=out)
    return f"Готово: {message}"


def _produce(idea: dict, facts: list, instruction: str = "") -> dict:
    """Позвать генератор текста (studio_text) и вернуть результат."""
    script = Path(__file__).with_name("studio_text.py")
    tmp_dir = Path(os.environ.get("STUDIO_TMP", REPO / "docker" / "studio" / "run"))
    tmp_dir.mkdir(parents=True, exist_ok=True)
    idea_path = tmp_dir / "idea-refine.json"
    idea_path.write_text(json.dumps(idea, ensure_ascii=False), encoding="utf-8")
    cmd = ["python3", str(script), "--produce", str(idea_path)]
    if instruction:
        cmd += ["--instruction", instruction]
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=400, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return {"error": "генератор не ответил"}
    try:
        return json.loads(done.stdout or "{}")
    except ValueError:
        return {"error": "генератор вернул не JSON"}


def _save_post(state_path: Path, record: dict) -> None:
    """Записать пост в состояние, не затирая остальное."""

    def _apply(current: dict) -> None:
        current.setdefault("posts", {})[record["id"]] = record

    with st.state_transaction(state_path) as current:
        _apply(current)


def _edit_post(bot: StudioBot, record: dict, out=print) -> str:
    """Обновить сообщение с постом на месте."""
    post_id = str(record.get("id") or "")
    message_id = record.get("message_id")
    body = record.get("body_html") or ""
    title = escape_html(record.get("title", ""))
    text = f"<b>Черновик {post_id}</b>" + (f": {title}" if title else "") + "\n\n" + body
    if not message_id:
        out(f"пост {post_id}: нет message_id — правлю новым сообщением")
        bot.send_message(SHIFU_CHAT_ID, text, keyboard=post_keyboard(post_id))
        return "прислал новым сообщением."
    result = bot.edit_message_text(SHIFU_CHAT_ID, message_id, text, keyboard=post_keyboard(post_id))
    if not result.get("ok"):
        out(f"пост {post_id}: правка не удалась: {result.get('description')}")
        return "правка не удалась, Telegram не принял."
    return "поправил на месте."


def apply_post_action(state: dict, post_id: str, action: str) -> str:
    """Применить решение по посту к состоянию. Возвращает текст ответа Шифу."""
    posts = state.setdefault("posts", {})
    record = posts.setdefault(post_id, {})
    if action in (ACTION_SHORTER, ACTION_SIMPLER, ACTION_ANGLE, ACTION_MEDIA):
        # Переделка идёт вне блокировки состояния: модель зовётся по HTTP.
        return {
            ACTION_SHORTER: "Короче — переделываю.",
            ACTION_SIMPLER: "Проще — переделываю.",
            ACTION_ANGLE: "Меняю подачу — переделываю.",
            ACTION_MEDIA: "Смотрю медиа.",
        }.get(action, "Переделываю.")
    if action == ACTION_PUBLISH:
        if record.get("status") == "rejected":
            return f"Пост {post_id} уже отклонён — вернуть его в публикацию можно только новой идеей."
        record["status"] = "approved"
        # Одобрение без действия — обещание в пустоту: кладём пост в очередь канала,
        # откуда его заберёт публикатор (studio_publish.sh).
        queue_dir = Path(os.environ.get("STUDIO_QUEUE_DIR", REPO / "docs" / "channel" / "queue"))
        # Повторное нажатие не должно плодить файлы: тот же пост — тот же файл.
        existing = record.get("queue_file")
        if existing:
            # Пост уже становился в очередь. Второй файл не создаём НИКОГДА:
            # иначе «Опубликовать» дважды дал бы два поста в канале. Если файл
            # исчез — сообщаем об этом, но не плодим замену молча.
            record["status"] = "approved"
            slot = next_slot()
            if Path(existing).exists():
                return (
                    f"Пост {post_id} уже одобрен и стоит в очереди — нового поста не будет. "
                    f"Слот {slot.strftime('%d.%m в %H:%M')} (по GMT+1)."
                )
            return (
                f"Пост {post_id} уже был одобрен и стоял в очереди ({existing}), "
                f"но файла на месте нет. Второй файл не создаю — публикация должна быть одна. "
                f"Нужна ручная проверка очереди."
            )
        path = queue_post(record, queue_dir)
        if path is None:
            record["status"] = "rejected"
            return f"Пост {post_id} не встал в очередь: в нём нет текста или хештегов. " f"Поправлю и пришлю заново."
        slot = next_slot()
        record["queue_file"] = str(path)
        # Событие закрыто: пост одобрен и поставлен в очередь. До этого факт висел
        # в ожидании, и его можно было предложить снова.
        st.mark_covered(state, set(record.get("fact_keys") or []), post_id, _now())
        return (
            f"Пост {post_id} одобрен — он в очереди канала, выйдет в слот "
            f"{slot.strftime('%d.%m в %H:%M')} (по GMT+1)."
        )
    if action == ACTION_REJECT:
        record["status"] = "rejected"
        st.mark_rejected(state, set(record.get("fact_keys") or []), _now())
        return f"Пост {post_id} отклонён."
    return f"Неизвестное действие {action}."


def handle_callback(state: dict, callback: dict) -> str:
    """Разобрать callback_query: обновить состояние и вернуть текст ответа Шифу.

    Нажатие обрабатывается только от Шифу: бот доступен по ссылке, и чужие нажатия
    не должны ни менять статус идеи, ни получать ответ от студии.
    """
    data = callback.get("data") or ""
    match = CALLBACK_RE.match(data)
    if not match:
        return "Непонятная кнопка."
    sender = (callback.get("from") or {}).get("id")
    # Нажатие без from — не от Шифу: пропускать нельзя, иначе анонимный вызов
    # менял бы статус идеи.
    if sender is None:
        return "Эта кнопка не для тебя."
    if int(sender) != SHIFU_CHAT_ID:
        return "Эта кнопка не для тебя."
    kind, item_id, action = match.groups()
    if kind == "idea":
        if action not in IDEA_ACTIONS:
            return "Непонятное действие."
        return apply_idea_action(state, item_id, action)
    if action not in POST_ACTIONS:
        return "Непонятное действие."
    return apply_post_action(state, item_id, action)


# --------------------------------------------------------------------------- опрос апдейтов


def poll_once(bot: StudioBot, state_path: Path, timeout: int = 30, out=print, state=None) -> int:
    """Один проход getUpdates: разобрать апдейты, сдвинуть offset, сохранить состояние.

    Быстрая часть (разбор апдейтов, смена статусов) идёт под блокировкой файла:
    иначе опрос кнопок и сессия анализа затирали бы друг друга.

    Медленная часть (ответ агента, производство поста) идёт ВНЕ блокировки.
    Агент думает десятки секунд, а держать файл состояния запертым всё это время
    нельзя: за это время опрос успел бы затереть то, что агент уже записал.
    """
    if state is None:
        with st.state_transaction(state_path) as current:
            handled, actions = _poll_cycle(bot, current, state_path, timeout=timeout, out=out)
    else:
        handled, actions = _poll_cycle(bot, state, state_path, timeout=timeout, out=out)
        st.save_state(state_path, state)
    for action, item_id, *rest in actions:
        item_action = rest[0] if rest else ""
        if action == "discuss":
            discuss_idea(bot, state_path, item_id, out=out)
        elif action == "take":
            produce_post_for_idea(bot, state_path, item_id, out=out)
        elif action == "refine":
            refine_post(bot, state_path, item_id, item_action, out=out)
        elif action == "reply":
            reply_to_shifu(bot, state_path, item_id, out=out)
    return handled


def _poll_cycle(bot: StudioBot, state: dict, state_path: Path, timeout: int = 30, out=print):
    """Разбор апдейтов внутри уже запертого состояния.

    Возвращает (сколько обработано, список отложенных действий). Действия с агентом
    выполняет вызывающий — вне блокировки.
    """
    updates = bot.get_updates(state.get("offset", 0), timeout=timeout)
    handled = 0
    actions = []
    for update in updates:
        update_id = update.get("update_id", 0)
        state["offset"] = max(state.get("offset", 0), update_id + 1)
        # Смещение двигаем СРАЗУ и пишем на диск: апдейт уже забрали у Telegram,
        # и если обработка упадёт (или агент зависнет), нажатие не потеряется —
        # Telegram его больше не отдаст.
        st.save_state(state_path, state)
        callback = update.get("callback_query")
        if callback:
            answer = handle_callback(state, callback)
            bot.answer_callback(callback.get("id", ""), answer[:200])
            message = callback.get("message") or {}
            chat_id = message.get("chat", {}).get("id")
            if chat_id:
                bot.send_message(chat_id, answer)
            out(f"обработано: {callback.get('data')} → {answer}")
            handled += 1
            # «Берём» и «Обсудить» требуют агента — это делается вне блокировки.
            data = callback.get("data") or ""
            if data.startswith("studio:idea:") and data.endswith(":take"):
                actions.append(("take", data.split(":")[2]))
            elif data.startswith("studio:idea:") and data.endswith(":discuss"):
                actions.append(("discuss", data.split(":")[2]))
            elif data.startswith("studio:post:"):
                actions.append(("refine", data.split(":")[2], data.split(":")[3]))
            continue
        message = update.get("message")
        if message:
            # Текст от Шифу — это обсуждение идеи. Сохраняем и отвечаем агентом.
            note = capture_message(state, message)
            if note:
                out(f"сообщение от Шифу: {note[:60]}")
                actions.append(("reply", note))
                handled += 1
    return handled, actions


def discuss_idea(bot: StudioBot, state_path: Path, idea_id: str, out=print) -> bool:
    """Разобрать идею с Шифу прямо по кнопке «Обсудить».

    Шифу жмёт кнопку и ждёт ответа — значит ответ должен быть сразу: сначала
    «Думаю…», потом в этом же сообщении разбор идеи от агента. Без этого кнопка
    выглядит как «ничего не произошло», и Шифу жмёт её ещё раз.

    Агент думает вне блокировки состояния; в файл пишем уже готовый ответ.
    """
    state = st.load_state(state_path)
    idea = find_idea(state, idea_id)
    if idea is None:
        out(f"идея {idea_id} не найдена — разбор пропущен")
        return False
    title = idea.get("title", "")
    facts = idea.get("facts") or []
    show = idea.get("show") or ""
    placeholder = bot.send_message(SHIFU_CHAT_ID, "Думаю над идеей…")
    message_id = (placeholder.get("result") or {}).get("message_id") if placeholder.get("ok") else None
    question = (
        f"Шифу нажал «Обсудить» на идее {idea_id}: «{title}». "
        f"Факты идеи: {facts}. Что показать: {show or 'не сказано'}. "
        f"Разбери идею по фактам и задай Шифу два уточняющих вопроса: "
        f"что важно подчеркнуть и какой материал нужен. Ответ — коротко, в Telegram."
    )
    answer = run_reply_agent(question, idea)
    if not answer:
        answer = (
            f"Идея {idea_id} — «{title}». Что показать в посте и какую мысль "
            f"подчеркнуть? Напишите в двух словах, соберу черновик."
        )
    # Ответ агента — обычный текст: parse_mode=None. С HTML-разметкой текст с «<» или «&»
    # Telegram не отправил бы вовсе (ошибка парсинга), и Шифу увидел бы пустоту.
    if message_id:
        edited = bot.edit_message_text(SHIFU_CHAT_ID, message_id, answer, parse_mode=None)
        if not edited.get("ok"):
            bot.send_message(SHIFU_CHAT_ID, answer, parse_mode=None)
    else:
        bot.send_message(SHIFU_CHAT_ID, answer, parse_mode=None)

    def _apply(current: dict) -> None:
        current.setdefault("messages", []).append({"at": None, "role": "studio", "text": answer[:2000]})

    with st.state_transaction(state_path) as current:
        _apply(current)
    out(f"идея {idea_id}: разбор отправлен")
    return True


def capture_message(state: dict, message: dict) -> str:
    """Сохранить текстовое сообщение Шифу в состояние (фаза обсуждения)."""
    text = (message.get("text") or "").strip()
    if not text:
        return ""
    chat = message.get("chat") or {}
    if chat.get("id") != SHIFU_CHAT_ID:
        return ""
    notes = state.setdefault("messages", [])
    notes.append({"at": message.get("date"), "text": text[:2000], "answered": False})
    del notes[:-20]  # последние 20 сообщений
    return text


def reply_to_shifu(bot: StudioBot, state_path: Path, text: str, out=print) -> bool:
    """Ответить Шифу на его вопрос: агент думает, сообщение редактируется ответом.

    Так видно, что студия работает: сначала «Думаю…», потом в этом же сообщении ответ.
    """
    state = st.load_state(state_path)
    placeholder = bot.send_message(SHIFU_CHAT_ID, "Думаю над твоим вопросом…")
    message_id = (placeholder.get("result") or {}).get("message_id") if placeholder.get("ok") else None
    idea = discussed_idea(state)
    answer = run_reply_agent(text, idea)
    if not answer:
        answer = "Не смог ответить сейчас — идея осталась на обсуждении, спрошу позже."
    if message_id:
        bot.edit_message_text(SHIFU_CHAT_ID, message_id, answer, parse_mode=None)
    else:
        bot.send_message(SHIFU_CHAT_ID, answer, parse_mode=None)

    def _apply(current: dict) -> None:
        # Ответ агента тоже в историю: следующий вопрос должен видеть, что уже обсудили.
        current.setdefault("messages", []).append({"at": None, "role": "studio", "text": answer[:2000]})
        for note in current.get("messages", []):
            if note.get("text") == text and note.get("role", "shifu") == "shifu" and not note.get("answered"):
                note["answered"] = True

    with st.state_transaction(state_path) as current:
        _apply(current)
    out(f"ответ отправлен: {answer[:60]}")
    return True


def discussed_idea(state: dict) -> dict:
    """Идея, которая сейчас на обсуждении: последняя со статусом discuss."""
    for idea in reversed(state.get("ideas") or []):
        if idea.get("status") == "discuss":
            return idea
    return {}


def run_reply_agent(text: str, idea: dict) -> str:
    """Позвать агента профиля studio. Ответ приходит из его же слов."""
    script = Path(__file__).with_name("studio_reply.py")
    cmd = ["python3", str(script), "--message", text]
    if idea.get("id"):
        cmd += ["--idea-id", idea["id"]]
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=400)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return (done.stdout or "").strip()


def watch(bot: StudioBot, state_path: Path, timeout: int = 30, once: bool = False, out=print) -> int:
    """Цикл long-poll. once=True — один проход (для cron)."""
    total = 0
    while True:
        total += poll_once(bot, state_path, timeout=timeout, out=out)
        if once:
            return total
        time.sleep(1)


# --------------------------------------------------------------------------- отправка идей и постов


def send_ideas(bot: StudioBot, ideas: list, chat_id: int, out=print) -> int:
    """Прислать Шифу список идей: по кнопке на каждую. Только идеи, не готовые посты.

    По брифу по каждой идее: о чём пост; факты и источники; что показать; примерный срок.
    """
    if not ideas:
        out("идей нет — Шифу ничего не шлю")
        return 0
    sent = 0
    for idea in ideas:
        idea_id = str(idea.get("id") or "")
        if not idea_id:
            continue
        title = idea.get("title", "")
        facts = idea.get("facts") or []
        show = idea.get("show") or ""
        deadline = idea.get("deadline") or ""
        lines = [f"<b>Идея {idea_id}</b>: {title}", ""]
        if facts:
            lines.append("Факты и источники:")
            lines.extend(f"— {f}" for f in facts)
            lines.append("")
        if show:
            lines.append(f"Что показать: {show}")
        if deadline:
            lines.append(f"Когда: {deadline}")
        lines.append("")
        lines.append("Берём, обсудить или мимо?")
        result = bot.send_message(chat_id, "\n".join(lines), keyboard=idea_keyboard(idea_id))
        if result.get("ok"):
            sent += 1
        else:
            out(f"идея {idea_id} не отправлена: {result.get('description')}")
    out(f"идей отправлено: {sent}")
    return sent


def produce_post_for_idea(bot: StudioBot, state_path: Path, idea_id: str, out=print) -> bool:
    """Произвести пост по взятой идее и прислать черновик с кнопками.

    Порядок брифа: идея согласована → и только теперь пишется текст.
    """
    state = st.load_state(state_path)
    idea = find_idea(state, idea_id)
    if idea is None:
        out(f"идея {idea_id} не найдена — производство пропущено")
    # Текст пишется только по согласованной идее.
    if idea.get("status") != "agreed":
        out(f"идея {idea_id} не согласована (статус {idea.get('status')!r}) — производство пропущено")
        return False
        return False
    script = Path(__file__).with_name("studio_text.py")
    facts_path = Path(os.environ.get("STUDIO_FACTS", REPO / "docker" / "studio" / "run" / "facts.json"))
    idea_path = Path(os.environ.get("STUDIO_TMP", REPO / "docker" / "studio" / "run")) / f"idea-{idea_id}.json"
    try:
        idea_path.parent.mkdir(parents=True, exist_ok=True)
        idea_path.write_text(json.dumps(idea, ensure_ascii=False), encoding="utf-8")
        done = subprocess.run(
            ["python3", str(script), "--facts", str(facts_path), "--produce", str(idea_path)],
            capture_output=True,
            text=True,
            timeout=400,
        )
    except (OSError, subprocess.TimeoutExpired):
        out(f"идея {idea_id}: производство не запустилось")
        return False
    try:
        post = json.loads(done.stdout or "{}")
    except ValueError:
        out(f"идея {idea_id}: производство вернуло не JSON")
        return False
    if post.get("error") or not post.get("body_html"):
        out(f"идея {idea_id}: пост не получился: {post.get('error') or 'пустой текст'}")
        return False
    if not post.get("ok"):
        out(f"идея {idea_id}: пост не прошёл проверку: {post.get('problems')}")
        return False
    post_id = f"{idea_id}-post"
    # Пост уже мог быть одобрен и стоять в очереди. Перезаписывать запись
    # нельзя: теряется queue_file, и «Опубликовать» создаст второй файл.
    existing = state.get("posts", {}).get(post_id) or {}
    if existing.get("queue_file"):
        out(f"идея {idea_id}: пост уже в очереди ({existing['queue_file']}) — второй не делаю")
        return False
    if existing.get("status") == "rejected":
        out(f"идея {idea_id}: пост отклонён — заново не собираю")
        return False
    record = {
        "id": post_id,
        "status": "proposed",
        "title": idea.get("title", ""),
        "body_html": post["body_html"],
        "tags": post.get("tags") or [],
        "media_ref": idea.get("media_ref", ""),
        "fact_keys": idea.get("fact_keys") or [],
    }
    for key in ("queue_file", "message_id"):
        if existing.get(key):
            record[key] = existing[key]

    def _apply(current: dict) -> None:
        current.setdefault("posts", {})[post_id] = record
        target = find_idea(current, idea_id)
        if target is not None:
            target["post_body"] = post["body_html"]

    with st.state_transaction(state_path) as current:
        _apply(current)
    sent = send_post(bot, record, SHIFU_CHAT_ID)
    if not sent:
        # Честный отчёт: черновик не ушёл — Шифу об этом должен знать.
        out(f"идея {idea_id}: пост собран, но черновик НЕ отправлен — Telegram не принял")
        return False
    out(f"идея {idea_id}: пост готов и отправлен с кнопками")
    return True


def escape_html(text: str) -> str:
    """Экранировать текст для Telegram HTML.

    Заголовок черновика приходит от модели и может содержать «<» или «&».
    Без экранирования Telegram отклонил бы сообщение целиком — Шифу увидел бы
    пустоту вместо поста.
    """
    return (text or "").replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def send_post(bot: StudioBot, post: dict, chat_id: int, out=print) -> int:
    """Прислать Шифу черновик поста с кнопками «Опубликовать/Отклонить»."""
    post_id = str(post.get("id") or "")
    if not post_id:
        out("у поста нет id")
        return 0
    body = post.get("body_html") or post.get("body") or ""
    title = escape_html(post.get("title", ""))
    text = f"<b>Черновик {post_id}</b>" + (f": {title}" if title else "") + "\n\n" + body
    result = bot.send_message(chat_id, text, keyboard=post_keyboard(post_id))
    if not result.get("ok"):
        out(f"черновик не отправлен: {result.get('description')}")
        return 0
    # message_id нужен для правки на месте: по кнопке «короче» текст меняется
    # в том же сообщении, а не приходит новым.
    message_id = (result.get("result") or {}).get("message_id")
    if message_id:
        post["message_id"] = message_id
    out(f"черновик {post_id} отправлен")
    return message_id or 1


def filming_requests(idea: dict) -> str:
    """Идея требует съёмки — по ней нужна заявка, а не только пост.

    По брифу: если показать нечего, студия просит съёмку. Тут — тот случай, когда
    модель прямо пишет, что материала нет.
    """
    show = (idea.get("show") or "").lower()
    media = (idea.get("media_ref") or "").strip()
    if media:
        return ""
    for marker in ("съёмк", "снимк", "запис", "видео нужно", "нужен ролик", "нужен гиф"):
        if marker in show:
            return idea.get("title", "")
    return ""


def register_ideas(state_path: Path, ideas: list) -> int:
    """Записать отправленные идеи в состояние.

    Без этого кнопка «Берём» не находит идею: она присылалась Шифу, но в state.json её не было.
    Повторная отправка той же идеи статус не затирает — идея уже может быть взята в работу.

    Факты идеи встают в ожидание: событие считается закрытым только после публикации
    поста, иначе неудавшийся пост съел бы событие навсегда.
    """
    added = 0
    with st.state_transaction(state_path) as state:
        known = state.setdefault("ideas", [])
        for idea in ideas:
            idea_id = str(idea.get("id") or "")
            if not idea_id:
                continue
            existing = find_idea(state, idea_id)
            if existing is None:
                known.append(
                    {
                        "id": idea_id,
                        "title": idea.get("title", ""),
                        "status": "proposed",
                        "facts": idea.get("facts") or [],
                        "fact_keys": idea.get("fact_keys") or [],
                        "show": idea.get("show", ""),
                        "deadline": idea.get("deadline", ""),
                        "media_ref": idea.get("media_ref", ""),
                    }
                )
                added += 1
            else:
                existing.setdefault("title", idea.get("title", ""))
                existing.setdefault("show", idea.get("show", ""))
                existing.setdefault("deadline", idea.get("deadline", ""))
                existing.setdefault("media_ref", idea.get("media_ref", ""))
                if not existing.get("facts"):
                    existing["facts"] = idea.get("facts") or []
                if not existing.get("fact_keys"):
                    existing["fact_keys"] = idea.get("fact_keys") or []
            st.save_pending(state, idea.get("fact_dicts") or [])
            # Заявки на съёмку копятся отдельным списком: их видно и в runbook,
            # и в отчёте сессии — иначе про «нужна съёмка» забывают.
            request = filming_requests(idea)
            if request:
                requests = state.setdefault("filming_requests", [])
                if request not in requests:
                    requests.append(request)
    return added


def register_post(state_path: Path, post: dict) -> None:
    """Записать отправленный черновик в состояние — чтобы кнопка «Опубликовать» его нашла."""
    post_id = str(post.get("id") or "")
    if not post_id:
        return
    state = st.load_state(state_path)
    posts = state.setdefault("posts", {})
    record = posts.setdefault(post_id, {})
    record.setdefault("id", post_id)
    record.setdefault("status", "proposed")
    record.setdefault("title", post.get("title", ""))
    record.setdefault("body_html", post.get("body_html", "") or post.get("body", ""))
    for key in ("tags", "media_ref", "fact_keys", "show", "message_id"):
        if post.get(key):
            record[key] = post[key]
    st.save_state(state_path, state)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", type=Path, default=STATE_PATH, help="файл состояния студии (state.json)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_poll = sub.add_parser("poll", help="один проход getUpdates (для cron)")
    p_poll.add_argument("--timeout", type=int, default=30, help="таймаут long-poll, с")
    p_poll.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    p_watch = sub.add_parser("watch", help="долгий long-poll в цикле (для контейнера студии)")
    p_watch.add_argument("--timeout", type=int, default=30, help="таймаут long-poll, с")
    p_watch.add_argument("--once", action="store_true", help="один проход и выход")
    p_watch.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    sub.add_parser("state", help="показать состояние студии")

    p_ideas = sub.add_parser("ideas", help="прислать Шифу список идей с кнопками")
    p_ideas.add_argument("file", type=Path, help="JSON со списком идей")
    p_ideas.add_argument("--chat-id", type=int, default=int(os.environ.get("TG_SHIFU_CHAT_ID", "495039871")))
    p_ideas.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    p_post = sub.add_parser("post", help="прислать Шифу черновик поста с кнопками")
    p_post.add_argument("file", type=Path, help="JSON с черновиком")
    p_post.add_argument("--chat-id", type=int, default=int(os.environ.get("TG_SHIFU_CHAT_ID", "495039871")))
    p_post.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    args = ap.parse_args()
    state_path = args.state

    if args.cmd == "state":
        print(json.dumps(st.load_state(state_path), ensure_ascii=False, indent=2))
        return 0

    bot = StudioBot(args.token)

    if args.cmd == "poll":
        # Опрос держит блокировку состояния на весь цикл: иначе он и сессия
        # анализа читали бы одно состояние и затирали друг друга.
        return 0 if poll_once(bot, state_path, timeout=args.timeout) >= 0 else 1
    if args.cmd == "watch":
        watch(bot, state_path, timeout=args.timeout, once=args.once)
        return 0
    if args.cmd == "ideas":
        data = json.loads(args.file.read_text(encoding="utf-8"))
        ideas = data.get("ideas", data) if isinstance(data, dict) else data
        # Регистрируем ДО отправки: иначе Шифу может нажать кнопку в первые миллисекунды,
        # а апдейт придёт раньше, чем идея появится в состоянии, — и кнопка скажет
        # «идея не найдена». Лишняя запись при неудачной отправке безвредна.
        added = register_ideas(state_path, ideas)
        print(f"идей записано в состояние: {added}")
        sent = send_ideas(bot, ideas, args.chat_id)
        return 0 if sent else 1
    if args.cmd == "post":
        data = json.loads(args.file.read_text(encoding="utf-8"))
        post = data.get("post", data) if isinstance(data, dict) else data
        register_post(state_path, post)
        sent = send_post(bot, post, args.chat_id)
        if sent:
            # message_id появился только после отправки — без него правка
            # «короче» не найдёт сообщение и пошлёт новое.
            register_post(state_path, post)
        return 0 if sent else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
