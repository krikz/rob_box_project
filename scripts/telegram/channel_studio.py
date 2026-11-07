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
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional

import channel_lib as lib
import studio_media as smedia
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
ACTION_ANIMATE = "animate"  # «Анимация»: GIF ≤ 2 МБ (studio_motion, #4513)
ACTION_VIDEO = "video"  # «Видео»: MP4 ≤ 50 МБ по просьбе Шифу или кнопке-предложению моста (#4513)
ACTION_FREEFORM = "edit"  # свободный текст Шифу к черновику (кнопкой — только «к какому черновику?»)
ACTION_FILE = "file"  # файл от Шифу в черновик (кнопкой — только «к какому черновику?»)
ACTION_POST = "post"  # «Пришли пост»: прислать собранный черновик (текст + медиа) в чат

# Чем переделать пост по кнопке Шифу (решение Шифу, 06.11).
REFINE_INSTRUCTIONS = {
    ACTION_SHORTER: "Сделай текст заметно короче: убери вводные и детали, остав суть.",
    ACTION_SIMPLER: "Упрости язык: короткие фразы, без терминов, как будто объясняешь другу.",
    ACTION_ANGLE: "Смени угол подачи: начни с другого факта и покажи другую сторону.",
}

IDEA_ACTIONS = (ACTION_TAKE, ACTION_DISCUSS, ACTION_SKIP)
REFINE_ACTIONS = (
    ACTION_POST,
    ACTION_SHORTER,
    ACTION_SIMPLER,
    ACTION_ANGLE,
    ACTION_MEDIA,
    ACTION_ANIMATE,
    ACTION_VIDEO,
    ACTION_FREEFORM,
    ACTION_FILE,
)
POST_ACTIONS = (ACTION_PUBLISH, ACTION_REJECT) + REFINE_ACTIONS

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

    def _send(self, session, url: str, params: dict, files: Optional[dict], timeout: int):
        """Один HTTP-вызов. С файлами — multipart: вложенные объекты Bot API уходят JSON-строкой."""
        if not files:
            return session.post(url, json=params, timeout=timeout)
        data = {
            k: (json.dumps(v, ensure_ascii=False) if isinstance(v, (bool, dict, list)) else v)
            for k, v in params.items()
        }
        handles = {field: open(path, "rb") for field, path in files.items()}
        try:
            return session.post(url, data=data, files=handles, timeout=timeout)
        finally:
            for handle in handles.values():
                handle.close()

    def _post(self, method: str, params: dict, files: Optional[dict] = None, timeout: Optional[int] = None) -> dict:
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
                r = self._send(session, url, params, files, timeout or self.timeout)
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
            # Bot API: ResponseParameters.retry_after — сколько секунд ждать при flood control.
            retry_after = (payload.get("parameters") or {}).get("retry_after")
            wait = int(retry_after) if isinstance(retry_after, (int, float)) and retry_after > 0 else 5 * (attempt + 1)
            time.sleep(min(wait, 60))
        return last

    def get_updates(self, offset: int, timeout: int = 30) -> list:
        """Апдейты с offset. Пустой offset=0 Telegram не примет — сначала сдвигаем на -1.

        Забираем и callback_query, и message: без message в allowed_updates сообщения
        Шифу (фаза обсуждения) не приходили бы вовсе, а capture_message был бы мёртвым кодом.
        """
        result = self._post(
            "getUpdates",
            {"offset": offset, "timeout": timeout, "allowed_updates": ["callback_query", "message"]},
            # HTTP-таймаут длиннее long-poll: иначе requests рвёт ожидание раньше Telegram.
            timeout=max(self.timeout, timeout + 15),
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

    def send_message(
        self,
        chat_id: int,
        text: str,
        keyboard: Optional[dict] = None,
        parse_mode: Optional[str] = "HTML",
        reply_to: Optional[int] = None,
    ) -> dict:
        params = {
            "chat_id": chat_id,
            "text": text,
            # Bot API 10.3: link_preview_options вместо устаревшего disable_web_page_preview
            "link_preview_options": {"is_disabled": True},
        }
        if parse_mode:
            params["parse_mode"] = parse_mode
        if keyboard:
            params["reply_markup"] = keyboard
        if reply_to:
            # reply_parameters: индикатор привязан к черновику; если черновик удалён — всё равно отправить.
            params["reply_parameters"] = {"message_id": reply_to, "allow_sending_without_reply": True}
        return self._post("sendMessage", params)

    def send_chat_action(self, chat_id: int, action: str) -> dict:
        """«печатает…» / «отправляет фото…»: статус живёт ≤ 5 с (Bot API, sendChatAction)."""
        return self._post("sendChatAction", {"chat_id": chat_id, "action": action}, timeout=10)

    def send_media(
        self,
        chat_id: int,
        path: Path,
        caption: str = "",
        keyboard: Optional[dict] = None,
        above: bool = False,
    ) -> dict:
        """Фото, GIF или видео с подписью (HTML) и кнопками — одним сообщением."""
        kind = smedia.kind_of(path)
        if kind is None:
            return {"ok": False, "description": f"тип {Path(path).suffix} не поддержан"}
        method, field, _action = kind
        params = {"chat_id": chat_id, "caption": caption, "parse_mode": "HTML"}
        if above:
            params["show_caption_above_media"] = True
        if keyboard:
            params["reply_markup"] = keyboard
        return self._post(method, params, files={field: str(path)}, timeout=max(self.timeout, 180))

    def edit_message_media(
        self, chat_id: int, message_id: int, path: Path, caption: str = "", keyboard: Optional[dict] = None
    ) -> dict:
        """Заменить медиа (и подпись) в том же сообщении: editMessageMedia + attach://."""
        kind = smedia.kind_of(path)
        if kind is None:
            return {"ok": False, "description": f"тип {Path(path).suffix} не поддержан"}
        _method, field, _action = kind
        input_media = {"type": field, "media": "attach://file", "caption": caption, "parse_mode": "HTML"}
        params = {"chat_id": chat_id, "message_id": message_id, "media": input_media}
        if keyboard:
            params["reply_markup"] = keyboard
        return self._post("editMessageMedia", params, files={"file": str(path)}, timeout=max(self.timeout, 180))

    def get_file(self, file_id: str) -> dict:
        """getFile: путь файла на серверах Telegram (ботам — до 20 МБ)."""
        return self._post("getFile", {"file_id": file_id})

    def download_file(self, file_path: str, dest: Path) -> str:
        """Скачать файл по пути из getFile. Пустая строка — успех. URL с токеном наружу не печатается."""
        try:
            import requests  # локальный импорт: сеть нужна только вживую
        except ImportError:
            return "нет модуля requests"
        url = f"https://api.telegram.org/file/bot{self.token}/{file_path}"
        part = Path(dest).with_suffix(Path(dest).suffix + ".part")
        try:
            with (self._session or requests).get(url, stream=True, timeout=max(self.timeout, 120)) as response:
                if response.status_code != 200:
                    return f"HTTP {response.status_code}"
                with open(part, "wb") as handle:
                    for chunk in response.iter_content(1 << 16):
                        handle.write(chunk)
        except Exception as e:  # noqa: BLE001 — не печатать URL: в нём токен
            return f"ERR {type(e).__name__}"
        part.replace(dest)
        return ""

    def edit_message_caption(
        self, chat_id: int, message_id: int, caption: str, keyboard: Optional[dict] = None
    ) -> dict:
        params = {"chat_id": chat_id, "message_id": message_id, "caption": caption, "parse_mode": "HTML"}
        if keyboard:
            params["reply_markup"] = keyboard
        return self._post("editMessageCaption", params)

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
                {"text": "Анимация", "callback_data": f"studio:post:{post_id}:{ACTION_ANIMATE}"},
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
    # Инфографика студии живёт в docker/studio/run/media (media_path), готовое медиа — в репо (media_ref).
    media = post.get("media_ref") or post.get("media_path") or ""
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
    """Факты последней сессии — из файла, который собрала studio_facts (реализация в studio_jobs)."""
    import studio_jobs as jobs

    return jobs.load_run_facts()


def apply_post_action(state: dict, post_id: str, action: str) -> str:
    """Применить решение по посту к состоянию. Возвращает текст ответа Шифу."""
    posts = state.setdefault("posts", {})
    if action in REFINE_ACTIONS:
        # Переделка идёт вне блокировки состояния: модель зовётся по HTTP. Запись поста
        # здесь НЕ создаём: пустая запись {} делала «черновик не найден» молчаливым (06.11).
        return {
            ACTION_SHORTER: "Короче — переделываю.",
            ACTION_SIMPLER: "Проще — переделываю.",
            ACTION_ANGLE: "Меняю подачу — переделываю.",
            ACTION_MEDIA: "Меняю медиа.",
            ACTION_ANIMATE: "Делаю анимацию.",
            ACTION_VIDEO: "Собираю видео.",
        }.get(action, "Переделываю.")
    record = posts.setdefault(post_id, {})
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


class JobRunner:
    """Тяжёлая работа (MiniMax, рендер, агент) — вне опроса.

    threaded=True — каждая работа в своём потоке: опрос сразу возвращается к getUpdates,
    и следующее нажатие подтверждается за доли секунды (просьба Шифу 06.11: «нажал —
    а оно уходит курить»). threaded=False — синхронно (разовый `poll` и тесты).

    Одна работа на черновик за раз: второе нажатие, пока идёт первое, получает честный
    ответ «уже делаю», а не вторую параллельную правку того же сообщения.
    """

    def __init__(self, threaded: bool = True, out=print):
        self.threaded = threaded
        self.out = out
        self._busy: set = set()
        self._guard = threading.Lock()
        self.threads: list = []

    def claim(self, key: str) -> bool:
        with self._guard:
            if key in self._busy:
                return False
            self._busy.add(key)
            return True

    def busy(self, key: str) -> bool:
        with self._guard:
            return key in self._busy

    def submit(self, key: str, fn, *args) -> None:
        """Запустить работу. key уже занят через claim (или занимается здесь)."""
        with self._guard:
            self._busy.add(key)
        if not self.threaded:
            self._run(key, fn, args)
            return
        thread = threading.Thread(target=self._run, args=(key, fn, args), name=f"studio-{key}", daemon=True)
        self.threads = [t for t in self.threads if t.is_alive()] + [thread]
        thread.start()

    def _run(self, key: str, fn, args: tuple) -> None:
        try:
            fn(*args)
        except Exception as exc:  # noqa: BLE001 — работа упала, опрос жить должен
            self.out(f"работа {key} упала: {type(exc).__name__}: {exc}")
        finally:
            with self._guard:
                self._busy.discard(key)

    def join(self, timeout: Optional[float] = None) -> None:
        for thread in list(self.threads):
            thread.join(timeout)


def _with_state(state_path: Path, state: Optional[dict], fn):
    """Короткая запись состояния под замком. Long-poll под замком НЕ держим."""
    if state is not None:
        result = fn(state)
        st.save_state(state_path, state)
        return result
    with st.state_transaction(state_path) as current:
        return fn(current)


def _advance_offset(state: dict, update_id: int) -> None:
    state["offset"] = max(state.get("offset", 0), update_id + 1)


def poll_once(
    bot: StudioBot,
    state_path: Path,
    timeout: int = 30,
    out=print,
    state=None,
    runner: Optional[JobRunner] = None,
) -> int:
    """Один проход getUpdates.

    Порядок (разбор 06.11, кнопка «Медиа» молчала):
      1. long-poll идёт БЕЗ замка состояния. Раньше state_transaction держалась весь
         long-poll и в finally писала устаревшее состояние поверх — так из state.json
         пропал черновик, который сессия записала в эти 50 секунд;
      2. на нажатие — сразу answerCallbackQuery («Делаю…»): без него у кнопки крутится
         индикатор (Bot API, CallbackQuery: «clients will display a progress bar until
         you call answerCallbackQuery»);
      3. состояние меняется короткой транзакцией: смещение и решение кнопки;
      4. тяжёлая работа — в runner (поток), опрос сразу идёт дальше.
    """
    runner = runner or JobRunner(threaded=False, out=out)
    offset = (state if state is not None else st.load_state(state_path)).get("offset", 0)
    updates = bot.get_updates(offset, timeout=timeout)
    handled = 0
    for update in updates:
        update_id = update.get("update_id", 0)
        callback = update.get("callback_query")
        if callback:
            handled += _on_callback(bot, state_path, state, update_id, callback, runner, out)
            continue
        import studio_messages

        handled += studio_messages.route(bot, state_path, state, update_id, update.get("message"), runner, out)
    return handled


def active_draft(state: dict, reply_to: Optional[int] = None) -> str:
    """Черновик, к которому относится сообщение Шифу. Решает код, не модель (ADR-0148).

    1. Ответ (reply) на сообщение черновика — правка именно этого черновика.
    2. Иначе — последний черновик на обсуждении (status proposed, есть message_id): после
       решения Шифу 06.11 обсуждение идёт над готовым постом, свободный текст — правка к нему.
    Черновика нет — пустая строка, сообщение уходит в обычный ответ модели.
    """
    drafts = [
        (post_id, record)
        for post_id, record in (state.get("posts") or {}).items()
        if record.get("status") == "proposed" and record.get("message_id") and record.get("body_html")
    ]
    if reply_to:
        for post_id, record in drafts:
            if record.get("message_id") == reply_to:
                return post_id
    if not drafts:
        return ""
    return max(drafts, key=lambda item: item[1].get("message_id") or 0)[0]


def _parse_callback(callback: dict) -> tuple:
    """(kind, item_id, action, от Шифу ли) или (None, …), если кнопка не наша."""
    match = CALLBACK_RE.match(callback.get("data") or "")
    sender = (callback.get("from") or {}).get("id")
    from_shifu = sender is not None and int(sender) == SHIFU_CHAT_ID
    if not match:
        return None, "", "", from_shifu
    kind, item_id, action = match.groups()
    return kind, item_id, action, from_shifu


def _on_callback(bot, state_path, state, update_id, callback, runner, out) -> int:
    kind, item_id, action, from_shifu = _parse_callback(callback)
    callback_id = callback.get("id", "")
    message = callback.get("message") or {}
    if kind == "post" and action in REFINE_ACTIONS and from_shifu:
        key = f"post:{item_id}"
        if not runner.claim(key):
            bot.answer_callback(callback_id, "Уже делаю прошлую правку этого черновика — дождись её.")
            _with_state(state_path, state, lambda cur: _advance_offset(cur, update_id))
            out(f"занято: {callback.get('data')}")
            return 1
        # Подтверждение — первым делом, до любой работы: кнопка перестаёт крутиться сразу.
        received = time.monotonic()
        answered = bot.answer_callback(callback_id, "Делаю…")
        answer_ms = int((time.monotonic() - received) * 1000)
        _with_state(state_path, state, lambda cur: _advance_offset(cur, update_id))
        status = "ok" if (answered or {}).get("ok") else f"ошибка: {(answered or {}).get('description')}"
        out(f"обработано: {callback.get('data')} → в работу; «Делаю…» за {answer_ms} мс ({status})")
        pending = _with_state(state_path, state, lambda cur: _take_pending(cur, action))
        if not pending and action in (ACTION_VIDEO, ACTION_ANIMATE):
            # Кнопку нажали без кнопки-предложения — берём свежую просьбу
            # Шифу про ролик, иначе сборка уйдёт по умолчанию (07.11).
            pending = _with_state(state_path, state, lambda cur: _take_wish(cur))
        if action == ACTION_FILE:
            import studio_inbox

            if not pending.get("media"):
                runner.submit(key, _say, bot, "Файл, о котором спрашивал, уже не найти — пришли его ещё раз.")
                return 1
            runner.submit(
                key, studio_inbox.attach_job, bot, state_path, item_id, pending["media"],
                pending.get("text", ""), pending.get("message_id"), out,
            )  # fmt: skip
            return 1
        import studio_jobs as jobs

        runner.submit(
            key,
            jobs.refine_job,
            bot,
            state_path,
            item_id,
            action,
            message.get("message_id"),
            out,
            pending.get("text", ""),
        )
        return 1

    def _decide(cur: dict) -> str:
        _advance_offset(cur, update_id)
        return handle_callback(cur, callback)

    answer = _with_state(state_path, state, _decide)
    bot.answer_callback(callback_id, answer[:200])
    chat_id = (message.get("chat") or {}).get("id")
    if chat_id and from_shifu:
        bot.send_message(chat_id, answer, parse_mode=None)
    out(f"обработано: {callback.get('data')} → {answer}")
    if kind == "idea" and from_shifu and action == ACTION_TAKE:
        runner.submit(f"idea:{item_id}", produce_post_for_idea, bot, state_path, item_id, out)
    elif kind == "idea" and from_shifu and action == ACTION_DISCUSS:
        runner.submit(f"idea:{item_id}", discuss_idea, bot, state_path, item_id, out)
    return 1


# Просьба про ролик: вертикальным, сверху-снизу, с музыкой, ролик, анимация.
VIDEO_WISH_RE = re.compile(r"видео|ролик|анимац|музык|трек|звук|вертикал|сверху|снизу", re.IGNORECASE)
WISH_TTL = 30 * 60  # просьба про ролик помнится полчаса


def _take_wish(state: dict) -> dict:
    """Свежая просьба Шифу про ролик — если кнопку нажали без предложения."""
    wish = state.get("wish") or {}
    if not wish.get("text"):
        return {}
    if time.time() - float(wish.get("at") or 0) > WISH_TTL:
        state.pop("wish", None)
        return {}
    state.pop("wish", None)
    return {"text": wish["text"]}


def _take_pending(state: dict, action: str) -> dict:
    """Просьба, которую студия переспросила кнопками «к какому черновику?», — забрать один раз."""
    pending = state.get("pending_request") or {}
    if pending.get("action") != action:
        return {}
    state.pop("pending_request", None)
    return pending


def _say(bot, text: str) -> None:
    bot.send_message(SHIFU_CHAT_ID, text, parse_mode=None)


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
    """Сохранить сообщение Шифу в состояние (фаза обсуждения): текст или подпись медиа.

    У медиа-сообщений Bot API кладёт текст в ``caption``, а не в ``text`` (#4513: видео с подписью
    студия молча пропускала).
    """
    text = (message.get("text") or message.get("caption") or "").strip()
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
    answer, offer = split_reply(run_reply_agent(text, idea))
    if not answer:
        answer = "Не смог ответить сейчас — модель не дала ответа, который прошёл бы проверку."
    draft = active_draft(state)
    buttons = None
    if draft and offer:
        # Модель предлагает действие полем offer, кнопку строит код (#4513).
        # Просьбу Шифу запоминаем: без неё кнопка собрала бы ролик по
        # умолчанию, а он просил «вертикальным, до сверху, после снизу»
        # (Шифу, 07.11: «чот оно меня пока не понимает»).
        buttons = keyboard([[{"text": OFFER_LABELS[offer], "callback_data": f"studio:post:{draft}:{offer}"}]])
        answer += f"\n\nКнопка ниже сделает это с черновиком {draft}."
    elif draft:
        # Подсказку строит код, а не модель: как править черновик — знает студия, а не LLM.
        answer += (
            f"\n\nПросьбу к черновику {draft} («короче», «анимацию», «ролик до/после», «убери …») пиши прямо сюда — "
            "студия сделает её на месте."
        )
    elif offer:
        answer += "\n\nЧерновика на обсуждении нет — делать это не над чем."
    if message_id:
        bot.edit_message_text(SHIFU_CHAT_ID, message_id, answer, keyboard=buttons, parse_mode=None)
    else:
        bot.send_message(SHIFU_CHAT_ID, answer, keyboard=buttons, parse_mode=None)

    def _apply(current: dict) -> None:
        # Ответ агента тоже в историю: следующий вопрос должен видеть ход.
        current.setdefault("messages", []).append({"at": None, "role": "studio", "text": answer[:2000]})
        for note in current.get("messages", []):
            if note.get("text") == text and note.get("role", "shifu") == "shifu" and not note.get("answered"):
                note["answered"] = True
        if offer and draft:
            # Просьбу, ради которой предложена кнопка, надо помнить: без неё
            # нажатие собрало бы ролик по умолчанию, а Шифу просил «вертикальным,
            # до сверху, после снизу» (07.11: «чот оно меня пока не понимает»).
            current["pending_request"] = {"action": offer, "text": text, "at": time.time()}
        if VIDEO_WISH_RE.search(text or ""):
            # Свежая просьба про ролик: кнопка на самом черновике тоже должна
            # её учитывать.
            current["wish"] = {"text": text, "at": time.time()}

    with st.state_transaction(state_path) as current:
        _apply(current)
    out(f"ответ отправлен: {answer[:60]}")
    return True


OFFER_LABELS = {
    ACTION_POST: "Пришли пост",
    ACTION_SHORTER: "Короче",
    ACTION_SIMPLER: "Проще",
    ACTION_ANGLE: "Подача",
    ACTION_MEDIA: "Медиа",
    ACTION_ANIMATE: "Анимация",
    ACTION_VIDEO: "Видео",
}


def split_reply(raw: str) -> tuple:
    """Вывод studio_reply.py --json → (ответ, предложение). Простой текст — ответ без предложения."""
    raw = (raw or "").strip()
    if raw.startswith("{"):
        try:
            data = json.loads(raw)
        except ValueError:
            return raw, ""
        offer = data.get("offer") if data.get("offer") in OFFER_LABELS else ""
        return str(data.get("answer") or "").strip(), offer
    return raw, ""


def discussed_idea(state: dict) -> dict:
    """Идея, которая сейчас на обсуждении: последняя со статусом discuss."""
    for idea in reversed(state.get("ideas") or []):
        if idea.get("status") == "discuss":
            return idea
    return {}


def run_reply_agent(text: str, idea: dict) -> str:
    """Ответ модели без инструментов (studio_reply.py: MiniMax по схеме {"answer"}, проверка кодом)."""
    script = Path(__file__).with_name("studio_reply.py")
    cmd = ["python3", str(script), "--message", text, "--json"]
    if idea.get("id"):
        cmd += ["--idea-id", idea["id"]]
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=400)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return (done.stdout or "").strip()


WATCH_LOCK = Path(os.environ.get("STUDIO_WATCH_LOCK", "/tmp/studio-poll.lock"))


def _single_instance(lock_path: Path):
    """Один получатель апдейтов на бота: второй getUpdates даёт 409 и теряет нажатия.

    Тот же файл замка, что у старого крона опроса (studio_poll.sh): пока жив сервис,
    случайно оставленный крон просто пропускает свой тик.
    """
    import fcntl

    handle = open(lock_path, "a+")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        return None
    return handle


def watch(
    bot: StudioBot,
    state_path: Path,
    timeout: int = 50,
    once: bool = False,
    out=print,
    lock_path: Optional[Path] = None,
) -> int:
    """Постоянный long-poll (systemd-сервис студии). once=True — один проход.

    Кнопки обслуживает этот процесс, а не крон: крон раз в 1–5 минут означал ожидание
    до следующего тика. Тяжёлая работа — в потоках JobRunner, опрос сразу ждёт следующее
    нажатие. Ошибка прохода не роняет сервис: пауза и новый проход (а упадёт процесс —
    systemd поднимет его, Restart=always).
    """
    lock = _single_instance(lock_path or WATCH_LOCK)
    if lock is None:
        out("[studio-watch] другой опрос уже держит бота — выхожу")
        return 0
    runner = JobRunner(threaded=True, out=out)
    total = 0
    try:
        while True:
            try:
                total += poll_once(bot, state_path, timeout=timeout, out=out, runner=runner)
            except Exception as exc:  # noqa: BLE001 — сервис живёт дальше, причина в журнал
                out(f"[studio-watch] проход упал: {type(exc).__name__}: {exc}")
                time.sleep(5)
            if once:
                runner.join()
                return total
    finally:
        lock.close()


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
        return False
    # Текст пишется только по согласованной идее.
    if idea.get("status") != "agreed":
        out(f"идея {idea_id} не согласована (статус {idea.get('status')!r}) — производство пропущено")
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
    # message_id появился только после отправки — без него правка на месте не найдёт сообщение.
    _save_post_fields(state_path, post_id, {k: record.get(k) for k in ("message_id", "message_kind")})
    out(f"идея {idea_id}: пост готов и отправлен с кнопками")
    return True


def _save_post_fields(state_path: Path, post_id: str, fields: dict) -> None:
    """Дописать поля поста под замком, не затирая остальное (короткая транзакция)."""
    with st.state_transaction(state_path) as current:
        record = current.setdefault("posts", {}).setdefault(post_id, {"id": post_id})
        record.update({k: v for k, v in fields.items() if v is not None})


def escape_html(text: str) -> str:
    """Экранировать текст для Telegram HTML.

    Заголовок черновика приходит от модели и может содержать «<» или «&».
    Без экранирования Telegram отклонил бы сообщение целиком — Шифу увидел бы
    пустоту вместо поста.
    """
    return (text or "").replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def draft_caption(post: dict) -> str:
    """Текст черновика: пометка «Черновик …» курсивом + сам пост, как он выйдет в канал.

    Пометка ставится, только если влезает в лимит: с медиа это подпись (1024 знака после
    разбора разметки), без медиа — сообщение (4096). Сам пост код уже проверил на лимит.
    """
    post_id = str(post.get("id") or "")
    body = post.get("body_html") or post.get("body") or ""
    label = f"Черновик {post_id}"
    if post.get("note"):
        label += f" · {post['note']}"
    head = f"<i>{escape_html(label)}</i>\n\n"
    limit = lib.CAPTION_LIMIT if post.get("media_path") else lib.MESSAGE_LIMIT
    if lib.visible_length(head + body) <= limit:
        return head + body
    return body


def send_post(bot: StudioBot, post: dict, chat_id: int, out=print) -> int:
    """Прислать Шифу черновик: медиа с подписью-постом и кнопками одним сообщением.

    Решение Шифу (06.11): черновик приходит сразу готовым, с графикой. Медиа —
    post["media_path"] (готовый файл или нарисованная инфографика); без него — текст.
    """
    post_id = str(post.get("id") or "")
    if not post_id:
        out("у поста нет id")
        return 0
    text = draft_caption(post)
    path = smedia.resolve(post.get("media_path") or "")
    if path is not None:
        result = bot.send_media(chat_id, path, caption=text, keyboard=post_keyboard(post_id))
        kind = "media"
    else:
        result = bot.send_message(chat_id, text, keyboard=post_keyboard(post_id))
        kind = "text"
    if not result.get("ok"):
        out(f"черновик не отправлен: {result.get('description')}")
        return 0
    # message_id нужен для правки на месте: по кнопке «короче» текст меняется
    # в том же сообщении, а не приходит новым.
    message_id = (result.get("result") or {}).get("message_id")
    if message_id:
        post["message_id"] = message_id
    post["message_kind"] = kind
    post["active_at"] = time.time()  # «черновик трогали»: studio_intent.target выбирает свежий
    out(f"черновик {post_id} отправлен ({kind})")
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
    # Только под замком: без него запись терялась. Опрос держал state_transaction весь
    # long-poll и в finally писал старое состояние поверх — черновик p20261106-129
    # исчез из state.json, и кнопка «Медиа» нашла пустую запись (разбор 06.11).
    with st.state_transaction(state_path) as state:
        record = state.setdefault("posts", {}).setdefault(post_id, {})
        record.setdefault("id", post_id)
        record.setdefault("status", "proposed")
        record.setdefault("title", post.get("title", ""))
        if post.get("body_html") or post.get("body"):
            record["body_html"] = post.get("body_html") or post.get("body")
        for key in POST_FIELDS:
            if post.get(key):
                record[key] = post[key]


# Поля черновика, которые переносятся из post.json в состояние.
POST_FIELDS = (
    "tags",
    "media_ref",
    "media_path",
    "media_choice",
    "fact_keys",
    "show",
    "message_id",
    "message_kind",
    "active_at",
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", type=Path, default=STATE_PATH, help="файл состояния студии (state.json)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_poll = sub.add_parser("poll", help="один проход getUpdates (для cron)")
    p_poll.add_argument("--timeout", type=int, default=30, help="таймаут long-poll, с")
    p_poll.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    p_watch = sub.add_parser("watch", help="постоянный long-poll (systemd-сервис студии, не крон)")
    p_watch.add_argument("--timeout", type=int, default=50, help="таймаут long-poll, с")
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

    p_draft = sub.add_parser("draft", help="собрать черновик из фактов и прислать Шифу с графикой и индикатором")
    p_draft.add_argument("--facts", type=Path, required=True, help="факты сессии (вывод studio_facts.py)")
    p_draft.add_argument("--note", default="", help="пометка в заголовке черновика (например «тест»)")
    p_draft.add_argument("--chat-id", type=int, default=int(os.environ.get("TG_SHIFU_CHAT_ID", "495039871")))
    p_draft.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    args = ap.parse_args()
    state_path = args.state

    if args.cmd == "state":
        print(json.dumps(st.load_state(state_path), ensure_ascii=False, indent=2))
        return 0

    bot = StudioBot(args.token)

    if args.cmd == "poll":
        # Разовый проход (ручная проверка). Постоянный опрос — `watch` под systemd.
        return 0 if poll_once(bot, state_path, timeout=args.timeout) >= 0 else 1
    if args.cmd == "watch":
        watch(bot, state_path, timeout=args.timeout, once=args.once)
        return 0
    if args.cmd == "draft":
        import studio_jobs as jobs

        ok = jobs.draft_job(bot, state_path, args.facts, chat_id=args.chat_id, note=args.note)
        return 0 if ok else 1
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
