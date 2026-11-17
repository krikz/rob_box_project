"""Оформитель чата Telegram: голый текст реплики -> сообщения Bot API (ADR-0160 §2.4, #4243).

Модель пишет голый текст, а как он выглядит в чате, решает код. Знание о Telegram (формат, лимиты, что умеет
разметка) живёт ЗДЕСЬ и нигде больше: ``chat_sender`` и узел только отправляют то, что вернул :func:`render`.

Формат - ``parse_mode="HTML"`` (Bot API, «Formatting options»): HTML экранируется тремя символами (``& < >``), а
MarkdownV2 требует экранировать 18 знаков и ломается на точке или дефисе в обычной речи. Текст модели в HTML
попадает только через :func:`escape`: тег из слов модели не оживёт.

Что делает оформитель:
- герой (``role``) - жирный заголовок и реплика цитатой (``<blockquote>``); рассказчик (``role == ""``) - обычный текст;
- длинное - первый абзац виден, остальное в раскрывающейся цитате (``<blockquote expandable>``: клиент сам
  показывает три строки и кнопку «развернуть»);
- больше лимита сообщения - режется по абзацам, строкам, словам; роль - в заголовке первой части;
- числа остаются цифрами (решение В2: словами только в голосе).

Если Telegram всё же отверг разметку, отправщик шлёт ``Rendered.plain`` без ``parse_mode``: реплика не пропадает.
Чистый Python без telegram и ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

#: Лимит текста сообщения после разбора разметки (Bot API, sendMessage: 1-4096 символов).
MESSAGE_LIMIT = 4096
#: Лимит подписи под медиа (Bot API: 0-1024). Сейчас реплики идут текстом; лимит записан для карточки с картинками.
CAPTION_LIMIT = 1024
PARSE_MODE = "HTML"
#: Теги и заголовок занимают место: тело части режем с запасом, чтобы размеченная часть влезла в лимит.
_BODY_LIMIT = MESSAGE_LIMIT - 256
#: Короче этого и без переносов - обычный текст без цитаты: цитата ради «Привет!» - шум.
PLAIN_MAX_CHARS = 400
#: Строк больше этого - тоже цитата, даже если символов мало (стих, песня).
PLAIN_MAX_LINES = 6


@dataclass(frozen=True)
class Rendered:
    """Одно сообщение чата: ``text`` с разметкой ``parse_mode``, ``plain`` - то же без разметки (запасной путь)."""

    text: str
    plain: str
    parse_mode: str = PARSE_MODE


def escape(text: str) -> str:
    """Экранирование под HTML Bot API: только ``&``, ``<``, ``>`` (остальные именованные сущности API не знает)."""
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _quote(body: str, expandable: bool) -> str:
    return f"<blockquote{' expandable' if expandable else ''}>{escape(body)}</blockquote>"


def _is_long(text: str) -> bool:
    return len(text) > PLAIN_MAX_CHARS or text.count("\n") + 1 > PLAIN_MAX_LINES


def _narrator(text: str) -> Tuple[str, str]:
    """Реплика рассказчика: короткая - как есть; длинная - первый абзац видно, остальное под «развернуть»."""
    if not _is_long(text):
        return escape(text), text
    lead, sep, rest = text.partition("\n\n")
    if not sep or not rest.strip():
        return _quote(text, True), text
    return f"{escape(lead)}\n{_quote(rest.strip(), True)}", text


def _hero(text: str, role: str, first: bool) -> Tuple[str, str]:
    """Реплика героя: заголовок с именем (в первой части) и цитата; продолжение без заголовка."""
    head = f"<b>{escape(role)}</b>\n" if first else ""
    plain_head = f"{role}\n" if first else ""
    return f"{head}{_quote(text, _is_long(text))}", f"{plain_head}{text}"


def _cut(text: str, limit: int) -> Tuple[str, str]:
    """Отрезать от текста кусок до ``limit`` по границе: абзац, строка, предложение, слово, иначе жёстко."""
    if len(text) <= limit:
        return text, ""
    window = text[:limit]
    for mark in ("\n\n", "\n", ". ", "! ", "? ", " "):
        at = window.rfind(mark)
        if at > limit // 3:  # слишком ранняя граница даёт крошечную первую часть
            end = at + len(mark)
            return text[:end].rstrip(), text[end:].lstrip()
    return window, text[limit:]


def split(text: str, limit: int = _BODY_LIMIT) -> List[str]:
    """Порезать длинный текст на части не длиннее ``limit``; пустые части не возвращаются."""
    parts: List[str] = []
    rest = text.strip()
    while rest:
        part, rest = _cut(rest, limit)
        if part:
            parts.append(part)
    return parts


def render(text: str, role: str = "") -> List[Rendered]:
    """Реплика робота -> сообщения чата по порядку. Пустой текст - пустой список."""
    out: List[Rendered] = []
    for index, part in enumerate(split(text)):
        marked, plain = _hero(part, role, index == 0) if role else _narrator(part)
        out.append(Rendered(marked, plain))
    return out


#: Сколько последних шагов хода видно в индикаторе (длинный ход не раздувает сообщение).
PROGRESS_STEPS_SHOWN = 6


def progress(steps: "Sequence[str] | str", elapsed_s: float) -> Rendered:
    """Шаги хода («Думаю…», «Ищу в сети…», «Ищу в сети - готово») и время; правится одним сообщением, пока ход идёт.
    Прошлые шаги - точкой, текущий (последний) - песочными часами со счётчиком секунд."""
    shown = [steps] if isinstance(steps, str) else list(steps)[-PROGRESS_STEPS_SHOWN:]
    lines = [f"• {step}" for step in shown[:-1]] + [f"⏳ {shown[-1]} {int(elapsed_s)} с"]
    text = "\n".join(lines)
    return Rendered(f"<i>{escape(text)}</i>", text)


__all__ = ["CAPTION_LIMIT", "MESSAGE_LIMIT", "PARSE_MODE", "PLAIN_MAX_CHARS", "PLAIN_MAX_LINES", "Rendered", "escape",
           "PROGRESS_STEPS_SHOWN", "progress", "render", "split"]
