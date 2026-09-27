"""yandex_stt_segments.py — склейка сегментов Yandex STT v3 в одну фразу (issue #2891).

Yandex SpeechKit gRPC v3 (``RecognizeStreaming``) режет присланное аудио на
сегменты по своему EOU-классификатору (у нас профиль ``balanced``,
max_pause 1200 мс). На КАЖДЫЙ сегмент сервер шлёт отдельный ``final``, а при
включённой нормализации — ещё и ``final_refinement`` с нормализованным
текстом того же сегмента. Фраза «Робот, здравствуй, я Саша, чиню тут технику
по вечерам.» с паузой после «здравствуй» приходит двумя сегментами.

До #2891 ``stt_node`` перезаписывал текст на каждом ``final`` и выходил из
стрима на первом ``final_refinement`` — оставался только первый сегмент
(«робот здравствуй»).

Поля ответа (``yandex/cloud/ai/stt/v3/stt.proto``):

- ``StreamingResponse.partial`` (``AlternativeUpdate``) — «current text
  estimation from ``final_time_ms`` to ``partial_time_ms``»: гипотеза по
  ещё не зафиксированному куску аудио (после последнего ``final``);
- ``StreamingResponse.final`` (``AlternativeUpdate``) — текст сегмента,
  ``alternatives[0].text``; ``Alternative.start_time_ms/end_time_ms`` — его
  границы во времени аудио;
- ``StreamingResponse.audio_cursors.final_index`` — «index of last final
  server send. Incremented after each new final»;
- ``StreamingResponse.final_refinement.final_index`` — «Index of final for
  which server sends additional information»;
- ``final_refinement.normalized_text.alternatives[0].text`` — уточнённый текст.

Инварианты сборщика (issue #2931 — «робот» пропал из начала фразы):

1. Каждый непустой ``final`` даёт ровно один сегмент, сегменты идут в
   порядке прихода. Уточнение (``final_refinement``) может только ЗАМЕНИТЬ
   текст сегмента, но не добавить и не убрать сегмент.
2. Уточнение применяется только к сегменту, чей ``final`` совпадает с ним
   по словам (:func:`_same_words`). Индексы v3 неоднозначны (курсор
   «после инкремента» против индекса уточнения), и чистая эвристика по
   индексу могла отдать сегменту «робот» текст соседнего сегмента: итог —
   «привет … привет …» (дубль + потеря) или «привет … робот» (перестановка).
   Сначала пробуем неуточнённый сегмент с тем же индексом, затем самый
   старый неуточнённый (уточнения идут в порядке finals). Не совпал ни один
   по словам — уточнение отбрасывается, остаётся сырой ``final``.
3. Текст, который сервер показал только в ``partial`` и не зафиксировал в
   ``final``, не теряется: если следующий ``final`` пуст — сегментом
   становится последний partial; если partial по времени целиком лежит
   ДО начала следующего ``final`` (``end_time_ms <= start_time_ms``) — он
   отдельный сегмент перед ним; partial после последнего ``final`` (стрим
   закрылся без EOU на хвосте) дописывается в конец. Без таймингов (0)
   partial вставляется только на место пустого ``final`` или когда
   ``final`` не было вовсе — так не бывает дубля с тем же куском аудио.
   Если новый partial начинается ПОСЛЕ конца предыдущего (а final между
   ними не было), предыдущий — отдельный кусок аудио: он тоже сохраняется.

Какой из путей реально случился на роботе, показывает :meth:`trace` —
короткая запись событий стрима для INFO-лога ``stt_node``.

Модуль не импортирует protobuf: ответы читаются duck-typing'ом, поэтому
тестируется на ``SimpleNamespace`` без ``yandex``/``grpc``.
"""

from __future__ import annotations

import re
from typing import List, Optional

# Сколько символов текста события кладём в trace (INFO-лог не раздуваем).
_TRACE_TEXT_CHARS = 24
# Потолок длины trace целиком.
_TRACE_MAX_CHARS = 600
# Доля общих слов, при которой уточнение считается «тем же сегментом».
_SAME_WORDS_MIN_RATIO = 0.5

_WORD_RE = re.compile(r"\w+", re.UNICODE)


def _first_alternative(update):
    """``alternatives[0]`` из ``AlternativeUpdate`` или ``None``."""
    alternatives = getattr(update, "alternatives", None)
    if not alternatives:
        return None
    return alternatives[0]


def _first_alternative_text(update) -> str:
    """``alternatives[0].text`` из ``AlternativeUpdate`` или ``""``."""
    alt = _first_alternative(update)
    return (getattr(alt, "text", "") or "") if alt is not None else ""


def _time_ms(alt, field: str) -> int:
    try:
        return int(getattr(alt, field, 0) or 0)
    except (TypeError, ValueError):
        return 0


def _words(text: str) -> List[str]:
    return _WORD_RE.findall(text.lower().replace("ё", "е"))


def _same_words(raw: str, refined: str) -> bool:
    """Уточнение — нормализация ЭТОГО сегмента, а не соседнего.

    Нормализация меняет регистр, пунктуацию и числа («девяносто восьмого» →
    «98»), но большая часть слов остаётся. Текст другого сегмента общих
    слов почти не имеет.
    """
    a, b = _words(raw), _words(refined)
    if not a or not b:
        return False
    common = len(set(a) & set(b))
    return common / max(len(set(a)), len(set(b))) >= _SAME_WORDS_MIN_RATIO


def _short(text: str) -> str:
    text = text.strip()
    if len(text) <= _TRACE_TEXT_CHARS:
        return text
    return text[:_TRACE_TEXT_CHARS] + "…"


class _Partial:
    __slots__ = ("text", "start_ms", "end_ms", "count")

    def __init__(self) -> None:
        self.text = ""
        self.start_ms = 0
        self.end_ms = 0
        self.count = 0


class _Segment:
    __slots__ = ("index", "final", "refined")

    def __init__(self, index: int, final: str) -> None:
        self.index = index
        self.final = final
        self.refined: Optional[str] = None

    def text(self) -> str:
        return (self.refined or self.final or "").strip()


class YandexSegmentCollector:
    """Собирает текст всех сегментов одного стрима по порядку, без дублей."""

    def __init__(self) -> None:
        self._segments: List[_Segment] = []
        self._pending = _Partial()
        # Partial'ы, которые сервер «отпустил» без final: следующий partial
        # начался по времени после их конца (инвариант 3).
        self._orphans: List[str] = []
        self._last_final_end_ms = 0
        self._trace: List[str] = []

    # ── приём событий ────────────────────────────────────────────────
    def feed(self, response, event_type: str) -> None:
        """Учесть ответ стрима; события кроме partial/final/final_refinement
        только попадают в trace."""
        if event_type == "partial":
            self._add_partial(response)
        elif event_type == "final":
            self._add_final(response)
        elif event_type == "final_refinement":
            self._add_refinement(response.final_refinement)
        elif event_type == "eou_update":
            self._trace.append("E")

    def _add_partial(self, response) -> None:
        alt = _first_alternative(getattr(response, "partial", None))
        text = (getattr(alt, "text", "") or "").strip() if alt is not None else ""
        if not text:
            return
        start_ms = _time_ms(alt, "start_time_ms")
        if self._pending_precedes(start_ms):
            self._flush_partial_trace()
            self._orphans.append(self._pending.text)
            self._trace.append("P→сегмент")
            self._pending = _Partial()
        self._pending.text = text
        self._pending.start_ms = start_ms
        self._pending.end_ms = _time_ms(alt, "end_time_ms")
        self._pending.count += 1

    def _add_final(self, response) -> None:
        cursors = getattr(response, "audio_cursors", None)
        index = int(getattr(cursors, "final_index", 0) or 0)
        alt = _first_alternative(getattr(response, "final", None))
        text = (getattr(alt, "text", "") or "").strip() if alt is not None else ""
        start_ms = _time_ms(alt, "start_time_ms")
        self._flush_partial_trace()
        self._segments.extend(_Segment(index, orphan) for orphan in self._orphans)
        self._orphans = []
        if not text and self._pending.text:
            # Сервер закрыл сегмент пустым final, хотя partial текст видел.
            text = self._pending.text
            self._trace.append("F#%d(пуст→P)" % index)
        else:
            if self._pending_precedes(start_ms):
                self._segments.append(_Segment(index, self._pending.text))
                self._trace.append("P→сегмент")
            self._trace.append(
                "F#%d'%s'@%d-%d"
                % (index, _short(text), start_ms, _time_ms(alt, "end_time_ms"))
            )
        self._segments.append(_Segment(index, text))
        self._last_final_end_ms = (
            _time_ms(alt, "end_time_ms") or self._last_final_end_ms
        )
        self._pending = _Partial()

    def _pending_precedes(self, start_ms: int) -> bool:
        """Текущий partial целиком по времени ДО ``start_ms`` (начала final
        или следующего partial) — отдельный кусок аудио, а не ранняя
        гипотеза того же."""
        p = self._pending
        return bool(p.text) and p.end_ms > 0 and start_ms > 0 and p.end_ms <= start_ms

    def _add_refinement(self, refinement) -> None:
        index = int(getattr(refinement, "final_index", 0) or 0)
        text = _first_alternative_text(
            getattr(refinement, "normalized_text", None)
        ).strip()
        if not text:
            self._trace.append("R#%d(пуст)" % index)
            return
        target = self._find_refinement_target(index, text)
        if target is None:
            self._trace.append("R#%d'%s'(не к чему)" % (index, _short(text)))
            return
        target.refined = text
        self._trace.append("R#%d→#%d" % (index, target.index))

    def _find_refinement_target(self, index: int, text: str) -> Optional[_Segment]:
        unrefined = [s for s in self._segments if s.refined is None]
        by_index = [s for s in unrefined if s.index == index]
        for seg in by_index + unrefined:
            if _same_words(seg.final, text):
                return seg
        return None

    def _flush_partial_trace(self) -> None:
        if self._pending.count:
            self._trace.append(
                "P×%d'%s'@%d-%d"
                % (
                    self._pending.count,
                    _short(self._pending.text),
                    self._pending.start_ms,
                    self._pending.end_ms,
                )
            )

    # ── результат ────────────────────────────────────────────────────
    def _tail_partial(self) -> str:
        """Partial после последнего final — хвост без EOU (или вся фраза)."""
        p = self._pending
        if not p.text:
            return ""
        if not self._segments and not self._orphans:
            return p.text
        if (
            p.start_ms > 0
            and self._last_final_end_ms > 0
            and p.start_ms >= self._last_final_end_ms
        ):
            return p.text
        return ""

    def _parts(self) -> List[str]:
        parts = [s.text() for s in self._segments if s.text()]
        parts.extend(self._orphans)
        tail = self._tail_partial()
        if tail:
            parts.append(tail)
        return parts

    @property
    def segment_count(self) -> int:
        """Число непустых сегментов (для телеметрии)."""
        return len(self._parts())

    def text(self) -> Optional[str]:
        """Склеенный текст всех непустых сегментов или ``None``."""
        parts = self._parts()
        return " ".join(parts) if parts else None

    def trace(self) -> str:
        """Короткая запись событий стрима для INFO-лога (issue #2931).

        ``P×N'текст'@start-end`` — N partial'ов до final (показан последний),
        ``F#i'текст'@start-end`` — final с ``audio_cursors.final_index``,
        ``R#i→#j`` — уточнение применено к сегменту j, ``E`` — eou_update.
        """
        items = list(self._trace)
        if self._pending.count:
            items.append(
                "P×%d'%s'@%d-%d(хвост)"
                % (
                    self._pending.count,
                    _short(self._pending.text),
                    self._pending.start_ms,
                    self._pending.end_ms,
                )
            )
        out = " ".join(items) or "-"
        if len(out) > _TRACE_MAX_CHARS:
            out = out[: _TRACE_MAX_CHARS - 1] + "…"
        return out
