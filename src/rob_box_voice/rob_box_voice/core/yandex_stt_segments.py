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

- ``StreamingResponse.final`` (``AlternativeUpdate``) — текст сегмента,
  ``alternatives[0].text``;
- ``StreamingResponse.audio_cursors.final_index`` — «index of last final
  server send. Incremented after each new final»: индекс сегмента, к которому
  относится этот ``final``;
- ``StreamingResponse.final_refinement.final_index`` — «Index of final for
  which server sends additional information»: индекс уточняемого сегмента;
- ``final_refinement.normalized_text.alternatives[0].text`` — уточнённый текст.

Сопоставление refinement → сегмент: по ``final_index`` среди ещё не
уточнённых сегментов; если такого нет (сервер не заполнил индекс или
семантика индекса «после инкремента») — уточняется последний неуточнённый
сегмент (refinement по контракту идёт после своего final). Так один
сегмент никогда не попадает в текст дважды.

Модуль не импортирует protobuf: ответы читаются duck-typing'ом, поэтому
тестируется на ``SimpleNamespace`` без ``yandex``/``grpc``.
"""

from __future__ import annotations

from typing import List, Optional


def _first_alternative_text(update) -> str:
    """``alternatives[0].text`` из ``AlternativeUpdate`` или ``""``."""
    alternatives = getattr(update, "alternatives", None)
    if not alternatives:
        return ""
    return getattr(alternatives[0], "text", "") or ""


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

    def feed(self, response, event_type: str) -> None:
        """Учесть ответ стрима; события кроме final/final_refinement игнорируются."""
        if event_type == "final":
            self._add_final(response)
        elif event_type == "final_refinement":
            self._add_refinement(response.final_refinement)

    def _add_final(self, response) -> None:
        cursors = getattr(response, "audio_cursors", None)
        index = int(getattr(cursors, "final_index", 0) or 0)
        self._segments.append(_Segment(index, _first_alternative_text(response.final)))

    def _add_refinement(self, refinement) -> None:
        text = _first_alternative_text(getattr(refinement, "normalized_text", None))
        if not text.strip():
            return
        target = self._find_unrefined(int(getattr(refinement, "final_index", 0) or 0))
        if target is not None:
            target.refined = text

    def _find_unrefined(self, index: int) -> Optional[_Segment]:
        unrefined = [s for s in self._segments if s.refined is None]
        for seg in unrefined:
            if seg.index == index:
                return seg
        return unrefined[-1] if unrefined else None

    @property
    def segment_count(self) -> int:
        """Число непустых сегментов (для телеметрии)."""
        return sum(1 for s in self._segments if s.text())

    def text(self) -> Optional[str]:
        """Склеенный текст всех непустых сегментов или ``None``."""
        parts = [s.text() for s in self._segments if s.text()]
        return " ".join(parts) if parts else None
