"""Такт рэпа в адаптере голоса (ADR-0160 §2.5 шаги 1–3, #4241).

Диалог провайдера не знает: сегмент выступления приходит голыми строками и тактом подложки ``beat_s`` (данные, не
метка). Паузы по ним ставит адаптер того сервиса, который реплику синтезирует:

* MiniMax (``pause_markup``) — :func:`minimax_text`: числа словами (длина строки считается по гласным, в «2024» их
  нет), затем паузы ``<#x#>`` по сетке строк ``rap_pauses.markup`` с темпом MiniMax (спайк PR-S1);
* Yandex и Silero — :func:`without_beat`: текст как есть, одна строка лога «рэп без такта» (решение Шифу 30.10, В3:
  темп запасных не мерим, такта у них нет).

Синтез MiniMax — ``rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider``; текст для него готовится здесь: раскладка
пауз и нормализатор — пакеты робота, которые ``rob_box_llm`` не импортирует.
"""

from __future__ import annotations

from typing import Callable

from rob_box_dialog.plan import rap_pauses

from ..tts_normalizer import spell_out

Log = Callable[[str], None]
LOG_MARK = "🎤 [beat]"


def minimax_text(text: str, beat_s: float, log: Log) -> str:
    """Текст для MiniMax: такт есть и строк больше одной — числа словами и паузы ``<#x#>`` по сетке; иначе как есть
    (MiniMax цифры читает сам)."""
    if beat_s <= 0:
        return text
    mark = rap_pauses.markup(spell_out(text), beat_s)
    if mark is None:
        log(f"{LOG_MARK} minimax: такт {beat_s:.2f} с, одна строка — пауз нет, встаёт на такт целиком")
        return text
    log(f"{LOG_MARK} minimax: такт {beat_s:.2f} с, тактов на строку {mark.bars_per_line}, паузы {mark.pauses}")
    return mark.text


def without_beat(provider: str, text: str, beat_s: float, log: Log) -> str:
    """Сервис без пауз такта: текст как есть; был такт — честная строка лога (решение Шифу 30.10, В3)."""
    if beat_s > 0:
        log(f"{LOG_MARK} {provider}: рэп без такта (такт {beat_s:.2f} с) — паузы такта ставит только MiniMax, "
            f"темп {provider} не мерен")
    return text


__all__ = ["LOG_MARK", "minimax_text", "without_beat"]
