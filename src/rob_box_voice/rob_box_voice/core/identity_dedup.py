"""Сказала ли LLM в этом ходе то же, что собирается сказать код (#3765, идея Шифу 21.10): Chooser ``{already_said, not_said}``.

Вопрос личности после ответа хода (#2828 «Вы разные люди или это ты…?», #2908 «скажи ещё пару слов») звучит кодом.
Если LLM в том же ходе уже спросила о том же своими словами, второй вопрос — двойной ответ. Решает модель по смыслу
(шаблон :mod:`.stop_chooser`: общий ``ModelChooser``, дедлайн ``IDENTITY_DEDUP_DEADLINE_MS``). LLM в ходе ничего не
говорила — ``not_said`` без вызова модели; таймаут, ошибка, нет ключа — ``not_said``: лучше спросить дважды, чем не
спросить. Ответные фразы ``merge_reply`` сюда не ходят — они отвечают человеку. Одна строка на решение:
``🔁 [identity-dedup] verdict=… ms=… reason=…``.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Mapping, Optional, Sequence, Tuple

from rob_box_dialog.knowledge import IDENTITY_DEDUP_CHOICES, IDENTITY_DEDUP_DEADLINE_MS
from rob_box_music.choice import Chooser, ChoiceQuestion, Option

_LOG = logging.getLogger(__name__)
ALREADY_SAID, NOT_SAID = IDENTITY_DEDUP_CHOICES
INSTRUCTIONS = (
    "Робот в этом ходе уже сказал человеку реплики (они ниже). Теперь робот собирается сказать фразу кода. Реши: "
    "реплики робота УЖЕ содержат по смыслу этот же вопрос или эту же просьбу (already_said) — или нет (not_said). "
    "Приветствие, пересказ, обсуждение другого — not_said. Выбери один id."
)
_LABELS: Mapping[str, str] = {
    ALREADY_SAID: "робот уже спросил или попросил о том же по смыслу — фразу кода не говорить",
    NOT_SAID: "в репликах робота этого вопроса или просьбы нет — фразу кода сказать",
}


def question(said: Sequence[str], phrase: str) -> ChoiceQuestion:
    text = "Реплики робота в этом ходе: " + " | ".join(f"«{s}»" for s in said) + f"\nФраза кода: «{phrase}»"
    options = tuple(Option(choice, {}, _LABELS[choice]) for choice in IDENTITY_DEDUP_CHOICES)
    return ChoiceQuestion(text, options, weights={}, instructions=INSTRUCTIONS)


def state_text(q: ChoiceQuestion) -> str:
    return q.text


class IdentityDedupChooser:
    """``decide(said, phrase) -> (already_said|not_said, reason)``; без модели — всегда ``not_said`` (reason ``off``)."""

    def __init__(self, model: Optional[Chooser], deadline_ms: int = IDENTITY_DEDUP_DEADLINE_MS,
                 log: Callable[[], Any] = lambda: _LOG, clock: Callable[[], float] = time.perf_counter) -> None:
        self._model, self._deadline_ms, self._log, self._clock = model, deadline_ms, log, clock

    def decide(self, said: Sequence[str], phrase: str) -> Tuple[str, str]:
        started = self._clock()
        said = [s.strip() for s in said if s and s.strip()]
        verdict, reason = NOT_SAID, "off"
        if not said:
            reason = "no_llm_speech"
        elif self._model is not None:
            try:
                answer = self._model.choose(question(said, phrase), deadline_ms=self._deadline_ms)
                verdict, reason = (answer.id, "model") if answer.id in IDENTITY_DEDUP_CHOICES else (NOT_SAID, "out")
            except Exception as exc:  # noqa: BLE001 — таймаут/ошибка: лучше спросить дважды, чем не спросить
                reason = f"timeout:{type(exc).__name__}"
        ms = (self._clock() - started) * 1000.0
        self._log().info(f"🔁 [identity-dedup] verdict={verdict} ms={ms:.0f} reason={reason} phrase={phrase[:50]!r} "
                         f"said={len(said)}")
        return verdict, reason


def identity_dedup_from_env(env: Optional[Mapping[str, str]] = None,
                            log: Callable[[], Any] = lambda: _LOG) -> IdentityDedupChooser:
    """Chooser над ``llm_harness`` (ключ из env), как :func:`.stop_chooser.stop_chooser_from_env`; нет ключа — без модели."""
    from rob_box_mcp_tools.engine.chooser_model import ModelChooser, build_provider  # noqa: PLC0415

    provider = build_provider("llm_harness", env, timeout_s=IDENTITY_DEDUP_DEADLINE_MS / 1000.0)
    if provider is None:
        log().warning("🔁 [identity-dedup] нет ключа/конфига llm_harness — вопрос личности звучит всегда")
        return IdentityDedupChooser(None, log=log)
    log().info(f"🔁 [identity-dedup] модель llm_harness, дедлайн {IDENTITY_DEDUP_DEADLINE_MS} мс")
    return IdentityDedupChooser(ModelChooser(provider, deadline_ms=IDENTITY_DEDUP_DEADLINE_MS, state=state_text),
                                log=log)


__all__ = ["ALREADY_SAID", "INSTRUCTIONS", "IdentityDedupChooser", "NOT_SAID", "identity_dedup_from_env", "question",
           "state_text"]
