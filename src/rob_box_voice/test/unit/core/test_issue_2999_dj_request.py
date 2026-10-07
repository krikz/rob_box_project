"""Issue #2999 / ADR-0140 — «Ты диджей X…» должен включать/менять DJ-сет.

Два живых сценария (Vision Pi, LLM minimax):

* **A (24.09.2026 11:40–11:44 UTC)** — трек играет с прошлого хода, DJ
  выключен. «Ты диджей Снупдог и у нас сегодня вечеринка ганкста…» →
  ``tools=[]``; Bug C ретраил музыкальным промптом ×3 на каждый из 5
  запросов, сет не стартовал до «забудь все».
* **B (28.09.2026, 1790611527)** — DJ включён («диджей Дайв»). «Ты диджей
  Анакен скайвокер и у нас сегодня имперский слет в клубе» → обёртка
  «Не вызывай set_dj_mode», ``tools=[]`` → ``['lookup_melody']`` →
  разовый ``compose_music``; персона/тема сета не сменились.

Issue #3134: «ты диджей X» исполняет роутер медиакоманд кодом ДО LLM
(``set_dj_mode``), поэтому DJ-ретрай ``MusicGuard.evaluate_turn``, его
промпт/фолбэк и особая обёртка хода удалены. Здесь остались: детектор
(перенесён в ``core/media_command_grammar.py``) и утечка CRITICAL в TTS.
``DJModeController`` удалён в ADR-0149 PR-13a (сет ведёт движок v2). Роутер —
в ``test_issue_3134_media_router.py``.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import (
    extract_dj_persona,
    is_dj_request,
)
from rob_box_voice.core.turn_speech import decide_turn_speech, is_retry_prompt_leak

LIVE_A = "[TG] Ты диджей Снупдог и у нас сегодня вечеринка ганкста в чорном квартале"
LIVE_B = "[TG] Ты диджей Анакен скайвокер и у нас сегодня имперский слет в клубе"
LIVE_DIVE = "[TG] Ты диджей Дайв и у нас сегодня вечеринка в клубе"


# ── 1. Детектор: таблица ────────────────────────────────────────────────


@pytest.mark.parametrize(
    "text",
    [
        LIVE_A,
        LIVE_B,
        LIVE_DIVE,
        "Ты диджей PAUL OAKENFOLD и у нас сегодня вечеринка",
        "стань диджеем",
        "будь диджеем Псом",
        "теперь ты диджей Кот",
        "ты сегодня диджей",
        "ты dj snoop",
        "Ты ДИДЖЕЙ Снуп",
        "запусти диджей-сет",
        "включи режим диджея",
        "давай dj сет",
        "врубай dj mode",
    ],
)
def test_is_dj_request_positive(text: str) -> None:
    assert is_dj_request(text) is True


@pytest.mark.parametrize(
    "text",
    [
        "",
        None,
        "диджей, сделай громче",
        "диджей Дайв, сделай погромче",
        "ты диджей, сделай громче",
        "ты диджей?",
        "стоп диджей",
        "хватит диджеить",
        "выключи диджея",
        "выключи режим диджея",
        "кто такой диджей?",
        "диджей играет в наушниках",
        "ты диджеишь круто",
        "сыграй трек",
        "включи музыку",
        "сделай громче",
    ],
)
def test_is_dj_request_negative(text) -> None:
    assert is_dj_request(text) is False


@pytest.mark.parametrize(
    ("text", "persona"),
    [
        (LIVE_A, "диджей Снупдог"),
        (LIVE_B, "диджей Анакен скайвокер"),
        (LIVE_DIVE, "диджей Дайв"),
        ("давай dj сет", ""),
        ("стань диджеем", ""),
    ],
)
def test_extract_dj_persona(text: str, persona: str) -> None:
    """Тему «у нас сегодня …» грамматика больше не выделяет — её выделяет ``dj_set`` из ``heard_text`` (07.10)."""
    assert extract_dj_persona(text) == persona


# ── 4. CRITICAL не доходит до TTS ──────────────────────────────────────


@pytest.mark.parametrize(
    "spoken",
    [
        "[CRITICAL] В прошлом цикле ты НЕ вызвал set_dj_mode, хотя юзер назначил тебя диджеем",
        "[Speaker:unknown] [CRITICAL] В прошлом цикле ты НЕ вызвал set_dj_mode",
        "Йо! [critical] вызови set_dj_mode",
    ],
)
def test_critical_leak_is_never_spoken(spoken: str) -> None:
    assert is_retry_prompt_leak(spoken)
    # Даже когда ретрая нет и тул вызван (гейт #1882 такой ответ пропускает).
    assert decide_turn_speech(spoken, n_chunks=1) is None


def test_normal_dj_reply_is_spoken() -> None:
    reply = "Йо, Анакен на связи, имперский слет начинается!"
    assert not is_retry_prompt_leak(reply)
    assert decide_turn_speech(reply, n_chunks=1) == reply
