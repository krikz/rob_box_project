"""Сбой MiniMax при пении — типизированная причина, а не «что-то пошло не так» (живой сбой 13.10 20:55 UTC).

Тогда ``TTSTimeoutError`` с пустым текстом пролетел мимо ``SingSynthError`` и стал ``internal``. Здесь: ошибки
провайдера → ``SingSynthError(reason)`` по типу; обрыв/таймаут сети — один повтор в той же петле; свой httpx-клиент
закрывается после каждого вокала. Без сети: провайдер — фейк.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

pytest.importorskip("rob_box_llm")

from rob_box_llm.errors import (  # noqa: E402
    TTSAuthError,
    TTSBadRequestError,
    TTSConnectionError,
    TTSError,
    TTSRateLimitError,
    TTSTimeoutError,
)
from rob_box_llm.tts import TTSAudio, TTSFormat  # noqa: E402

from rob_box_voice import sing_synth  # noqa: E402
from rob_box_voice.sing_synth import MiniMaxSingBackend, SingSynthError, synth_vocal_to_file  # noqa: E402

OK = TTSAudio(samples=b"ID3ok", sample_rate=32000, format=TTSFormat.MP3, raw={"extra_info": {"audio_length": 1000}})


class _Prov:
    """Отвечает по очереди: исключение — бросить, иначе вернуть аудио."""

    def __init__(self, *answers) -> None:
        self.answers, self.calls, self.closed = list(answers), 0, 0

    async def synthesize(self, text, *, settings):
        self.calls += 1
        answer = self.answers.pop(0)
        if isinstance(answer, BaseException):
            raise answer
        return answer

    async def aclose(self) -> None:
        self.closed += 1


def _sing(prov, tmp_path):
    return synth_vocal_to_file("minimax", "male-qn-qingse", "Я робот", tmp_path / "v",
                               backends={"minimax": MiniMaxSingBackend(provider=prov)})


@pytest.mark.parametrize("exc,reason", [
    (TTSConnectionError("ReadError", provider="minimax"), "network"),
    (TTSTimeoutError("ReadTimeout", provider="minimax"), "timeout"),
    (TTSAuthError("401: bad key", provider="minimax"), "auth"),
    (TTSRateLimitError("minimax API error 1002: rate limit", provider="minimax"), "rate_limit"),
    (TTSBadRequestError("minimax API error 2013: invalid params", provider="minimax"), "rejected"),
    (TTSError("500: upstream", provider="minimax"), "provider_error"),
])
def test_provider_error_becomes_a_typed_reason_with_its_class_in_the_detail(tmp_path, exc, reason):
    prov = _Prov(exc, exc)
    with pytest.raises(SingSynthError) as err:
        _sing(prov, tmp_path)
    assert err.value.reason == reason
    assert type(exc).__name__ in err.value.detail and str(exc) in err.value.detail
    assert not list(tmp_path.iterdir())  # пустой файл не создан


def test_network_drop_and_timeout_are_retried_once_other_errors_are_not(tmp_path):
    flaky = _Prov(TTSConnectionError("ReadError", provider="minimax"), OK)
    assert _sing(flaky, tmp_path).size_bytes == len(b"ID3ok") and flaky.calls == 2
    slow = _Prov(TTSTimeoutError("ReadTimeout", provider="minimax"), TTSTimeoutError("ReadTimeout", provider="m"))
    with pytest.raises(SingSynthError):
        _sing(slow, tmp_path)
    assert slow.calls == sing_synth.MINIMAX_SING_ATTEMPTS == 2
    auth = _Prov(TTSAuthError("401", provider="minimax"), OK)
    with pytest.raises(SingSynthError):
        _sing(auth, tmp_path)
    assert auth.calls == 1  # повтор ключ не лечит


def test_no_key_is_its_own_reason():
    backend = MiniMaxSingBackend(api_key="", group_id="")
    with pytest.raises(SingSynthError) as err:
        backend.synth("т", sing_synth.sing_preset_for("minimax", "male-qn-qingse"))
    assert err.value.reason == "no_api_key"


def test_own_client_is_made_and_closed_per_vocal_injected_one_is_kept(monkeypatch, tmp_path):
    """Свой клиент живёт в одной петле ``_run_coro`` и закрывается в ней: пул не переезжает в чужую петлю."""
    made = []

    def new_client(self):
        made.append(_Prov(OK))
        return made[-1]

    monkeypatch.setattr(MiniMaxSingBackend, "_new_client", new_client)
    backend = MiniMaxSingBackend(api_key="k")
    preset = sing_synth.sing_preset_for("minimax", "male-qn-qingse")
    backend.synth("раз", preset)
    backend.synth("два", preset)
    assert [(p.calls, p.closed) for p in made] == [(1, 1), (1, 1)]
    injected = _Prov(OK)
    _sing(injected, tmp_path)
    assert injected.closed == 0  # чужой (тестовый/общий) клиент не закрываем
