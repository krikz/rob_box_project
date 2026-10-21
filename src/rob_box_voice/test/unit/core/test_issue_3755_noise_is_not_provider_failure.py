"""#3755: таймаут STT-фолбека на шуме - не отказ провайдера; фраза об отказе - своего вида и только про активного."""

from __future__ import annotations

from types import SimpleNamespace

from rob_box_dialog.degrade import ProviderBoard
from rob_box_voice.core.provider_health import stt_provider_states
from rob_box_voice.core.provider_reporter import ProviderStateReporter
from rob_box_voice.stt_fallback import ProviderDeadCache, STTQuotaError, STTTimeoutError, select_recognition


class _Stt:
    def __init__(self, name, result=None, exc=None):
        self.name, self._result, self._exc = name, result, exc

    def recognize(self, audio):
        if self._exc is not None:
            raise self._exc
        return self._result


class _Llm:
    def __init__(self, rows):
        self._rows = rows

    def snapshot(self):
        return [{"name": n, "health": h, "since": 0.0, "last_error_code": ""} for n, h in self._rows]


class _Rig:
    """STT-нода + репортёр на одних часах: фраза прогоняется как в stt_node, состояние уходит на доску."""

    def __init__(self, chain, llm_rows=(("deepseek", "ok"), ("minimax", "ok"))):
        self.now = [0.0]
        self.cache = ProviderDeadCache(clock=lambda: self.now[0])
        self.chain = chain
        self.node = SimpleNamespace(provider_chain=[p.name for p in chain], _provider_dead_cache=self.cache)
        self.board, self.spoken = ProviderBoard(), []
        self.reporter = ProviderStateReporter(
            _Llm(llm_rows), lambda s: None, self.spoken.append, board=self.board, clock=lambda: self.now[0])

    def set_chain(self, chain):
        self.chain = chain
        self.node.provider_chain = [p.name for p in chain]

    def phrase(self):
        select_recognition(self.chain, b"pcm", dead_cache=self.cache, clock=lambda: self.now[0])
        self.last_health = {s.name: s.health for s in stt_provider_states(self.node)}
        self.board.update(stt_provider_states(self.node), self.now[0])
        self.reporter.tick()
        self.now[0] += 400.0  # TTL мёртвых истёк: следующая фраза снова пробует всех


def test_noise_timeout_of_fallback_after_empty_primary_says_nothing_three_times():
    rig = _Rig([_Stt("yandex", ""), _Stt("minimax", exc=STTTimeoutError("t")), _Stt("vosk", "")])
    for _ in range(3):
        rig.phrase()
    assert rig.spoken == []
    assert set(rig.last_health.values()) == {"ok"}


def test_minimax_llm_quota_with_deepseek_first_is_silent():
    rig = _Rig([_Stt("yandex", "привет робот")], llm_rows=(("deepseek", "ok"), ("minimax", "quota")))
    rig.phrase()
    assert rig.spoken == []


def test_deepseek_llm_quota_says_one_phrase_switching_to_minimax():
    rig = _Rig([_Stt("yandex", "привет робот")], llm_rows=(("deepseek", "quota"), ("minimax", "ok")))
    rig.phrase()
    rig.phrase()
    assert len(rig.spoken) == 1 and "deepseek" in rig.spoken[0] and "переключаюсь на minimax" in rig.spoken[0].lower()


def test_real_stt_quota_of_active_provider_names_the_kind_and_stt_fallback():
    rig = _Rig([_Stt("yandex", exc=STTQuotaError("402")), _Stt("minimax", "привет робот"), _Stt("vosk", "")])
    rig.phrase()
    rig.phrase()
    assert len(rig.spoken) == 1
    assert "yandex" in rig.spoken[0] and "распознавани" in rig.spoken[0] and "minimax" in rig.spoken[0]
    assert "deepseek" not in rig.spoken[0]


def test_fallback_stt_failing_on_real_speech_goes_red_on_board_but_stays_silent():
    rig = _Rig([_Stt("yandex", ""), _Stt("minimax", exc=STTTimeoutError("t")), _Stt("vosk", "включи музыку пожалуйста")])
    for _ in range(4):
        rig.phrase()  # основной yandex жив (пусто), запасной minimax падает на речи - доска красная, речи нет
    assert rig.last_health["minimax"] == "timeout" and rig.last_health["yandex"] == "ok"
    assert rig.spoken == []


def test_primary_stt_timeouts_on_real_speech_three_in_a_row_are_a_failure_once():
    rig = _Rig([_Stt("minimax", exc=STTTimeoutError("t")), _Stt("vosk", "включи музыку пожалуйста")])
    rig.phrase()
    rig.phrase()
    assert rig.spoken == []  # два раза - ещё не отказ
    rig.phrase()
    rig.phrase()
    assert len(rig.spoken) == 1
    assert "minimax" in rig.spoken[0] and "распознавани" in rig.spoken[0] and "vosk" in rig.spoken[0]


def test_acceptance_by_the_provider_resets_the_count():
    rig = _Rig([_Stt("minimax", exc=STTTimeoutError("t")), _Stt("vosk", "включи музыку пожалуйста")])
    rig.phrase()
    rig.phrase()
    rig.set_chain([_Stt("minimax", "включи музыку пожалуйста"), _Stt("vosk", "")])
    rig.phrase()
    rig.set_chain([_Stt("minimax", exc=STTTimeoutError("t")), _Stt("vosk", "включи музыку пожалуйста")])
    rig.phrase()
    rig.phrase()
    assert rig.spoken == []
