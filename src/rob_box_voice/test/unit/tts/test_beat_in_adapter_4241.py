"""#4241 (ADR-0160 PR-4): паузы такта рэпа ставит адаптер голоса, а не диалог.

Путь реплики: ``SpeechRequest`` (голые строки + ``beat_s``) → ``chunk_from_request`` → ``ctx["beat_s"]`` конвейера
``tts_node`` → ``prepare`` адаптера. MiniMax (``pause_markup``) получает паузы ``<#x#>`` по сетке строк с темпом
MiniMax (``rap_pauses.markup``), числа — словами (длина строки по гласным). Yandex и Silero читают без такта и пишут об
этом строку лога (решение Шифу 30.10, В3).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

import pytest  # noqa: E402

from rob_box_core.msg_stub import install_dialog_msgs_stub  # noqa: E402
from rob_box_dialog.plan.rap_pauses import markup  # noqa: E402
from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider  # noqa: E402
from rob_box_voice.core.speech_owner import chunk_from_request  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402
from rob_box_voice.tts_providers import SileroTTSProvider, YandexTTSProvider  # noqa: E402
from rob_box_voice.tts_providers.beat import minimax_text, without_beat  # noqa: E402
from rob_box_voice.tts_text_clean import speech_text  # noqa: E402

M = install_dialog_msgs_stub()
VERSE = "Я робот Робби, я читаю под бит,\nКолёса крутятся, мотор не молчит.\nКаждая строчка в такт звучит."
BAR_S = 240.0 / 131


# ── провод: такт едет полем, текст — голые строки ──────────────────────────────


def test_beat_rides_the_speech_request_and_reaches_the_chunk():
    req = M.SpeechRequest(text=VERSE, speech_id="s1", agent="personality", commit=False, beat_s=BAR_S)
    chunk = chunk_from_request(req)
    assert chunk["beat_s"] == pytest.approx(BAR_S)
    assert chunk["text"] == VERSE and speech_text(chunk["text"]) == VERSE  # строки переживают очистку


def test_no_beat_no_field_in_the_chunk():
    assert "beat_s" not in chunk_from_request(M.SpeechRequest(text="Привет.", speech_id="s2", agent="personality"))


# ── адаптеры ───────────────────────────────────────────────────────────────────


def test_minimax_marks_the_pauses_from_the_beat_grid():
    log = []
    assert minimax_text(VERSE, BAR_S, log.append) == markup(VERSE, BAR_S).text
    assert "<#" in markup(VERSE, BAR_S).text
    assert log and "minimax: такт 1.83 с" in log[0] and "паузы" in log[0]


def test_minimax_spells_numbers_before_counting_the_line():
    """Длина строки — по гласным: «2024» без нормализации — ноль слогов, пауза вышла бы длиннее (ADR-0160 §2.5 п.1)."""
    verse = "Йо, в 2024 енот на бите,\nМоет печенье, считает мусор в корыте."
    got = minimax_text(verse, 2.0, [].append)
    assert "две тысячи двадцать четыре" in got and "<#" in got
    assert got != markup(verse, 2.0).text


def test_minimax_without_beat_or_with_one_line_keeps_the_text():
    log = []
    assert minimax_text(VERSE, 0.0, log.append) == VERSE and log == []
    assert minimax_text("Я робот Робби, я читаю под бит.", 2.0, log.append) == "Я робот Робби, я читаю под бит."
    assert "одна строка" in log[0]


def test_minimax_adapter_flag_is_the_one_that_marks():
    assert MiniMaxTTSProvider(api_key="k", group_id="g", client=MagicMock()).capabilities().pause_markup is True
    assert YandexTTSProvider("k", object(), logger=MagicMock()).capabilities().pause_markup is False
    assert SileroTTSProvider(logger=MagicMock()).capabilities().pause_markup is False


@pytest.mark.parametrize("adapter", [
    lambda log: YandexTTSProvider("fake-key", object(), logger=log),
    lambda log: SileroTTSProvider(logger=log),
])
def test_fallback_reads_without_beat_and_says_so_once(adapter):
    log = MagicMock()
    prepared = adapter(log).prepare(VERSE, BAR_S)
    assert "<#" not in prepared and prepared.startswith("Я робот Робби, я читаю под бит,\nКолёса")
    (line,), _ = log.info.call_args
    assert log.info.call_count == 1 and "рэп без такта" in line and "темп" in line and "не мерен" in line


def test_fallback_without_beat_writes_nothing():
    log = []
    assert without_beat("yandex", VERSE, 0.0, log.append) == VERSE and log == []


# ── конвейер tts_node: такт из ctx доходит до своего адаптера ────────────────────


def _chain_node(chain):
    node = TTSNode.__new__(TTSNode)
    node.provider_chain = chain
    node.cloud_tts_budget_s = 8.0
    node._provider_is_dead = lambda name: False
    node._ros_log = MagicMock()
    node.get_logger = lambda: node._ros_log
    node.publish_state = MagicMock()
    node._sap_reset_provider_fail_streak = MagicMock()
    node._mark_provider_dead = MagicMock()
    node.minimax_voice = "Russian_ReliableMan"
    node.minimax_model = "speech-02-hd"
    node.minimax_streaming = False
    got = {}

    def minimax_http(text, *, voice=None, language=None, budget_deadline=None):
        got["minimax"] = text
        raise RuntimeError("quota")  # квота: аудио нет — дальше Yandex

    def yandex_synth(text, *, voice=None, speed=1.0):
        got["yandex"] = text  # что адаптер Yandex реально отдал сервису (после своего ``prepare``)
        return [0.0]

    node._synthesize_minimax = minimax_http
    node._yandex_tts = YandexTTSProvider("fake-key", object(), logger=node._ros_log)
    node._yandex_tts.synthesize_np = yandex_synth
    return node, got


def test_chain_gives_minimax_the_marks_and_the_yandex_fallback_plain_lines():
    node, got = _chain_node(["minimax", "yandex", "silero"])
    ctx = {"audio_np": None, "sample_rate": 16000, "used_provider": None, "used_voice": None, "result": {},
           "beat_s": BAR_S}
    t0 = time.monotonic()
    node._sap_run_provider_chain(None, None, VERSE, "speaker", ctx)
    assert time.monotonic() - t0 < 1.0
    assert got["minimax"] == markup(VERSE, BAR_S).text
    assert got["yandex"] == VERSE and ctx["used_provider"] == "yandex"
    lines = [c.args[0] for c in node._ros_log.info.call_args_list]
    assert sum("yandex: рэп без такта" in line for line in lines) == 1


def test_chain_without_beat_sends_plain_text_to_minimax():
    node, got = _chain_node(["minimax", "yandex", "silero"])
    ctx = {"audio_np": None, "sample_rate": 16000, "used_provider": None, "used_voice": None, "result": {}}
    node._sap_run_provider_chain(None, None, VERSE, "speaker", ctx)
    assert got["minimax"] == VERSE and "<#" not in got["yandex"]


def test_commit_path_forwards_the_beat_to_the_synthesis():
    """Звучание без готового пред-синтеза (``commit=true``): ``beat_s`` чанка — в воркер и в ``_synthesize_and_play``."""
    import json  # noqa: PLC0415

    node = object.__new__(TTSNode)
    node.get_logger = MagicMock
    node.current_speech_id = None
    node.finished_pub = MagicMock()
    node._submit_synthesis = MagicMock()
    node._pending_speech_queue, node._play_active_seq, node._active_batch_id = [], None, None
    node.provider_chain = ["minimax", "yandex", "silero"]
    node.dialogue_callback(SimpleNamespace(data=json.dumps({"text": VERSE, "speech_id": "s1", "beat_s": BAR_S})))
    assert node._submit_synthesis.call_args.kwargs["beat_s"] == pytest.approx(BAR_S)

    node._synthesize_and_play = MagicMock()
    TTSNode._run_synthesis_worker(node, VERSE, None, "s1", None, None, None, play_seq=0, beat_s=BAR_S)
    assert node._synthesize_and_play.call_args.kwargs["beat_s"] == pytest.approx(BAR_S)


def test_presynth_worker_takes_the_beat_from_the_chunk():
    """Пред-синтез (``commit=false``) — путь сегментов выступления: ``beat_s`` чанка попадает в ``ctx`` цепочки."""
    from rob_box_voice.core.speech_timing import Presynth  # noqa: PLC0415

    seen = {}

    def chain(voice, language, text, sink, ctx):
        seen["beat_s"], seen["text"] = ctx["beat_s"], text
        ctx["audio_np"], ctx["sample_rate"] = [0.0] * 16, 16000

    node = SimpleNamespace(_sap_run_provider_chain=chain, _publish_tts_finished=MagicMock())
    done = MagicMock()
    Presynth()._worker(node, {"speech_id": "s1", "text": VERSE, "beat_s": BAR_S}, done, lambda _sid: False)
    assert seen == {"beat_s": BAR_S, "text": VERSE} and done.called
