"""Паузы MiniMax ``<#x#>`` выступления (#3901, О11) доходят до MiniMax как есть, а фолбеку — сняты.

Путь реплики: ``SpeechRequest.text`` → ``chunk_from_request`` (``Utterance.ssml``: экранирование ``<``/``>``) →
``_extract_text_from_ssml`` (теги прочь, entity назад) → цепочка провайдеров. Метка ``<#1.06#>`` — не тег (имя не с
буквы), поэтому переживает и сборку SSML, и разбор; Yandex и Silero её не понимают — им текст без меток.
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

from rob_box_dialog.plan.rap_pauses import markup  # noqa: E402
from rob_box_voice.core.speech_owner import chunk_from_request  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402

VERSE = "Я робот Робби, я читаю под бит,\nКолёса крутятся, мотор не молчит.\nКаждая строчка в такт звучит."
MARKED = markup(VERSE, 240.0 / 131).text


def _request(text: str) -> SimpleNamespace:
    return SimpleNamespace(text=text, agent="personality", epoch=0, turn_id="t", interruptible=True, at=0.0,
                           boundary=1, commit=False, sink="", priority=0, speech_id="s1", emotion="", voice="",
                           language="")


def test_pause_marks_survive_ssml_and_its_parsing():
    assert "<#" in MARKED
    ssml = chunk_from_request(_request(MARKED))["ssml"]
    assert "&lt;#" in ssml  # экранированы в SSML, а не выброшены как тег
    assert TTSNode.__new__(TTSNode)._extract_text_from_ssml(ssml) == MARKED


def _chain_node(chain):
    node = TTSNode.__new__(TTSNode)
    node.provider_chain = chain
    node.cloud_tts_budget_s = 8.0
    node._provider_is_dead = lambda name: False
    node.get_logger = MagicMock
    got = {}

    def minimax(voice, language, attrs, text, sink, provider_chain, ctx, budget_deadline=None):
        got["minimax"] = text  # квота: аудио нет — дальше Yandex

    def yandex(voice, language, attrs, text, ctx):
        got["yandex"] = text
        ctx["audio_np"] = [0.0]

    node._sap_synthesize_minimax = minimax
    node._sap_synthesize_yandex = yandex
    return node, got


def test_minimax_gets_the_marks_and_yandex_fallback_gets_them_stripped():
    node, got = _chain_node(["minimax", "yandex", "silero"])
    ctx = {"audio_np": None, "sample_rate": 16000, "used_provider": None, "used_voice": None, "result": {}}
    t0 = time.monotonic()
    node._sap_run_provider_chain(None, None, {}, MARKED, "speaker", ctx)
    assert time.monotonic() - t0 < 1.0
    assert got["minimax"] == MARKED
    assert "<#" not in got["yandex"] and got["yandex"].startswith("Я робот Робби, я читаю под бит, Колёса")
