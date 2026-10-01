"""Issue #3248 — паттерны mv02/mv03/ml01 проверяют реальное событие, а не дамп.

Run 36775544782 / 36777967495 (30.09.2026): шаги краснели по паттернам, которые
не могли найтись (``current_voice`` пишется только при СМЕНЕ голоса,
``voice_used`` — только при синтезе MiniMax, ``generate_music`` снят с
регистрации 22.08 в db84ff590). Обратная сторона той же беды: голое имя тула
в ``patterns`` матчит строку ``tools(61): …`` — её dialogue_node печатает на
каждом ходе (ml06 run 36777967495: PATTERN_OK gen_get_track_info при НЕвызванном
туле, GATE-1 это поймал).

Строки-образцы ниже — дословно из ``docker logs voice-assistant`` робота за
окна этих шагов.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCENARIOS = ROOT / ".github/e2e/scenarios"

TTS_ALENA = (
    "[tts_node-5] [INFO] [1790801810.759956860] [tts_node]: 🔊 TTS: speech_id=91b3cb4b, "
    "dialogue_id=None, batch=None None/None, voice=alena, lang=default, "
    "text='Привет! Рада слышать, я уже на Алёнке, голос мягче прежнего. Чем займёмся?'"
)
TTS_ERMIL = (
    "[tts_node-5] [INFO] [1790801911.731759463] [tts_node]: 🔊 TTS: speech_id=69a0baf1, "
    "dialogue_id=None, batch=None None/None, voice=ermil, lang=default, text='А это чтобы тебя скорее съесть!'"
)
SPEAK_TEXT_ALENA = (
    "[mcp_server-10] [INFO] [1790801751.132630578] [mcp_server]: [speak_text] Произношение текста: "
    "animation=happy voice=alena text='Говорю голосом Алены через Яндекс!'"
)
COMPOSE_CALL = (
    "[mcp_server-10] [INFO] [1790802853.579022058] [mcp_server]: 📥 Запрос выполнения: compose_music "
    "с параметрами {'bpm': 76, 'root': 'F', 'scale': 'major'}"
)
CATALOG_DUMP = (
    "[dialogue_node-4]   tools(61): add_music_material, clear_waypoints, compose_music, "
    "continue_mapping, execute_music_code, gen_get_track_info, set_voice, speak_text"
)


def _steps(name: str) -> dict:
    data = json.loads((SCENARIOS / name).read_text(encoding="utf-8"))
    return {s["label"]: s for s in data["steps"]}


def _hits(patterns: list[str], line: str) -> bool:
    # check_patterns в e2e_voice_test.sh — grep -E; эти паттерны не используют
    # ничего, что различалось бы между ERE и re.
    return all(re.search(p, line) for p in patterns)


def test_mv02_matches_answer_in_mv01_voice_not_voice_change() -> None:
    pats = _steps("voice_core_suite_v1.json")["mv02_speak_alena"]["patterns"]
    assert pats, "mv02 без паттерна ничего не проверяет"
    assert _hits(pats, TTS_ALENA)
    assert not _hits(pats, TTS_ERMIL), "ответ другим голосом не должен проходить"
    assert not _hits(pats, SPEAK_TEXT_ALENA), "строка mcp_server — не факт синтеза"
    assert not _hits(pats, CATALOG_DUMP)


def test_mv03_requires_real_speak_text_and_set_voice() -> None:
    step = _steps("voice_core_suite_v1.json")["mv03_skazka_raznymi_golosami"]
    assert "voice_used" not in step["patterns"]
    calls = step["acceptance"]["expected_tool_calls"]
    # RULE #VOICE-MULTI: set_voice МЕЖДУ speak_text-фрагментами. Без
    # speak_text реплики персонажей остаются в content и не звучат.
    assert "set_voice" in calls and "speak_text" in calls


def test_ml01_matches_real_music_call_not_catalog_or_dead_tool() -> None:
    pats = _steps("music_library_suite_v1.json")["ml01_generate_romantic"]["patterns"]
    assert "generate_music" not in pats
    assert _hits(pats, COMPOSE_CALL)
    assert _hits(pats, COMPOSE_CALL.replace("compose_music", "execute_music_code"))
    assert not _hits(pats, CATALOG_DUMP), "имя тула в дампе каталога — не вызов"


def test_log_lines_the_patterns_rely_on_still_exist_in_source() -> None:
    tts = (ROOT / "src/rob_box_voice/rob_box_voice/tts_node.py").read_text(encoding="utf-8")
    assert 'f"🔊 TTS: speech_id={speech_id[:8]}, "' in tts
    assert "f'voice={voice or \"default\"}, '" in tts
    mcp = (ROOT / "src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py").read_text(encoding="utf-8")
    assert 'f"📥 Запрос выполнения: {tool_name} с параметрами {parameters}"' in mcp
