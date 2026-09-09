"""test_e2e_tool_match.py — «тул вызван» vs «тул доступен» (run 34408526453).

Фикстуры ниже — НЕ выдуманные строки, а фрагменты живого
``docker logs voice-assistant`` с робота (Vision Pi, 09.09.2026, акт 1
ночного марафона). Именно на них старый подстрочный матчер давал:

  * ``expected_tool_calls`` PASS всегда — имя любого тула есть в строке
    ``tools(56): ...``, которую dialogue_node печатает на каждом ходе;
  * ``must_not_call`` FAIL всегда — по той же причине.

Замер на том прогоне (1065 строк лога): голое имя ``stop_music`` — 17
совпадений при НУЛЕ реальных вызовов; в кавычках — 0.

Run:
  python3 -m pytest tests/unit/e2e_scripts/test_e2e_tool_match.py -v --no-cov
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from e2e_tool_match import tool_invoked  # noqa: E402


# --- живые фрагменты лога ---------------------------------------------------

#: Строка со списком ДОСТУПНЫХ тулов. Печатается на каждом ходе перед
#: запросом к LLM. Здесь нет ни одного вызова — только каталог.
TOOLS_CATALOG = (
    "[dialogue_node-4]   tools(56): clear_waypoints, compose_music, "
    "continue_mapping, delete_track, delete_waypoint, estimate_tts_duration, "
    "execute_music_code, faq_search, finish_mapping, gen_delete_from_library, "
    "get_battery_level, get_current_time, get_music_state, move_direction, "
    "play_animation, play_sound, register_speaker, save_track, set_dj_mode, "
    "set_voice, set_volume, speak_text, start_mapping, stop_music, "
    "stop_navigation, task_delta, load_skill\n"
)

#: Ход, в котором робот НЕ вызвал ни одного тула. Реальный n110 марафона:
#: на вопрос «у тебя сейчас играет музыка?» ответил голосом, состояние
#: плеера не спросил.
TURN_NO_TOOLS = TOOLS_CATALOG + (
    "[dialogue_node-4] ✅ [turn] process_input returned: spoken='нет'[:60] "
    "tools=[] finish_reason='stop' truncated_tool_args=False error=None\n"
)

#: Ход с РЕАЛЬНЫМ вызовом get_current_time (+ speak_text).
TURN_WITH_CALL = TOOLS_CATALOG + (
    "[dialogue_node-4] 📤 Отправлен запрос b5119339: get_current_time\n"
    "[mcp_server-10] 📥 Запрос выполнения: get_current_time с параметрами {}\n"
    "[mcp_server-10] 📤 Публикую результат для get_current_time (request_id: b5119339)\n"
    "[mcp_server-10] ✅ Инструмент get_current_time выполнен успешно\n"
    "[dialogue_node-4]   [3] assistant: '' tool_calls=(ToolCall("
    "id='call_01a0', name='get_current_time', arguments=mappingproxy({})),)\n"
    "[dialogue_node-4] ✅ [turn] process_input returned: spoken='00:47'[:60] "
    "tools=['get_current_time', 'speak_text'] finish_reason='stop'\n"
)


class TestNotInvoked:
    """Каталог доступных тулов НЕ должен считаться вызовом."""

    @pytest.mark.parametrize(
        "tool",
        ["stop_music", "execute_music_code", "move_direction", "clear_waypoints",
         "get_music_state", "set_dj_mode", "register_speaker"],
    )
    def test_catalog_alone_is_not_a_call(self, tool):
        assert tool_invoked(TOOLS_CATALOG, tool) is False

    @pytest.mark.parametrize("tool", ["stop_music", "execute_music_code", "get_music_state"])
    def test_turn_without_tools_is_not_a_call(self, tool):
        # Это в точности регресс n110 акта 1: acceptance сообщал
        # «forbidden tool calls invoked: ['stop_music', 'execute_music_code']»
        # на ходе, где tools=[].
        assert tool_invoked(TURN_NO_TOOLS, tool) is False

    def test_expected_call_no_longer_passes_vacuously(self):
        # Обратная сторона той же монеты: GATE-1 печатал «all checks passed»,
        # потому что имя ожидаемого тула всегда было в каталоге.
        assert tool_invoked(TURN_NO_TOOLS, "get_music_state") is False


class TestInvoked:
    """Каждый маркер реального вызова должен срабатывать сам по себе."""

    def test_full_turn_with_call(self):
        assert tool_invoked(TURN_WITH_CALL, "get_current_time") is True

    def test_other_tools_in_same_turn_stay_uncalled(self):
        assert tool_invoked(TURN_WITH_CALL, "stop_music") is False
        assert tool_invoked(TURN_WITH_CALL, "move_direction") is False

    @pytest.mark.parametrize(
        "line",
        [
            "tools=['get_current_time', 'speak_text']",
            "ToolCall(id='x', name='get_current_time', arguments={})",
            "📥 Запрос выполнения: get_current_time с параметрами {}",
            "✅ Инструмент get_current_time выполнен успешно",
            "📤 Публикую результат для get_current_time (request_id: b5)",
        ],
    )
    def test_each_marker_alone(self, line):
        assert tool_invoked(line, "get_current_time") is True

    def test_case_insensitive(self):
        assert tool_invoked("TOOLS=['GET_CURRENT_TIME']", "get_current_time") is True


class TestFreeTextFallback:
    """must_not_call используется и для кусков строк лога, не только для
    имён тулов (напр. «добавка куплета не оборвала песню»). Этот способ
    ломать нельзя — для не-имён остаётся обычный подстрочный поиск."""

    @pytest.mark.parametrize(
        "frag",
        ["STOP command received", "Cancel: new STT input", "Воспроизведение прервано",
         "session reset", "[backlog] flushed to LLM"],
    )
    def test_free_text_is_substring_matched(self, frag):
        assert tool_invoked("... %s ..." % frag, frag) is True
        assert tool_invoked("ничего похожего", frag) is False

    def test_mixed_case_free_text(self):
        assert tool_invoked("🔇 stop command received - остановка", "STOP command received") is True


class TestEdges:
    def test_empty_and_none(self):
        assert tool_invoked("любой лог", "") is False
        assert tool_invoked("любой лог", None) is False

    def test_whitespace_is_stripped(self):
        assert tool_invoked(TURN_WITH_CALL, "  get_current_time  ") is True

    def test_substring_tool_names_do_not_bleed(self):
        # 'save_track' не должен срабатывать от 'gen_save_to_library' и наоборот.
        log = "tools=['gen_save_to_library']"
        assert tool_invoked(log, "gen_save_to_library") is True
        assert tool_invoked(log, "save_track") is False
