"""Ответ тула в окне модели ограничен: один поиск в интернете не должен стирать историю разговора.

Живой замер 08.11 на колонке (Игорёк, вторая Станция Лайт):

    окно после хода: записей 3, токенов 3994      ← ход с search_web
    окно до хода:    записей 3, токенов 3994      ← бюджет 3000 уже превышен
    окно после хода: записей 2, токенов 76        ← прошлый ход выброшен целиком

После этого на вопрос «а ты что думаешь?» модель видела только системный промпт и текущую фразу: «По какому
поводу, бля? Ты в воздух базаришь». Причина — полный ответ тула лежал в окне и один занимал больше бюджета.
"""
import json
from types import MappingProxyType

from rob_box_dialog import knowledge as kn
from rob_box_dialog.context import build_context
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.turn import RECENT_ACTIONS, recent_results
from rob_box_dialog.turn_log import TurnLog


def _search_payload(results: int = 5) -> MappingProxyType:
    return MappingProxyType({"query_results": [{"tool": "search_web", "status": "done",
                                                "data": {"query": "новости", "max_results": results,
                                                         "results": [{"title": f"Заголовок {i}",
                                                                      "url": f"https://example.com/{i}",
                                                                      "snippet": "текст новости " * 12}
                                                                     for i in range(results)]}}]})


def _turn_with_search(log: TurnLog, turn_id: str, ask: str, answer: str, results: int = 5) -> None:
    log.begin(turn_id, 1, ask)
    log.result(turn_id, ActionResult(f"{turn_id}/1", 1, "voice", "search_web", "perceive", "done",
                                     data=_search_payload(results)))
    log.said(turn_id, f"s-{turn_id}", answer)
    log.spoken(f"s-{turn_id}", len(answer))


def test_search_payload_does_not_evict_the_history():
    """Ход с поиском влезает в окно: следующий ход видит предыдущий разговор."""
    log = TurnLog()
    _turn_with_search(log, "t1", "Игорёк, ты тут?", "Тут я.")
    _turn_with_search(log, "t2", "Игорёк, что там про Макрона?", "Пишут про бункер.")

    window = log.window(exclude="t3")
    texts = [e.text() for e in window]

    assert window, "окно пустое: история выброшена"
    assert any("ты тут?" in t for t in texts), texts
    assert log.tokens() <= kn.TURN_WINDOW_MAX_TOKENS, log.tokens()


def test_big_tool_data_compressed_with_marker():
    """Ответ тула сжимается по структуре: урезанные списки и строки, пометка «сколько было»."""
    log = TurnLog()
    _turn_with_search(log, "t1", "Игорёк, что там про Макрона?", "Пишут про бункер.")

    data = next(e.content for e in log.window() if e.role == "tool")["data"]
    results = data["query_results"][0]["data"]["results"]

    assert data["truncated"]["chars"] > kn.WINDOW_TOOL_DATA_CHARS
    assert len(results) == kn.WINDOW_TOOL_ITEMS + 1, results  # 3 результата + пометка об остатке
    assert results[-1] == {"omitted": 5 - kn.WINDOW_TOOL_ITEMS}
    assert results[0]["snippet"].endswith("…") and len(results[0]["snippet"]) == kn.WINDOW_TOOL_TEXT_CHARS + 1
    entry = next(e for e in log.window() if e.role == "tool")
    assert len(entry.text()) <= kn.WINDOW_TOOL_DATA_CHARS + 300, len(entry.text())


def test_compressed_tool_data_is_json_safe_in_the_next_turn_snapshot_4688():
    """Сжатый mappingproxy-результат прошлого хода сериализуется в ``recent_actions`` следующего хода."""
    log = TurnLog()
    _turn_with_search(log, "t1", "Игорёк, что ты видишь?", "Вижу мастерскую.")

    recent = recent_results(log.window(exclude="t2"), (), owner=None)
    snapshot = build_context({RECENT_ACTIONS: recent})

    assert '"tool":"search_web"' in snapshot and '"truncated"' in snapshot


def test_small_tool_data_kept_as_is():
    """Малый ответ тула не трогаем — модель видит его целиком."""
    log = TurnLog()
    log.begin("t1", 1, "Игорёк, который час?")
    log.result("t1", ActionResult("t1/1", 1, "voice", "get_current_time", "perceive", "done",
                                  data=MappingProxyType({"time": "16:58"})))

    data = next(e.content for e in log.window() if e.role == "tool")["data"]

    assert dict(data) == {"time": "16:58"}


def test_window_tokens_stay_under_budget_after_several_search_turns():
    """Три хода с поиском подряд — окно всё ещё в бюджете и держит хотя бы последние ходы."""
    log = TurnLog()
    for i in range(3):
        _turn_with_search(log, f"t{i}", f"вопрос {i}", f"ответ {i}")

    assert log.tokens() <= kn.TURN_WINDOW_MAX_TOKENS, log.tokens()
    assert any("вопрос 2" in e.text() for e in log.window()), "последний ход должен остаться"
    assert list(log.window())[0].text()  # запись сериализуется для модели
