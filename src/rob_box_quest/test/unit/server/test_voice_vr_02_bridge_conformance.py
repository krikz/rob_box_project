"""Conformance-тест контракта мостика webxr_client ↔ ws_server.

Зачем (issue #2187, ADR-0080 §2.2 инвариант 3):
    webxr_client (TS) шлёт команды через JSON_CMD{cmd: "..."}.
    Сервер ``WSSServer._on_json_cmd`` их диспатчит по имени cmd.
    Контракт: каждое имя cmd, которое клиент реально использует, должно
    иметь обработчик на сервере. Иначе — молчаливый дроп (как
    ``avatar_set_mode`` сейчас: клиент шлёт, сервер игнорирует, кнопки
    режима на мостике не работают, см. [voice-vr 09]).

Что фиксирует:
    - [voice-vr 09 / issue #2194] ЗАКРЫТ: сервер теперь обрабатывает
      ``avatar_set_mode``, ``avatar_acquire_floor``, ``avatar_release_floor``
      (см. ``ws_server.py`` — ветки avatar_* рядом с legacy supervisor_*).
      ``xfail(strict=True)`` с теста снят — это больше не честный FAIL по
      ADR-0018, а обычное зелёное утверждение контракта.

Что НЕ проверяет (out of scope):
    - Каталог протокола [voice-vr 07] — этот тест переживёт его появление
      и будет читать каталог вместо грепа.
    - Сами обработчики (payload schema, FSM-переходы и т.п.) — это
      функциональные тесты вроде ``test_voice_floor_e2e_full_flow``.

События (JSON_EVENT) сверяются симметрично — но в режиме ОТЧЁТА
(печать diff в pytest -v, без гейта), потому что список серверных
событий богаче, чем фактические хендлеры в ``_on_json_event`` (часть
отправляется из других методов вроде ``_send(stream_list)``).

Запуск:
    cd src/rob_box_quest && PYTHONPATH=. python3 -m pytest \\
        test/unit/server/test_voice_vr_02_bridge_conformance.py -v
"""

from __future__ import annotations

import ast
import re
from pathlib import Path


# --- Пути (относительно src/rob_box_quest/test/unit/server/<file>) ----------

REPO_ROOT = Path(__file__).resolve().parents[4]  # ... → src/
WEBXR_CLIENT_SRC = REPO_ROOT / "rob_box_quest" / "webxr_client" / "src"
WIRE_MESSAGES_TS = WEBXR_CLIENT_SRC / "wire" / "messages.ts"
WS_SERVER_PY = REPO_ROOT / "rob_box_quest" / "rob_box_quest" / "server" / "ws_server.py"


# --- Парсеры ----------------------------------------------------------------

_CLIENT_CMD_LITERAL = re.compile(r'\bcmd:\s*"([a-z_][a-z0-9_]*)"')
_TS_EVENT_LITERAL = re.compile(r'\btype:\s*"([a-z_][a-z0-9_]*)"')


_GENERATED_FILE_MARKER = "GENERATED FILE"


def _parse_client_used_commands() -> set[str]:
    """Имена ``cmd: "..."``, которые клиент реально шлёт (grep по .ts).

    Соответствует формулировке задачи «собирать имена команд из
    TS-исходников (``cmd: "..."`` в ``webxr_client/src``)».

    [voice-vr 09 / issue #2194 доп. фикс]: наивный grep не отличает
    ОБЪЯВЛЕНИЕ типа команды (``interface FooCmd { cmd: "foo"; ... }``) от
    места, где команда реально уходит на сервер. Раньше единственным
    источником таких объявлений считался ``wire/messages.ts`` — но после
    того как каталог протокола (см. tools/gen_bridge_protocol_ts.py,
    [voice-vr 07]) сгенерировал ``wire/protocol_generated.ts``, именно
    ЭТОТ файл стал содержать per-cmd интерфейсы (``cmd: "set_panel_topic";``
    и т.п.), а ``messages.ts`` лишь реэкспортирует их union и не матчится
    регэкспом сам по себе.
    Мы НЕ можем просто сузить парсер до буквальных ``conn.send({cmd: ...})``
    сайтов (альтернативный вариант починки) — часть команд (например,
    ``avatar_acquire_floor`` / ``avatar_release_floor`` в main.ts) строится
    в отдельной функции и возвращается как ``JsonCmd``, а отправляется уже
    в вызывающем коде через переменную; такое сужение потеряло бы реальные
    команды и вернуло бы ложноотрицательный результат (тест перестал бы
    ловить реальный баг [voice-vr 09]).
    Поэтому исключаем из обхода файлы, помеченные как сгенерированные
    (маркер ``GENERATED FILE`` в шапке, см. protocol_generated.ts) — они по
    определению содержат только объявления типов, а не код отправки.
    """
    names: set[str] = set()
    for ts_file in WEBXR_CLIENT_SRC.rglob("*.ts"):
        text = ts_file.read_text(encoding="utf-8")
        if _GENERATED_FILE_MARKER in text[:200]:
            continue
        names.update(_CLIENT_CMD_LITERAL.findall(text))
    return names


def _parse_server_cmd_handlers() -> set[str]:
    """Имена cmd, которые сервер реально диспатчит.

    До voice-vr 10 (issue #2195) это был AST-обход if-цепочки в
    ``_on_json_cmd`` (``if cmd == "..."`` / ``if cmd in (...)``). Тот
    рефакторинг свёл ``_on_json_cmd`` к терминальному dispatcher'у
    (``JSON_CMD_HANDLERS.get(cmd)``), поэтому статический AST-обход
    функции больше ничего не находит — веток там просто нет.

    ``JSON_CMD_HANDLERS`` не объявлен как единый dict-литерал (сначала
    пустой dict на уровне модуля, затем ``.update({...})`` ниже, рядом
    с самими ``_json_cmd_*`` хендлерами) — статически парсить его AST
    было бы не проще и более хрупко, чем импортировать модуль и читать
    ключи напрямую. Тот же приём уже используется в
    ``test_voice_vr_07_catalog_conformance.py::_server_dispatched_cmds_from_ws_server``
    (см. issue #2195 в её докстринге) — тот же контракт: каждая
    зарегистрированная cmd = один обработчик на сервере.
    """
    from rob_box_quest.server.ws_server import JSON_CMD_HANDLERS

    return set(JSON_CMD_HANDLERS.keys())


def _parse_server_event_handlers() -> set[str]:
    """AST-обход WSSServer._on_json_event: имена event_type с веткой."""
    src = WS_SERVER_PY.read_text(encoding="utf-8")
    tree = ast.parse(src)

    class Finder(ast.NodeVisitor):
        def __init__(self) -> None:
            self.events: set[str] = set()

        def visit_FunctionDef(self, node) -> None:
            if node.name == "_on_json_event":
                self._scan(node)

        def visit_AsyncFunctionDef(self, node) -> None:
            if node.name == "_on_json_event":
                self._scan(node)

        def _scan(self, fn_node) -> None:
            for stmt in ast.walk(fn_node):
                if not isinstance(stmt, ast.If):
                    continue
                test = stmt.test
                if (
                    isinstance(test, ast.Compare)
                    and len(test.ops) == 1
                    and isinstance(test.ops[0], ast.Eq)
                    and len(test.comparators) == 1
                    and isinstance(test.left, ast.Name)
                    and test.left.id == "event_type"
                    and isinstance(test.comparators[0], ast.Constant)
                    and isinstance(test.comparators[0].value, str)
                ):
                    self.events.add(test.comparators[0].value)

    finder = Finder()
    finder.visit(tree)
    return finder.events


def _parse_ts_event_union() -> set[str]:
    """Имена ``type: "..."`` в TS-юнионе ``JsonEvent`` — события,
    которые клиент УМЕЕТ читать (т.е. union-каталог серверных событий)."""
    if not WIRE_MESSAGES_TS.exists():
        return set()
    text = WIRE_MESSAGES_TS.read_text(encoding="utf-8")
    match = re.search(r"export type JsonEvent\b", text)
    if not match:
        return set()
    tail = text[match.start():]
    return set(_TS_EVENT_LITERAL.findall(tail))


# --- Тест --------------------------------------------------------------------

def test_client_cmds_have_server_handlers() -> None:
    """Каждая команда из webxr_client должна иметь ветку в _on_json_cmd.

    Проверяет инвариант «мостик не теряет команды молча» (ADR-0080 §2.2,
    инвариант 3). [voice-vr 09 / issue #2194] закрыл серверную сторону
    (avatar_set_mode / avatar_acquire_floor / avatar_release_floor теперь
    обрабатываются) — маркер ``xfail(strict=True)`` снят, тест стал
    обычным зелёным утверждением контракта.
    """
    client_cmds = _parse_client_used_commands()
    server_cmds = _parse_server_cmd_handlers()

    missing = sorted(client_cmds - server_cmds)
    assert not missing, (
        "Клиент шлёт команды без обработчика на сервере "
        "(молчаливый дроп — см. voice-vr 09):\n"
        + "\n".join(f"  - {cmd!r}" for cmd in missing)
    )


def test_server_handlers_have_client_user() -> None:
    """Симметричный sanity-чек: серверные обработчики, которые не
    используются клиентом — вероятно мёртвый код.

    Допускается исключение для команд, отправляемых старыми версиями
    клиента (обратная совместимость). Здесь — точная сверка; если
    позже понадобится whitelist — расширим через ``--ignore`` или
    общий known-set в conftest.
    """
    client_cmds = _parse_client_used_commands()
    server_cmds = _parse_server_cmd_handlers()

    # supervisor_* — серверный API, рассчитан на внешних клиентов
    # (см. ADR-0028). Допустимо, что webxr_client их не шлёт напрямую:
    # мостик их форвардит. Исключаем из dead-code списка.
    KNOWN_SERVER_ONLY = {
        "supervisor_set_mode",
        "supervisor_acquire_floor",
        "supervisor_release_floor",
        "supervisor_get_state",
        # voice_listen_start/stop — старые названия; клиент сейчас не
        # шлёт их через union (используется voice_ptt_*), но сервер
        # поддерживает для обратной совместимости (см. ws_server.py:1864).
        "voice_listen_start",
        "voice_listen_stop",
        # stream_select (closes #2236): клиент шлёт из main.ts
        # `onPanelTopicChange` после клика по строке меню
        # (scene/stream_menu.ts: `topicFromTargetId` → applyMenuChoice →
        # switchStream → callback). Серверный обработчик — мета-команда:
        # проверяет топик в registry и возвращает stream_select_ack.
        # До #2236 клиент только менял локальный стор панели, и эта
        # запись в KNOWN_SERVER_ONLY была честной отметкой о шве; теперь
        # шов закрыт и исключение больше не нужно.
    }
    suspicious = sorted(server_cmds - client_cmds - KNOWN_SERVER_ONLY)
    assert not suspicious, (
        "Серверные обработчики, которые не использует webxr_client "
        "(возможный мёртвый код; либо добавьте в KNOWN_SERVER_ONLY):\n"
        + "\n".join(f"  - {cmd!r}" for cmd in suspicious)
    )


def test_events_diff_report() -> None:
    """Сверка событий в режиме ОТЧЁТА — НЕ падаем.

    Серверная сторона шлёт много типов JSON_EVENT (stream_list, pong,
    voice_list, floor_lost, tars1_text, tars_panel_url, ...), но
    ``_on_json_event`` обрабатывает только ``ping`` — остальные
    отправляются из специализированных хендлеров (``stream_select`` →
    ``_send(stream_select_ack)``, мост → ``_send(voice_list)`` и т.п.).

    Это значит, что простая сверка «что _on_json_event знает vs что
    клиент ждёт» даст ложноположительные «недостающие обработчики».
    Поэтому здесь — отчёт через ``print`` + ``pytest -v`` (появляется
    в captured stdout), без гейта. Когда появится каталог [voice-vr 07],
    эту проверку можно будет ужесточить.
    """
    server_events = _parse_server_event_handlers()
    client_events = _parse_ts_event_union()

    print("\n--- voice-vr 02 events diff (informational) ---")
    print(f"server _on_json_event handles: {sorted(server_events)}")
    print(f"client JsonEvent union:        {sorted(client_events)}")
    only_server = sorted(server_events - client_events)
    only_client = sorted(client_events - server_events)
    if only_server:
        print(f"server-only events (not in client union): {only_server}")
    if only_client:
        print(f"client-only events (not in server handler): {only_client}")
    print("--- end events diff ---\n")
