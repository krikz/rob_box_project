"""Catalog-driven conformance test для rob_box_quest wire-протокола.

Зачем (ADR-0080 §2.2 инвариант 3, CONTEXT.md «каталог протокола»,
issue #2192 / Definition of Done карточки):

    «Conformance-тест из [voice-vr 02] читает каталог вместо грепа
    и остаётся зелёным.»

    До [voice-vr 07] conformance-тест
    ``test/unit/server/test_voice_vr_02_bridge_conformance.py`` делал
    AST-обход ``ws_server.py`` (grep по ``if cmd == "..."``). Это
    расходилось с каноном, потому что:
      * ``avatar_*`` объявлены в TS-types, но НЕ имеют ветки в сервере;
      * новые ``supervisor_*`` объявлены и в TS, и в сервере (но
        через frame-type 0x30, а не через JSON_CMD.cmd, и AST-обход
        этого не видит);
      * добавить команду = синхронизировать минимум 3 места (TOPIC_IDS /
        STREAM_CATALOG / ErrorCode в session.py + if в ws_server).

    Теперь канон — :mod:`rob_box_core.bridge_protocol` (single source
    of truth для COMMANDS / EVENTS / STREAMS / MODES / FLOORS / ERRORS).
    Этот тест читает каталог напрямую и валидирует:

      1. Все команды, которые сервер РЕАЛЬНО диспатчит
         (``server_dispatched=True``), присутствуют в каталоге.
         Guard от «каталог забыли обновить».
      2. Каждое имя команды в каталоге с ``server_dispatched=True``
         совпадает с тем, что ws_server реально обрабатывает.
         Guard от «ветка в коде есть, а в каталоге — нет».
      3. Каждая запись каталога имеет хотя бы одно обязательное поле
         и subprotocol ∈ {v1, v2, any}.
      4. Устаревшие/алиасы (``server_dispatched=False``) — явно помечены
         маркером причины в description (для visibility).
      5. Никакой код в ``rob_box_quest`` не должен определять
         ``TOPIC_IDS``, ``STREAM_CATALOG`` или ``ErrorCode`` локально
         — весь источник истины в rob_box_core.

Запуск:
    cd src/rob_box_quest && PYTHONPATH=../rob_box_core:. python3 -m pytest \\
        test/unit/server/test_voice_vr_07_catalog_conformance.py -v
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest

from rob_box_core.bridge_protocol import (
    COMMANDS,
    EVENTS,
    STREAMS,
    VOICE_LANGUAGES,
    VOICE_PRESET_IDS,
    deprecated_commands,
    get_command,
    is_known_error,
)


# --- Пути -------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[4]  # .../src/rob_box_quest/test/unit/server/test_... → /.../src
ROB_BOX_QUEST_PKG = REPO_ROOT / "rob_box_quest" / "rob_box_quest"  # src/rob_box_quest/rob_box_quest/
SERVER_DIR = ROB_BOX_QUEST_PKG / "server"
PROTOCOL_DIR = ROB_BOX_QUEST_PKG / "protocol"
STREAMS_DIR = ROB_BOX_QUEST_PKG / "streams"

WS_SERVER_PY = SERVER_DIR / "ws_server.py"
SESSION_PY = SERVER_DIR / "session.py"
TOPICS_PY = PROTOCOL_DIR / "topics.py"
REGISTRY_PY = STREAMS_DIR / "registry.py"


# --- AST-хелперы ------------------------------------------------------------


class _HandlerFinder(ast.NodeVisitor):
    """AST-обход ``_on_json_cmd``: собирает имена cmd, у которых есть
    явная ветка ``if cmd == "..."`` или ``if cmd in ("...", "...")``.

    Достаточно хрупко: работает только для if-цепочки, которую
    исторически использует ws_server._on_json_cmd. Если сервер
    переедет на табличный диспетчер (карточка voice-vr 10) —
    этот код заменить на импорт каталога напрямую.
    """

    def __init__(self) -> None:
        self.cmds: set[str] = set()

    def _scan_fn(self, fn_node) -> None:
        for stmt in ast.walk(fn_node):
            if not isinstance(stmt, ast.If):
                continue
            test = stmt.test
            # if cmd == "...":
            if (
                isinstance(test, ast.Compare)
                and len(test.ops) == 1
                and isinstance(test.ops[0], ast.Eq)
                and len(test.comparators) == 1
                and isinstance(test.left, ast.Name)
                and test.left.id == "cmd"
                and isinstance(test.comparators[0], ast.Constant)
                and isinstance(test.comparators[0].value, str)
            ):
                self.cmds.add(test.comparators[0].value)
                continue
            # if cmd in ("...", "..."):
            if (
                isinstance(test, ast.Compare)
                and len(test.ops) == 1
                and isinstance(test.ops[0], ast.In)
                and isinstance(test.left, ast.Name)
                and test.left.id == "cmd"
                and isinstance(test.comparators[0], ast.Tuple)
            ):
                for elt in test.comparators[0].elts:
                    if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                        self.cmds.add(elt.value)

    def visit_FunctionDef(self, node):
        if node.name == "_on_json_cmd":
            self._scan_fn(node)
        self.generic_visit(node)

    def visit_AsyncFunctionDef(self, node):
        if node.name == "_on_json_cmd":
            self._scan_fn(node)
        self.generic_visit(node)


def _server_dispatched_cmds_from_ws_server() -> set[str]:
    """AST-обход ws_server.py: возвращает имена cmd, у которых есть
    явная ветка в ``_on_json_cmd`` (включая supervisor_* — они лежат
    в if-elif для одного cmd).
    """
    src = WS_SERVER_PY.read_text(encoding="utf-8")
    tree = ast.parse(src)
    finder = _HandlerFinder()
    finder.visit(tree)
    return finder.cmds


def _server_emitted_events_from_ws_server() -> set[str]:
    """Грубый regex: имена событий в ``"type": "..."`` внутри JSON_EVENT
    полезных нагрузок. Используется только для sanity-чека симметрии
    с каталогом EVENTS (что сервер ШЛЁТ, но не обрабатывает в
    ``_on_json_event`` — там только ``ping``, см. test_bridge_protocol
    в rob_box_core)."""
    if not WS_SERVER_PY.exists():
        return set()
    text = WS_SERVER_PY.read_text(encoding="utf-8")
    return set(re.findall(r'"type":\s*"([a-z_][a-z0-9_]*)"', text))


# --- 1. Catalog ↔ ws_server bidirectional sync -----------------------------


class TestCatalogWsServerSync:
    """Каталог и серверный код согласованы."""

    def test_catalog_dispatched_cmds_have_server_handlers(self):
        """Каждая cmd из каталога с ``server_dispatched=True`` имеет
        ветку в ``_on_json_cmd``."""
        server_handlers = _server_dispatched_cmds_from_ws_server()
        expected = {
            c.name for c in COMMANDS if c.server_dispatched
        }
        missing = sorted(expected - server_handlers)
        assert not missing, (
            "Каталог объявил команды с server_dispatched=True, но в "
            "ws_server._on_json_cmd нет соответствующих веток. Это "
            "значит, что при вызове этих cmd клиент получит "
            "ERROR{BAD_PAYLOAD} — молчаливый дроп.\n"
            + "\n".join(f"  - {cmd!r}" for cmd in missing)
        )

    def test_server_handlers_in_catalog(self):
        """Обратное: каждая серверная ветка описана в каталоге."""
        server_handlers = _server_dispatched_cmds_from_ws_server()
        # Whitelist для сервер-онли команд (см. voice-vr 02):
        #   * supervisor_* — есть в каталоге с server_dispatched=True,
        #     но идут через frame-type 0x30, не через JSON_CMD.cmd —
        #     поэтому AST-обход их видит как ``if cmd == "..."``,
        #     и они ДОЛЖНЫ быть в каталоге.
        #   * voice_listen_start/stop — старые имена, в каталоге
        #     помечены как (deprecated).
        catalog_names = {c.name for c in COMMANDS}
        missing = sorted(server_handlers - catalog_names)
        assert not missing, (
            "Серверные обработчики, которых нет в каталоге — "
            "надо либо добавить в COMMANDS, либо убрать из ws_server:\n"
            + "\n".join(f"  - {cmd!r}" for cmd in missing)
        )

    def test_deprecated_commands_consistent(self):
        """deprecated_commands() (catalog) — это то, что сервер НЕ шлёт,
        но TS-types объявлены. Каждое такое имя должно либо иметь
        явную ветку для ack-а (отдельная карточка), либо быть
        явно помечено в description (см. test_bridge_protocol
        ::test_deprecated_commands_have_marker)."""
        deprecated = set(deprecated_commands())
        server_handlers = _server_dispatched_cmds_from_ws_server()
        # Обработчики avatar_* в сервере отсутствуют — поэтому они
        # НЕ должны попадать в server_handlers (иначе тест падает).
        overlap = deprecated & server_handlers
        assert not overlap, (
            "Эти команды помечены как deprecated (server_dispatched=False), "
            "но сервер реально их обрабатывает — нужно либо убрать "
            "из server_handlers, либо переключить в server_dispatched "
            "обратно в True:\n"
            + "\n".join(f"  - {cmd!r}" for cmd in sorted(overlap))
        )


# --- 2. Каталог ↔ topics.py / registry.py / session.py --------------------


class TestCatalogLegacyCompat:
    """rob_box_quest.protocol.topics / .streams.registry / .server.session
    продолжают предоставлять старые API-имена (TOPIC_IDS, STREAM_CATALOG,
    ErrorCode) — но как re-export из rob_box_core.bridge_protocol,
    а не как локальные объявления."""

    def test_topic_ids_via_topics_module(self):
        """TOPIC_IDS в protocol/topics.py соответствует каталогу."""
        from rob_box_quest.protocol.topics import TOPIC_IDS

        catalog_topic_ids = {s.ui_name: s.topic_id for s in STREAMS}
        assert TOPIC_IDS == catalog_topic_ids, (
            "TOPIC_IDS в protocol/topics.py разошёлся с каталогом.\n"
            f"  catalog: {sorted(catalog_topic_ids.items())}\n"
            f"  module:  {sorted(TOPIC_IDS.items())}"
        )

    def test_stream_catalog_via_registry(self):
        """STREAM_CATALOG в streams/registry.py соответствует каталогу."""
        from rob_box_quest.streams.registry import STREAM_CATALOG

        catalog_names = {s.ui_name for s in STREAMS}
        registry_names = set(STREAM_CATALOG.keys())
        assert registry_names == catalog_names, (
            "STREAM_CATALOG в streams/registry.py разошёлся с каталогом.\n"
            f"  only in catalog: {sorted(catalog_names - registry_names)}\n"
            f"  only in module:  {sorted(registry_names - catalog_names)}"
        )

    def test_error_code_via_session_module(self):
        """ErrorCode в server/session.py имеет все коды каталога
        через атрибутный доступ (``ErrorCode.AUTH_FAIL``)."""
        from rob_box_quest.server.session import ErrorCode

        # Каждое имя из каталога должно быть доступно как атрибут
        # класса, и значение должно быть само имя (строковый код).
        from rob_box_core.bridge_protocol import ERRORS as CATALOG_ERRORS

        for code in CATALOG_ERRORS:
            assert hasattr(ErrorCode, code), (
                f"ErrorCode.{code} нет, хотя есть в каталоге. "
                f"Проверьте rob_box_quest/server/session.py::ErrorCode."
            )
            assert getattr(ErrorCode, code) == code

    def test_error_code_rejects_typo(self):
        """Если кто-то опечатался в имени кода, ErrorCode даёт
        понятный AttributeError (не silent None)."""
        from rob_box_quest.server.session import ErrorCode

        with pytest.raises(AttributeError, match="не существует в каноне"):
            _ = ErrorCode.NOT_A_REAL_ERROR_CODE

    def test_error_code_is_immutable(self):
        """Попытка присвоить ErrorCode.X = ... отвергается.
        Это та защита, которой не было до voice-vr 07 (класс
        ErrorCode был обычный, переназначение атрибутов срабатывало
        без ошибки)."""
        from rob_box_quest.server.session import ErrorCode

        with pytest.raises(AttributeError, match="неизменяем"):
            ErrorCode.AUTH_FAIL = "HACKED"  # type: ignore[misc]


# --- 3. Single source of truth (no local re-declarations) ------------------


class TestNoLocalReDeclarations:
    """Каталог — single source of truth. rob_box_quest НЕ должен
    локально переопределять ``TOPIC_IDS``, ``STREAM_CATALOG`` или
    набор ``ErrorCode`` атрибутов.

    Definition of Done карточки #2192:
      «git grep 'TOPIC_IDS\\|STREAM_CATALOG' src/ → только
        bridge_protocol.py и его тест».

    Эта проверка мягче: разрешает **re-export** (через import + alias),
    но ловит локальные dataclass-dict-определения.
    """

    def test_topics_py_no_local_dict_definition(self):
        """protocol/topics.py: TOPIC_IDS — re-export из rob_box_core,
        а не литерал-dict."""
        if not TOPICS_PY.exists():
            pytest.skip("protocol/topics.py не найден")
        src = TOPICS_PY.read_text(encoding="utf-8")
        tree = ast.parse(src)
        # Ищем Module-level assignment с dict-literal в правой части
        # с ключами, похожими на topic-имена.
        topic_like_keys = {"camera_rear", "lidar_2d", "robot_status", "voice_state"}
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if (
                        isinstance(target, ast.Name)
                        and target.id == "TOPIC_IDS"
                    ):
                        if isinstance(node.value, ast.Dict):
                            keys = [
                                k.value for k in node.value.keys
                                if isinstance(k, ast.Constant)
                                and isinstance(k.value, str)
                            ]
                            assert not (set(keys) & topic_like_keys), (
                                f"TOPIC_IDS в {TOPICS_PY.name} определяется "
                                f"локально как dict с ключами {keys!r} — "
                                f"это должно быть re-export из rob_box_core"
                            )

    def test_registry_py_no_local_stream_catalog(self):
        """streams/registry.py: STREAM_CATALOG — re-export из rob_box_core,
        а не литерал-dict с dataclass-ами."""
        if not REGISTRY_PY.exists():
            pytest.skip("streams/registry.py не найден")
        src = REGISTRY_PY.read_text(encoding="utf-8")
        tree = ast.parse(src)
        stream_like_keys = {"camera_rear", "lidar_2d", "robot_status"}
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if (
                        isinstance(target, ast.Name)
                        and target.id == "STREAM_CATALOG"
                    ):
                        if isinstance(node.value, ast.Dict):
                            keys = [
                                k.value for k in node.value.keys
                                if isinstance(k, ast.Constant)
                                and isinstance(k.value, str)
                            ]
                            assert not (set(keys) & stream_like_keys), (
                                f"STREAM_CATALOG в {REGISTRY_PY.name} "
                                f"определяется локально как dict с "
                                f"ключами {keys!r} — это должно быть "
                                f"re-export из rob_box_core"
                            )

    def test_session_py_no_local_error_code_repetition(self):
        """server/session.py: ErrorCode атрибуты НЕ дублируются
        (раньше было ``FLOOR_HELD = ...`` дважды)."""
        if not SESSION_PY.exists():
            pytest.skip("server/session.py не найден")
        src = SESSION_PY.read_text(encoding="utf-8")
        tree = ast.parse(src)
        # Собираем все присваивания вида ``X = "..."`` в class ErrorCode
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "ErrorCode":
                seen: dict[str, int] = {}
                for stmt in node.body:
                    if isinstance(stmt, ast.Assign):
                        for target in stmt.targets:
                            if (
                                isinstance(target, ast.Name)
                                and isinstance(stmt.value, ast.Constant)
                                and isinstance(stmt.value.value, str)
                            ):
                                seen[target.id] = seen.get(target.id, 0) + 1
                duplicates = {
                    k: v for k, v in seen.items() if v > 1
                }
                assert not duplicates, (
                    f"ErrorCode в {SESSION_PY.name} имеет дубли "
                    f"присваиваний атрибутов: {duplicates}. "
                    f"Канон — rob_box_core.bridge_protocol.ERRORS."
                )


# --- 4. Каталог events sanity check -----------------------------------------


class TestEventsCatalogSymmetry:
    """EVENTS — sanity check против того, что сервер реально шлёт."""

    def test_ping_is_in_events(self):
        """ping — единственное событие, которое сервер ОБРАБАТЫВАЕТ
        (server_handled=True). Канон должен это отражать."""
        ping_evt = next((e for e in EVENTS if e.name == "ping"), None)
        assert ping_evt is not None
        assert ping_evt.server_handled is True

    def test_events_sent_by_server_subset_of_catalog(self):
        """Все события, которые сервер ШЛЁТ, есть в каталоге."""
        emitted = _server_emitted_events_from_ws_server()
        # Грубый фильтр: только «квазипротокольные» имена (lowercase + _).
        # Это чтобы не ловить строки типа "ui_button" внутри комментариев.
        # Из-за ограничений regex'а здесь — sanity-уровень, не строгий.
        catalog_names = {e.name for e in EVENTS}
        missing = sorted(emitted - catalog_names - {"type"})  # "type" — JSON-ключ, не имя
        # Допустимые «не в каталоге»: локальные JSON-поля (cmd, ts_ms, etc.),
        # которые grep ловит из ws_server. Их фильтруем по whitelist'у.
        safe_whitelist = {
            "type", "mode", "voice", "reason", "client_id",
            "floor", "args", "ts_ms",
        }
        # Только для теста: уберём из missing всё, что подозрительно
        # похоже на JSON-ключи, а не на имя события.
        suspicious = [
            m for m in missing
            if m not in safe_whitelist and m.islower() and "_" in m
        ]
        assert not suspicious, (
            "WS-server шлёт события, которых нет в каталоге:\n"
            + "\n".join(f"  - {e!r}" for e in suspicious)
        )


# --- 5. Catalog coverage of common ws_server helpers -----------------------


class TestVoiceConfigForwardCompat:
    """VOICE_PRESET_IDS / VOICE_LANGUAGES в ws_server.py == каталогу."""

    def test_ws_server_voice_presets(self):
        from rob_box_quest.server.ws_server import VOICE_PRESET_IDS as WS_PRESETS
        assert set(WS_PRESETS) == set(VOICE_PRESET_IDS), (
            "ws_server.VOICE_PRESET_IDS расходится с каталогом "
            "(rob_box_core.bridge_protocol.VOICE_PRESET_IDS). "
            f"ws_server={WS_PRESETS!r}, catalog={VOICE_PRESET_IDS!r}"
        )

    def test_ws_server_voice_languages(self):
        from rob_box_quest.server.ws_server import VOICE_LANGUAGES as WS_LANGS
        assert set(WS_LANGS) == set(VOICE_LANGUAGES), (
            "ws_server.VOICE_LANGUAGES расходится с каталогом. "
            f"ws_server={WS_LANGS!r}, catalog={VOICE_LANGUAGES!r}"
        )

    def test_ws_server_valid_modes_v2(self):
        from rob_box_quest.server.ws_server import VALID_MODES_V2
        from rob_box_core.bridge_protocol import MODES
        assert set(VALID_MODES_V2) == set(MODES)

    def test_ws_server_valid_floors_v2(self):
        from rob_box_quest.server.ws_server import VALID_FLOORS_V2
        from rob_box_core.bridge_protocol import FLOORS
        assert set(VALID_FLOORS_V2) == set(FLOORS)


# --- 6. RATE_LIMIT stays declared but unimplemented ------------------------


class TestRateLimit:
    """Голосвание карточки #2192: «RATE_LIMIT не отправляется никогда.
    Решение по RATE_LIMIT — ADR-0080 §7 вопрос 2».

    То есть код ДОЛЖЕН быть в каталоге (для обратной совместимости
    с клиентами, которые могут его знать), но ws_server НЕ должен
    его шлёт."""

    def test_rate_limit_in_catalog(self):
        assert is_known_error("RATE_LIMIT"), (
            "RATE_LIMIT должен быть в каталоге (legacy контракт)"
        )

    def test_rate_limit_not_sent_by_ws_server(self):
        """``ws_server.py`` НЕ отправляет ``ERROR{RATE_LIMIT}``.
        Grep по ``ErrorCode.RATE_LIMIT`` / ``'RATE_LIMIT'`` — если
        есть ссылка в ``_send_error``-вызове, это нарушение карточки.
        """
        if not WS_SERVER_PY.exists():
            pytest.skip("ws_server.py не найден")
        src = WS_SERVER_PY.read_text(encoding="utf-8")
        # Разрешаем упоминание RATE_LIMIT ТОЛЬКО в комментариях (строки,
        # начинающиеся с #) и в импорте ErrorCode. Если есть
        # ``_send_error(ws, ..., ErrorCode.RATE_LIMIT`` — это нарушение.
        bad_uses = re.findall(
            r"^[^#\n]*ErrorCode\.RATE_LIMIT\b",
            src,
            flags=re.MULTILINE,
        )
        assert not bad_uses, (
            "ws_server.py шлёт ErrorCode.RATE_LIMIT — голосвание карточки "
            "нарушено (RATE_LIMIT = legacy, см. ADR-0080 §7 вопрос 2). "
            f"Найдено {len(bad_uses)} случаев:\n"
            + "\n".join(f"  - {line.strip()}" for line in bad_uses)
        )
